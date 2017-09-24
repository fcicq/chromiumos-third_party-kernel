/*
 * cros_ec_sensors_core - Common function for Chrome OS EC sensor driver.
 *
 * Copyright (C) 2015 Google, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This driver uses the cros-ec interface to communicate with the Chrome OS
 * EC about accelerometer data. Accelerometer access is presented through
 * iio sysfs.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/kernel.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/mfd/cros_ec_dev.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>

#include "cros_ec_sensors_core.h"

/*
 * Hard coded to the first device to support sensor fifo.  The EC has a 2048
 * byte fifo and will trigger an interrupt when fifo is 2/3 full.
 */
#define CROS_EC_FIFO_SIZE (2048 * 2 / 3)

static char *cros_ec_loc[] = {
	[MOTIONSENSE_LOC_BASE] = "base",
	[MOTIONSENSE_LOC_LID] = "lid",
	[MOTIONSENSE_LOC_MAX] = "unknown",
};

static void get_default_min_max_freq_and_fifo_size(enum motionsensor_type type,
	uint32_t *min_freq, uint32_t *max_freq, uint32_t *max_fifo_events)
{
	/* we don't know fifo size, set to size previously used by sensor HAL */
	*max_fifo_events = CROS_EC_FIFO_SIZE;

	switch (type) {
	case MOTIONSENSE_TYPE_ACCEL:
		*min_freq = 12500;
		*max_freq = 100000;
		break;
	case MOTIONSENSE_TYPE_GYRO:
		*min_freq = 25000;
		*max_freq = 100000;
		break;
	case MOTIONSENSE_TYPE_MAG:
		*min_freq = 5000;
		*max_freq = 25000;
		break;
	case MOTIONSENSE_TYPE_PROX:
	case MOTIONSENSE_TYPE_LIGHT:
		*min_freq = 100;
		*max_freq = 50000;
		break;
	case MOTIONSENSE_TYPE_BARO:
		*min_freq = 250;
		*max_freq = 20000;
		break;
	case MOTIONSENSE_TYPE_ACTIVITY:
	default:
		*max_fifo_events = 0;
		*min_freq = 0;
		*max_freq = 0;
		break;
	}
}

static int cros_ec_get_host_cmd_version_mask(
				struct cros_ec_sensors_core_state *state,
				u16 cmd, u32 *mask)
{
	struct ec_params_get_cmd_versions pver;
	struct ec_response_get_cmd_versions rver;
	struct cros_ec_command msg = {
		.command = EC_CMD_GET_CMD_VERSIONS,
		.version = 0,
		.outdata = (u8 *)&pver,
		.outsize = sizeof(pver),
		.indata = (u8 *)&rver,
		.insize = sizeof(rver),
	};
	int ret;

	pver.cmd = cmd;
	ret = cros_ec_cmd_xfer_status(state->ec, &msg);
	if (ret >= 0) {
		if (msg.result == EC_RES_SUCCESS)
			*mask = rver.version_mask;
		else
			*mask = 0;
	}
	return ret;
}

/*
 * cros_ec_sensors_core_init
 *
 * Initialize core sensor strucure, fill the response are
 * with the return of Sensor info.
 *
 * @pdev plarform device created for the sensors
 * @indio_dev iio device structure of the device
 * @physical_device True if the device refers to a physical device.
 */
int cros_ec_sensors_core_init(struct platform_device *pdev,
			      struct iio_dev *indio_dev,
			      bool physical_device)
{
	struct device *dev = &pdev->dev;
	struct cros_ec_sensors_core_state *state = iio_priv(indio_dev);
	struct cros_ec_dev *ec = dev_get_drvdata(pdev->dev.parent);
	struct cros_ec_sensor_platform *sensor_platform = dev_get_platdata(dev);
	u32 ver_mask = 0;
	int ret = 0;

	platform_set_drvdata(pdev, indio_dev);

	state->ec = ec->ec_dev;
	state->resp = devm_kzalloc(&pdev->dev, state->ec->max_response,
			GFP_KERNEL);
	if (!state->resp)
		return -ENOMEM;

	mutex_init(&state->cmd_lock);

	/* determine what version of MOTIONSENSE CMD EC has */
	ret = cros_ec_get_host_cmd_version_mask(state,
						EC_CMD_MOTION_SENSE_CMD,
						&ver_mask);
	if (ret < 0 || ver_mask == 0) {
		dev_warn(dev, "Motionsense cmd version too old, aborting...\n");
		return -ENODEV;
	}

	/* Set up the host command structure. */
	state->msg.version = fls(ver_mask) - 1;
	state->msg.command = EC_CMD_MOTION_SENSE_CMD + ec->cmd_offset;
	state->msg.outdata = (u8 *)&state->param;
	state->msg.outsize = sizeof(struct ec_params_motion_sense);
	state->msg.indata = (u8 *)state->resp;

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->name;

	if (physical_device) {
		indio_dev->modes = INDIO_DIRECT_MODE;

		state->param.cmd = MOTIONSENSE_CMD_INFO;
		state->param.info.sensor_num = sensor_platform->sensor_num;
		if (cros_ec_motion_send_host_cmd(state, 0)) {
			dev_warn(dev, "Can not access sensor info\n");
			return -EIO;
		}
		state->type = state->resp->info.type;
		state->loc = state->resp->info.location;
		if (state->msg.version < 3) {
			get_default_min_max_freq_and_fifo_size(
					state->resp->info.type,
					&state->min_freq,
					&state->max_freq,
					&state->fifo_max_event_count);
		} else {
			state->min_freq =
				state->resp->info_3.min_frequency;
			state->max_freq =
				state->resp->info_3.max_frequency;
			state->fifo_max_event_count =
				state->resp->info_3.fifo_max_event_count;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(cros_ec_sensors_core_init);

/*
 * cros_ec_motion_send_host_cmd - send motion sense host command
 *
 * @st Pointer to state information for device.
 * @return 0 if ok, -ve on error.
 *
 * Note, when called, the sub-command is assumed to be set in param->cmd.
 */
int cros_ec_motion_send_host_cmd(struct cros_ec_sensors_core_state *state,
				 u16 opt_length)
{
	int ret;

	if (opt_length != 0)
		state->msg.insize = min(opt_length, state->ec->max_response);
	else
		state->msg.insize = state->ec->max_response;

	/* Send host command. */
	ret = cros_ec_cmd_xfer_status(state->ec, &state->msg);

	/* Send host command. */
	if (ret >= 0)
		return 0;
	else
		return -EIO;
}
EXPORT_SYMBOL_GPL(cros_ec_motion_send_host_cmd);

static ssize_t __maybe_unused cros_ec_sensors_flush(struct iio_dev *indio_dev,
		uintptr_t private, const struct iio_chan_spec *chan,
		const char *buf, size_t len)
{
	struct cros_ec_sensors_core_state *st = iio_priv(indio_dev);
	int ret = 0;
	bool flush;

	ret = strtobool(buf, &flush);
	if (ret < 0)
		return ret;
	if (!flush)
		return -EINVAL;

	mutex_lock(&st->cmd_lock);
	st->param.cmd = MOTIONSENSE_CMD_FIFO_FLUSH;
	ret = cros_ec_motion_send_host_cmd(st, 0);
	if (ret != 0)
		dev_warn(&indio_dev->dev, "Unable to flush sensor\n");
	mutex_unlock(&st->cmd_lock);
	return ret ? ret : len;
}

static ssize_t cros_ec_sensors_calibrate(struct iio_dev *indio_dev,
		uintptr_t private, const struct iio_chan_spec *chan,
		const char *buf, size_t len)
{
	struct cros_ec_sensors_core_state *st = iio_priv(indio_dev);
	int ret, i;
	bool calibrate;

	ret = strtobool(buf, &calibrate);
	if (ret < 0)
		return ret;
	if (!calibrate)
		return -EINVAL;

	mutex_lock(&st->cmd_lock);
	st->param.cmd = MOTIONSENSE_CMD_PERFORM_CALIB;
	ret = cros_ec_motion_send_host_cmd(st, 0);
	if (ret != 0) {
		dev_warn(&indio_dev->dev, "Unable to calibrate sensor\n");
	} else {
		/* Save values */
		for (i = X; i < MAX_AXIS; i++)
			st->calib[i].offset = st->resp->perform_calib.offset[i];
	}
	mutex_unlock(&st->cmd_lock);
	return ret ? ret : len;
}

static ssize_t cros_ec_sensors_loc(struct iio_dev *indio_dev,
		uintptr_t private, const struct iio_chan_spec *chan,
		char *buf)
{
	struct cros_ec_sensors_core_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%s\n", cros_ec_loc[st->loc]);
}

#define DEVICE_STATE_INT_ATTR(_name, _var) \
static ssize_t cros_ec_sensors_##_name(struct iio_dev *indio_dev, \
		uintptr_t private, const struct iio_chan_spec *chan, \
		char *buf) \
{ \
	struct cros_ec_sensors_core_state *st = iio_priv(indio_dev); \
	return sprintf(buf, "%d\n", st->_var); \
}

DEVICE_STATE_INT_ATTR(id, param.info.sensor_num);
DEVICE_STATE_INT_ATTR(min_freq, min_freq);
DEVICE_STATE_INT_ATTR(max_freq, max_freq);
DEVICE_STATE_INT_ATTR(max_events, fifo_max_event_count);

const struct iio_chan_spec_ext_info cros_ec_sensors_ext_info[] = {
#if IS_ENABLED(CONFIG_IIO_CROS_EC_SENSORS_RING)
	{
		.name = "flush",
		.shared = IIO_SHARED_BY_ALL,
		.write = cros_ec_sensors_flush
	},
#endif
	{
		.name = "calibrate",
		.shared = IIO_SHARED_BY_ALL,
		.write = cros_ec_sensors_calibrate
	},
	{
		.name = "id",
		.shared = IIO_SHARED_BY_ALL,
		.read = cros_ec_sensors_id
	},
	{
		.name = "location",
		.shared = IIO_SHARED_BY_ALL,
		.read = cros_ec_sensors_loc
	},
	{
		.name = "min_frequency",
		.shared = IIO_SHARED_BY_ALL,
		.read = cros_ec_sensors_min_freq
	},
	{
		.name = "max_frequency",
		.shared = IIO_SHARED_BY_ALL,
		.read = cros_ec_sensors_max_freq
	},
	{
		.name = "max_events",
		.shared = IIO_SHARED_BY_ALL,
		.read = cros_ec_sensors_max_events
	},
	{ },
};
EXPORT_SYMBOL_GPL(cros_ec_sensors_ext_info);

const struct iio_chan_spec_ext_info cros_ec_sensors_limited_info[] = {
	{
		.name = "id",
		.shared = IIO_SHARED_BY_ALL,
		.read = cros_ec_sensors_id
	},
	{
		.name = "location",
		.shared = IIO_SHARED_BY_ALL,
		.read = cros_ec_sensors_loc
	},
	{ },
};
EXPORT_SYMBOL_GPL(cros_ec_sensors_limited_info);
/*
 * idx_to_reg - convert sensor index into offset in shared memory region.
 *
 * @st: private data
 * @idx: sensor index (should be element of enum sensor_index)
 * @return address to read at.
 */
static unsigned idx_to_reg(struct cros_ec_sensors_core_state *st, unsigned idx)
{
	/*
	 * When using LPC interface, only space for 2 Accel and one Gyro.
	 * First halfword of MOTIONSENSE_TYPE_ACCEL is used by angle.
	 */
	if (st->type == MOTIONSENSE_TYPE_ACCEL)
		return EC_MEMMAP_ACC_DATA + sizeof(u16) *
			(1 + idx + st->param.info.sensor_num *
			 MAX_AXIS);
	else
		return EC_MEMMAP_GYRO_DATA + sizeof(u16) * idx;
}

/*
 * read_ec_until_not_busy - read from EC status byte until it reads not busy.
 *
 * @st Pointer to state information for device.
 * @return 8-bit status if ok, -ve on error
 */
static int read_ec_until_not_busy(struct cros_ec_sensors_core_state *st)
{
	struct cros_ec_device *ec = st->ec;
	u8 status;
	int attempts = 0;

	ec->cmd_read_u8(ec, EC_MEMMAP_ACC_STATUS, &status);
	while (status & EC_MEMMAP_ACC_STATUS_BUSY_BIT) {
		/* Give up after enough attempts, return error. */
		if (attempts++ >= 50)
			return -EIO;

		/* Small delay every so often. */
		if (attempts % 5 == 0)
			msleep(25);

		ec->cmd_read_u8(ec, EC_MEMMAP_ACC_STATUS, &status);
	}

	return status;
}

/*
 * read_ec_sensors_data_unsafe - read acceleration data from EC shared memory.
 *
 * @st Pointer to state information for device.
 * @scan_mask Bitmap of the sensor indices to scan.
 * @data Location to store data.
 *
 * Note this is the unsafe function for reading the EC data. It does not
 * guarantee that the EC will not modify the data as it is being read in.
 */
static void read_ec_sensors_data_unsafe(struct iio_dev *indio_dev,
			 unsigned long scan_mask, s16 *data)
{
	struct cros_ec_sensors_core_state *st = iio_priv(indio_dev);
	struct cros_ec_device *ec = st->ec;
	unsigned i = 0;

	/*
	 * Read all sensors enabled in scan_mask. Each value is 2
	 * bytes.
	 */
	for_each_set_bit(i, &scan_mask, indio_dev->masklength) {
		ec->cmd_read_u16(ec, idx_to_reg(st, i), data);
		data++;
	}
}

/*
 * cros_ec_sensors_read_lpc - read acceleration data from EC shared memory.
 *
 * @st Pointer to state information for device.
 * @scan_mask Bitmap of the sensor indices to scan.
 * @data Location to store data.
 * @return 0 if ok, -ve on error
 *
 * Note: this is the safe function for reading the EC data. It guarantees
 * that the data sampled was not modified by the EC while being read.
 */
int cros_ec_sensors_read_lpc(struct iio_dev *indio_dev,
			     unsigned long scan_mask, s16 *data)
{
	struct cros_ec_sensors_core_state *st = iio_priv(indio_dev);
	struct cros_ec_device *ec = st->ec;
	u8 samp_id = 0xff, status = 0;
	int attempts = 0;

	/*
	 * Continually read all data from EC until the status byte after
	 * all reads reflects that the EC is not busy and the sample id
	 * matches the sample id from before all reads. This guarantees
	 * that data read in was not modified by the EC while reading.
	 */
	while ((status & (EC_MEMMAP_ACC_STATUS_BUSY_BIT |
			  EC_MEMMAP_ACC_STATUS_SAMPLE_ID_MASK)) != samp_id) {
		/* If we have tried to read too many times, return error. */
		if (attempts++ >= 5)
			return -EIO;

		/* Read status byte until EC is not busy. */
		status = read_ec_until_not_busy(st);
		if (status < 0)
			return status;

		/*
		 * Store the current sample id so that we can compare to the
		 * sample id after reading the data.
		 */
		samp_id = status & EC_MEMMAP_ACC_STATUS_SAMPLE_ID_MASK;

		/* Read all EC data, format it, and store it into data. */
		read_ec_sensors_data_unsafe(indio_dev, scan_mask, data);

		/* Read status byte. */
		ec->cmd_read_u8(ec, EC_MEMMAP_ACC_STATUS, &status);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cros_ec_sensors_read_lpc);

int cros_ec_sensors_read_cmd(struct iio_dev *indio_dev,
			     unsigned long scan_mask, s16 *data)
{
	struct cros_ec_sensors_core_state *st = iio_priv(indio_dev);
	int ret;
	unsigned i = 0;

	/*
	 * read all sensor data through a command.
	 */
	st->param.cmd = MOTIONSENSE_CMD_DATA;
	ret = cros_ec_motion_send_host_cmd(st, sizeof(st->resp->data));
	if (ret != 0) {
		dev_warn(&indio_dev->dev, "Unable to read sensor data\n");
		return ret;
	}

	for_each_set_bit(i, &scan_mask, indio_dev->masklength) {
		*data = st->resp->data.data[i];
		data++;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(cros_ec_sensors_read_cmd);

irqreturn_t cros_ec_sensors_capture(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct cros_ec_sensors_core_state *st = iio_priv(indio_dev);

	mutex_lock(&st->cmd_lock);
	/* Clear capture data. */
	memset(st->samples, 0, indio_dev->scan_bytes);

	/* Read data based on which channels are enabled in scan mask. */
	st->read_ec_sensors_data(indio_dev, *(indio_dev->active_scan_mask),
			   (s16 *)st->samples);

	/* Store the timestamp last 8 bytes of data. */
	if (indio_dev->scan_timestamp)
		*(s64 *)&st->samples[round_down(indio_dev->scan_bytes -
						sizeof(s64),
				     sizeof(s64))] = iio_get_time_ns();

	iio_push_to_buffers(indio_dev, st->samples);

	/*
	 * Tell the core we are done with this trigger and ready for the
	 * next one.
	 */
	iio_trigger_notify_done(indio_dev->trig);
	mutex_unlock(&st->cmd_lock);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(cros_ec_sensors_capture);

int cros_ec_sensors_core_read(struct cros_ec_sensors_core_state *st,
			  struct iio_chan_spec const *chan,
			  int *val, int *val2, long mask)
{
	int ret = IIO_VAL_INT;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		st->param.cmd = MOTIONSENSE_CMD_EC_RATE;
		st->param.ec_rate.data =
			EC_MOTION_SENSE_NO_VALUE;

		if (cros_ec_motion_send_host_cmd(st, 0))
			ret = -EIO;
		else
			*val = st->resp->ec_rate.ret;
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		st->param.cmd = MOTIONSENSE_CMD_SENSOR_ODR;
		st->param.sensor_odr.data =
			EC_MOTION_SENSE_NO_VALUE;

		if (cros_ec_motion_send_host_cmd(st, 0))
			ret = -EIO;
		else
			*val = st->resp->sensor_odr.ret;
		break;
	default:
		break;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(cros_ec_sensors_core_read);

int cros_ec_sensors_core_write(struct cros_ec_sensors_core_state *st,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		st->param.cmd = MOTIONSENSE_CMD_SENSOR_ODR;
		st->param.sensor_odr.data = val;

		/* Always roundup, so caller gets at least what it asks for. */
		st->param.sensor_odr.roundup = 1;

		if (cros_ec_motion_send_host_cmd(st, 0))
			ret = -EIO;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		st->param.cmd = MOTIONSENSE_CMD_EC_RATE;
		st->param.ec_rate.data = val;

		if (cros_ec_motion_send_host_cmd(st, 0))
			ret = -EIO;
		else
			st->curr_sampl_freq = val;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(cros_ec_sensors_core_write);

static int __maybe_unused cros_ec_sensors_prepare(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct cros_ec_sensors_core_state *st = iio_priv(indio_dev);

	if (st->curr_sampl_freq == 0)
		return 0;

	/*
	 * If the sensors are sampled at high frequency, we will not be able to
	 * sleep. Set to sampling to a long period if necessary.
	 */
	if (st->curr_sampl_freq < CROS_EC_MIN_SUSPEND_SAMPLING_FREQUENCY) {
		mutex_lock(&st->cmd_lock);
		st->param.cmd = MOTIONSENSE_CMD_EC_RATE;
		st->param.ec_rate.data = CROS_EC_MIN_SUSPEND_SAMPLING_FREQUENCY;
		cros_ec_motion_send_host_cmd(st, 0);
		mutex_unlock(&st->cmd_lock);
	}
	return 0;
}

static void __maybe_unused cros_ec_sensors_complete(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct cros_ec_sensors_core_state *st = iio_priv(indio_dev);

	if (st->curr_sampl_freq == 0)
		return;

	if (st->curr_sampl_freq < CROS_EC_MIN_SUSPEND_SAMPLING_FREQUENCY) {
		mutex_lock(&st->cmd_lock);
		st->param.cmd = MOTIONSENSE_CMD_EC_RATE;
		st->param.ec_rate.data = st->curr_sampl_freq;
		cros_ec_motion_send_host_cmd(st, 0);
		mutex_unlock(&st->cmd_lock);
	}
}

#ifdef CONFIG_PM_SLEEP
const struct dev_pm_ops cros_ec_sensors_pm_ops = {
	.prepare = cros_ec_sensors_prepare,
	.complete = cros_ec_sensors_complete
};
#else
const struct dev_pm_ops cros_ec_sensors_pm_ops = { };
#endif
EXPORT_SYMBOL_GPL(cros_ec_sensors_pm_ops);


MODULE_DESCRIPTION("ChromeOS EC sensor hub core functions");
MODULE_LICENSE("GPL v2");
