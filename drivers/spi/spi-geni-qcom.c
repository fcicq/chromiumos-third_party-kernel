// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017-2018, The Linux foundation. All rights reserved.

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/qcom-geni-se.h>
#include <linux/spi/spi.h>

/* SPI SE specific registers and respective register fields */
#define SE_SPI_CPHA		0x224
#define CPHA			BIT(0)

#define SE_SPI_LOOPBACK		0x22c
#define LOOPBACK_ENABLE		0x1
#define NORMAL_MODE		0x0
#define LOOPBACK_MSK		GENMASK(1, 0)

#define SE_SPI_CPOL		0x230
#define CPOL			BIT(2)

#define SE_SPI_DEMUX_OUTPUT_INV	0x24c
#define CS_DEMUX_OUTPUT_INV_MSK	GENMASK(3, 0)

#define SE_SPI_DEMUX_SEL	0x250
#define CS_DEMUX_OUTPUT_SEL	GENMASK(3, 0)

#define SE_SPI_TRANS_CFG	0x25c
#define CS_TOGGLE		BIT(0)

#define SE_SPI_WORD_LEN		0x268
#define WORD_LEN_MSK		GENMASK(9, 0)
#define MIN_WORD_LEN		4

#define SE_SPI_TX_TRANS_LEN	0x26c
#define SE_SPI_RX_TRANS_LEN	0x270
#define TRANS_LEN_MSK		GENMASK(23, 0)

#define SE_SPI_PRE_POST_CMD_DLY	0x274

#define SE_SPI_DELAY_COUNTERS	0x278
#define SPI_INTER_WORDS_DELAY_MSK	GENMASK(9, 0)
#define SPI_CS_CLK_DELAY_MSK		GENMASK(19, 10)
#define SPI_CS_CLK_DELAY_SHFT		10

/* M_CMD OP codes for SPI */
#define SPI_TX_ONLY		1
#define SPI_RX_ONLY		2
#define SPI_FULL_DUPLEX		3
#define SPI_TX_RX		7
#define SPI_CS_ASSERT		8
#define SPI_CS_DEASSERT		9
#define SPI_SCK_ONLY		10
/* M_CMD params for SPI */
#define SPI_PRE_CMD_DELAY	BIT(0)
#define TIMESTAMP_BEFORE	BIT(1)
#define FRAGMENTATION		BIT(2)
#define TIMESTAMP_AFTER		BIT(3)
#define POST_CMD_DELAY		BIT(4)

static irqreturn_t geni_spi_isr(int irq, void *data);

struct spi_geni_master {
	struct geni_se se;
	struct device *dev;
	unsigned int rx_fifo_depth;
	unsigned int tx_fifo_depth;
	unsigned int tx_fifo_width;
	unsigned int tx_wm;
	bool setup;
	unsigned int cur_speed_hz;
	unsigned int cur_word_len;
	unsigned int tx_rem_bytes;
	unsigned int rx_rem_bytes;
	struct spi_transfer *cur_xfer;
	struct completion xfer_done;
	unsigned int oversampling;
};

static int get_spi_clk_cfg(unsigned int speed_hz,
			struct spi_geni_master *mas,
			unsigned int *clk_idx,
			unsigned int *clk_div)
{
	unsigned int actual_hz;
	unsigned long sclk_freq;
	struct geni_se *se = &mas->se;
	int ret;

	ret = geni_se_clk_freq_match(&mas->se,
				speed_hz * mas->oversampling, clk_idx,
				&sclk_freq, false);
	if (ret) {
		dev_err(mas->dev, "%s: Failed(%d) to find src clk for %dHz\n",
						__func__, ret, speed_hz);
		return ret;
	}
	*clk_div = DIV_ROUND_UP(sclk_freq, mas->oversampling * speed_hz);
	actual_hz = sclk_freq / (mas->oversampling * *clk_div);

	dev_dbg(mas->dev, "%s: clk %u=>%u sclk %lu, idx %d, div %d\n",
		__func__, speed_hz, actual_hz, sclk_freq, *clk_idx, *clk_div);
	ret = clk_set_rate(se->clk, sclk_freq);
	if (ret)
		dev_err(mas->dev, "%s: clk_set_rate failed %d\n",
							__func__, ret);
	return ret;
}

static void spi_setup_word_len(struct spi_geni_master *mas, u16 mode,
					unsigned int bits_per_word)
{
	unsigned int word_len, pack_words;
	bool msb_first = (mode & SPI_LSB_FIRST) ? false : true;
	struct geni_se *se = &mas->se;

	word_len = readl_relaxed(se->base + SE_SPI_WORD_LEN);

	/*
	 * If bits_per_word isn't a byte aligned value, set the packing to be
	 * 1 SPI word per FIFO word.
	 */
	if (!(mas->tx_fifo_width % bits_per_word))
		pack_words = mas->tx_fifo_width / bits_per_word;
	else
		pack_words = 1;
	word_len &= ~WORD_LEN_MSK;
	word_len |= ((bits_per_word - MIN_WORD_LEN) & WORD_LEN_MSK);
	geni_se_config_packing(&mas->se, bits_per_word, pack_words, msb_first,
								true, true);
	writel_relaxed(word_len, se->base + SE_SPI_WORD_LEN);
}

static int setup_fifo_params(struct spi_device *spi_slv,
					struct spi_master *spi)
{
	struct spi_geni_master *mas = spi_master_get_devdata(spi);
	struct geni_se *se = &mas->se;
	unsigned int mode = spi_slv->mode;
	unsigned int loopback_cfg = readl_relaxed(se->base + SE_SPI_LOOPBACK);
	unsigned int cpol = readl_relaxed(se->base + SE_SPI_CPOL);
	unsigned int cpha = readl_relaxed(se->base + SE_SPI_CPHA);
	unsigned int demux_sel, clk_sel, m_clk_cfg, idx, div;
	int ret = 0;
	unsigned int demux_output_inv = 0;

	loopback_cfg &= ~LOOPBACK_MSK;
	cpol &= ~CPOL;
	cpha &= ~CPHA;

	if (mode & SPI_LOOP)
		loopback_cfg |= LOOPBACK_ENABLE;

	if (mode & SPI_CPOL)
		cpol |= CPOL;

	if (mode & SPI_CPHA)
		cpha |= CPHA;

	if (spi_slv->mode & SPI_CS_HIGH)
		demux_output_inv = BIT(spi_slv->chip_select);

	demux_sel = spi_slv->chip_select;
	mas->cur_speed_hz = spi_slv->max_speed_hz;
	mas->cur_word_len = spi_slv->bits_per_word;

	ret = get_spi_clk_cfg(mas->cur_speed_hz, mas, &idx, &div);
	if (ret) {
		dev_err(mas->dev, "Err setting clks ret(%d) for %d\n",
							ret, mas->cur_speed_hz);
		return ret;
	}

	clk_sel = (idx & CLK_SEL_MSK);
	m_clk_cfg = ((div << CLK_DIV_SHFT) | SER_CLK_EN);
	spi_setup_word_len(mas, spi_slv->mode, spi_slv->bits_per_word);
	writel_relaxed(loopback_cfg, se->base + SE_SPI_LOOPBACK);
	writel_relaxed(demux_sel, se->base + SE_SPI_DEMUX_SEL);
	writel_relaxed(cpha, se->base + SE_SPI_CPHA);
	writel_relaxed(cpol, se->base + SE_SPI_CPOL);
	writel_relaxed(demux_output_inv, se->base + SE_SPI_DEMUX_OUTPUT_INV);
	writel_relaxed(clk_sel, se->base + SE_GENI_CLK_SEL);
	writel_relaxed(m_clk_cfg, se->base + GENI_SER_M_CLK_CFG);
	return 0;
}

static int spi_geni_prepare_message(struct spi_master *spi,
					struct spi_message *spi_msg)
{
	int ret = 0;
	struct spi_geni_master *mas = spi_master_get_devdata(spi);
	struct geni_se *se = &mas->se;

	geni_se_select_mode(se, GENI_SE_FIFO);
	reinit_completion(&mas->xfer_done);
	ret = setup_fifo_params(spi_msg->spi, spi);
	if (ret) {
		dev_err(mas->dev, "%s: Couldn't select mode %d", __func__, ret);
		ret = -EINVAL;
	}
	return ret;
}

static int spi_geni_prepare_transfer_hardware(struct spi_master *spi)
{
	struct spi_geni_master *mas = spi_master_get_devdata(spi);
	struct geni_se *se = &mas->se;

	if (!mas->setup) {
		unsigned int proto = geni_se_read_proto(se);
		unsigned int major, minor, step, ver;

		if (proto != GENI_SE_SPI) {
			dev_err(mas->dev, "Invalid proto %d\n", proto);
			return -ENXIO;
		}
		mas->tx_fifo_depth = geni_se_get_tx_fifo_depth(se);
		mas->rx_fifo_depth = geni_se_get_rx_fifo_depth(se);
		mas->tx_fifo_width = geni_se_get_tx_fifo_width(se);

		/*
		 * Hardware programming guide suggests to configure
		 * RX FIFO RFR level to fifo_depth-2.
		 */
		geni_se_init(se, 0x0, mas->tx_fifo_depth - 2);
		mas->oversampling = 1;
		/* Transmit an entire FIFO worth of data per IRQ */
		mas->tx_wm = 1;
		ver = geni_se_get_qup_hw_version(se);
		major = GENI_SE_VERSION_MAJOR(ver);
		minor = GENI_SE_VERSION_MINOR(ver);
		step = GENI_SE_VERSION_STEP(ver);

		if (major == 1 && minor == 0)
			mas->oversampling = 2;
		mas->setup = 1;
	}
	return 0;
}

static void setup_fifo_xfer(struct spi_transfer *xfer,
				struct spi_geni_master *mas, u16 mode,
				struct spi_master *spi)
{
	unsigned int m_cmd = 0;
	unsigned int m_param = 0;
	struct geni_se *se = &mas->se;
	unsigned int spi_tx_cfg = readl_relaxed(se->base + SE_SPI_TRANS_CFG);
	unsigned int trans_len = 0;

	if (xfer->bits_per_word != mas->cur_word_len) {
		spi_setup_word_len(mas, mode, xfer->bits_per_word);
		mas->cur_word_len = xfer->bits_per_word;
	}

	/* Speed and bits per word can be overridden per transfer */
	if (xfer->speed_hz != mas->cur_speed_hz) {
		int ret = 0;
		unsigned int clk_sel = 0;
		unsigned int m_clk_cfg = 0;
		unsigned int idx = 0;
		unsigned int div = 0;

		ret = get_spi_clk_cfg(xfer->speed_hz, mas, &idx, &div);
		if (ret) {
			dev_err(mas->dev, "%s:Err setting clks:%d\n",
								__func__, ret);
			return;
		}
		/*
		 * SPI core clock gets configured with the requested frequency
		 * or the frequency closer to the requested frequency.
		 * For that reason requested frequency is stored in the
		 * cur_speed_hz and referred in the consicutive transfer instead
		 * of calling clk_get_rate() API.
		 */
		mas->cur_speed_hz = xfer->speed_hz;
		clk_sel |= (idx & CLK_SEL_MSK);
		m_clk_cfg |= ((div << CLK_DIV_SHFT) | SER_CLK_EN);
		writel_relaxed(clk_sel, se->base + SE_GENI_CLK_SEL);
		writel_relaxed(m_clk_cfg, se->base + GENI_SER_M_CLK_CFG);
	}

	mas->tx_rem_bytes = 0;
	mas->rx_rem_bytes = 0;
	if (xfer->tx_buf && xfer->rx_buf)
		m_cmd = SPI_FULL_DUPLEX;
	else if (xfer->tx_buf)
		m_cmd = SPI_TX_ONLY;
	else if (xfer->rx_buf)
		m_cmd = SPI_RX_ONLY;

	spi_tx_cfg &= ~CS_TOGGLE;
	if (!(mas->cur_word_len % MIN_WORD_LEN)) {
		trans_len =
			((xfer->len * BITS_PER_BYTE) /
					mas->cur_word_len) & TRANS_LEN_MSK;
	} else {
		unsigned int bytes_per_word =
			(mas->cur_word_len / BITS_PER_BYTE) + 1;

		trans_len = (xfer->len / bytes_per_word) & TRANS_LEN_MSK;
	}

	/*
	 * If CS change flag is set, then toggle the CS line in between
	 * transfers and keep CS asserted after the last transfer.
	 * Else if keep CS flag asserted in between transfers and de-assert
	 * CS after the last message.
	 */
	if (xfer->cs_change) {
		if (list_is_last(&xfer->transfer_list,
				&spi->cur_msg->transfers))
			m_param |= FRAGMENTATION;
	} else {
		if (!list_is_last(&xfer->transfer_list,
				&spi->cur_msg->transfers))
			m_param |= FRAGMENTATION;
	}

	mas->cur_xfer = xfer;
	if (m_cmd & SPI_TX_ONLY) {
		mas->tx_rem_bytes = xfer->len;
		writel_relaxed(trans_len, se->base + SE_SPI_TX_TRANS_LEN);
	}

	if (m_cmd & SPI_RX_ONLY) {
		writel_relaxed(trans_len, se->base + SE_SPI_RX_TRANS_LEN);
		mas->rx_rem_bytes = xfer->len;
	}
	writel_relaxed(spi_tx_cfg, se->base + SE_SPI_TRANS_CFG);
	geni_se_setup_m_cmd(se, m_cmd, m_param);
	if (m_cmd & SPI_TX_ONLY)
		writel_relaxed(mas->tx_wm, se->base + SE_GENI_TX_WATERMARK_REG);
}

static void handle_fifo_timeout(struct spi_master *spi,
				struct spi_message *msg)
{
	struct spi_geni_master *mas = spi_master_get_devdata(spi);
	unsigned long timeout;
	struct geni_se *se = &mas->se;

	reinit_completion(&mas->xfer_done);
	geni_se_cancel_m_cmd(se);
	writel_relaxed(0, se->base + SE_GENI_TX_WATERMARK_REG);
	timeout = wait_for_completion_timeout(&mas->xfer_done, HZ);
	if (!timeout) {
		reinit_completion(&mas->xfer_done);
		geni_se_abort_m_cmd(se);
		timeout = wait_for_completion_timeout(&mas->xfer_done,
								HZ);
		if (!timeout)
			dev_err(mas->dev,
				"Failed to cancel/abort m_cmd\n");
	}
}

static int spi_geni_transfer_one(struct spi_master *spi,
				struct spi_device *slv,
				struct spi_transfer *xfer)
{
	struct spi_geni_master *mas = spi_master_get_devdata(spi);

	setup_fifo_xfer(xfer, mas, slv->mode, spi);
	return 1;
}

static irqreturn_t geni_spi_handle_tx(struct spi_geni_master *mas)
{
	unsigned int i = 0;
	unsigned int tx_fifo_width = mas->tx_fifo_width / BITS_PER_BYTE;
	unsigned int max_bytes;
	const u8 *tx_buf;
	struct geni_se *se = &mas->se;

	if (!mas->cur_xfer)
		return IRQ_NONE;

	/*
	 * For non-byte aligned bits-per-word values(e.g 9).
	 * The FIFO packing is set to 1 SPI word per FIFO word.
	 * Assumption is that each SPI word will be accomodated in
	 * ceil (bits_per_word / bits_per_byte)
	 * and the next SPI word starts at the next byte.
	 * In such cases, we can fit 1 SPI word per FIFO word so adjust the
	 * max byte that can be sent per IRQ accordingly.
	 */
	max_bytes = (mas->tx_fifo_depth - mas->tx_wm);
	if (mas->tx_fifo_width % mas->cur_word_len)
		max_bytes *= ((mas->cur_word_len / BITS_PER_BYTE) + 1);
	else
		max_bytes *= tx_fifo_width;
	tx_buf = mas->cur_xfer->tx_buf;
	tx_buf += mas->cur_xfer->len - mas->tx_rem_bytes;
	if (mas->tx_rem_bytes < max_bytes)
		max_bytes = mas->tx_rem_bytes;
	while (i < max_bytes) {
		unsigned int j;
		unsigned int fifo_word = 0;
		u8 *fifo_byte;
		unsigned int bytes_per_fifo = tx_fifo_width;
		unsigned int bytes_to_write = 0;

		if (mas->tx_fifo_width % mas->cur_word_len)
			bytes_per_fifo =
				(mas->cur_word_len / BITS_PER_BYTE) + 1;

		if (bytes_per_fifo < (max_bytes - i))
			bytes_to_write = bytes_per_fifo;
		else
			bytes_to_write = max_bytes - i;

		fifo_byte = (u8 *)&fifo_word;
		for (j = 0; j < bytes_to_write; j++)
			fifo_byte[j] = tx_buf[i++];
		iowrite32_rep(se->base + SE_GENI_TX_FIFOn, &fifo_word, 1);
	}
	mas->tx_rem_bytes -= max_bytes;
	if (!mas->tx_rem_bytes)
		writel_relaxed(0, se->base + SE_GENI_TX_WATERMARK_REG);
	return IRQ_HANDLED;
}

static irqreturn_t geni_spi_handle_rx(struct spi_geni_master *mas)
{
	unsigned int i = 0;
	struct geni_se *se = &mas->se;
	unsigned int fifo_width = mas->tx_fifo_width / BITS_PER_BYTE;
	unsigned int rx_fifo_status;
	unsigned int rx_bytes = 0;
	unsigned int rx_wc = 0;
	u8 *rx_buf;

	if (!mas->cur_xfer)
		return IRQ_NONE;

	rx_fifo_status = readl_relaxed(se->base + SE_GENI_RX_FIFO_STATUS);
	rx_buf = mas->cur_xfer->rx_buf;
	rx_wc = rx_fifo_status & RX_FIFO_WC_MSK;
	if (rx_fifo_status & RX_LAST) {
		unsigned int rx_last_byte_valid =
			(rx_fifo_status & RX_LAST_BYTE_VALID_MSK)
					>> RX_LAST_BYTE_VALID_SHFT;
		if (rx_last_byte_valid && (rx_last_byte_valid < 4)) {
			rx_wc -= 1;
			rx_bytes += rx_last_byte_valid;
		}
	}

	/*
	 * For non-byte aligned bits-per-word values. (e.g 9)
	 * The FIFO packing is set to 1 SPI word per FIFO word.
	 * Assumption is that each SPI word will be accomodated in
	 * ceil (bits_per_word / bits_per_byte)
	 * and the next SPI word starts at the next byte.
	 */
	if (!(mas->tx_fifo_width % mas->cur_word_len))
		rx_bytes += rx_wc * fifo_width;
	else
		rx_bytes += rx_wc *
			((mas->cur_word_len / BITS_PER_BYTE) + 1);
	if (mas->rx_rem_bytes < rx_bytes)
		rx_bytes = mas->rx_rem_bytes;
	rx_buf += mas->cur_xfer->len - mas->rx_rem_bytes;
	while (i < rx_bytes) {
		unsigned int fifo_word = 0;
		u8 *fifo_byte;
		unsigned int bytes_per_fifo = fifo_width;
		unsigned int read_bytes = 0;
		unsigned int j;

		if (mas->tx_fifo_width % mas->cur_word_len)
			bytes_per_fifo =
				(mas->cur_word_len / BITS_PER_BYTE) + 1;
		if (bytes_per_fifo < (rx_bytes - i))
			read_bytes = bytes_per_fifo;
		else
			read_bytes = rx_bytes - i;
		ioread32_rep(se->base + SE_GENI_RX_FIFOn, &fifo_word, 1);
		fifo_byte = (u8 *)&fifo_word;
		for (j = 0; j < read_bytes; j++)
			rx_buf[i++] = fifo_byte[j];
	}
	mas->rx_rem_bytes -= rx_bytes;
	return IRQ_HANDLED;
}

static irqreturn_t geni_spi_isr(int irq, void *data)
{
	struct spi_master *spi = data;
	struct spi_geni_master *mas = spi_master_get_devdata(spi);
	struct geni_se *se = &mas->se;
	unsigned int m_irq = 0;
	irqreturn_t ret = IRQ_HANDLED;

	if (pm_runtime_status_suspended(mas->dev)) {
		ret = IRQ_NONE;
		goto exit_geni_spi_irq;
	}
	m_irq = readl_relaxed(se->base + SE_GENI_M_IRQ_STATUS);
	if ((m_irq & M_RX_FIFO_WATERMARK_EN) || (m_irq & M_RX_FIFO_LAST_EN))
		ret = geni_spi_handle_rx(mas);

	if ((m_irq & M_TX_FIFO_WATERMARK_EN))
		ret = geni_spi_handle_tx(mas);

	if (m_irq & M_CMD_DONE_EN) {
		spi_finalize_current_transfer(spi);
		/*
		 * If this happens, then a CMD_DONE came before all the Tx
		 * buffer bytes were sent out. This is unusual, log this
		 * condition and disable the WM interrupt to prevent the
		 * system from stalling due an interrupt storm.
		 * If this happens when all Rx bytes haven't been received, log
		 * the condition.
		 * The only known time this can happen is if bits_per_word != 8
		 * and some registers that expect xfer lengths in num spi_words
		 * weren't written correctly.
		 */
		if (mas->tx_rem_bytes) {
			writel_relaxed(0, se->base + SE_GENI_TX_WATERMARK_REG);
			dev_err(mas->dev,
				"%s:Premature Done.tx_rem%d bpw%d\n",
				__func__, mas->tx_rem_bytes, mas->cur_word_len);
		}
		if (mas->rx_rem_bytes)
			dev_err(mas->dev,
				"%s:Premature Done.rx_rem%d bpw%d\n",
				__func__, mas->rx_rem_bytes, mas->cur_word_len);
	}

	if ((m_irq & M_CMD_CANCEL_EN) || (m_irq & M_CMD_ABORT_EN))
		complete(&mas->xfer_done);
exit_geni_spi_irq:
	writel_relaxed(m_irq, se->base + SE_GENI_M_IRQ_CLEAR);
	return ret;
}

static int spi_geni_probe(struct platform_device *pdev)
{
	int ret;
	struct spi_master *spi;
	struct spi_geni_master *spi_geni;
	struct resource *res;
	struct geni_se *se;

	spi = spi_alloc_master(&pdev->dev, sizeof(struct spi_geni_master));
	if (!spi) {
		ret = -ENOMEM;
		goto spi_geni_probe_err;
	}

	platform_set_drvdata(pdev, spi);
	spi_geni = spi_master_get_devdata(spi);
	spi_geni->dev = &pdev->dev;
	spi_geni->se.dev = &pdev->dev;
	spi_geni->se.wrapper = dev_get_drvdata(pdev->dev.parent);
	se = &spi_geni->se;

	spi->bus_num = -1;
	spi->dev.of_node = pdev->dev.of_node;
	spi_geni->se.clk = devm_clk_get(&pdev->dev, "se");
	if (IS_ERR(spi_geni->se.clk)) {
		ret = PTR_ERR(spi_geni->se.clk);
		dev_err(&pdev->dev, "Err getting SE Core clk %d\n", ret);
		goto spi_geni_probe_err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	se->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(se->base)) {
		ret = -ENOMEM;
		goto spi_geni_probe_err;
	}

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "Err getting IRQ: %d\n", ret);
		goto spi_geni_probe_unmap;
	}
	ret = devm_request_irq(&pdev->dev, ret, geni_spi_isr,
			       IRQF_TRIGGER_HIGH, "spi_geni", spi);
	if (ret)
		goto spi_geni_probe_unmap;

	spi->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP | SPI_CS_HIGH;
	spi->bits_per_word_mask = SPI_BPW_RANGE_MASK(4, 32);
	spi->num_chipselect = 4;
	spi->max_speed_hz = 50000000;
	spi->prepare_transfer_hardware = spi_geni_prepare_transfer_hardware;
	spi->prepare_message = spi_geni_prepare_message;
	spi->transfer_one = spi_geni_transfer_one;
	spi->auto_runtime_pm = true;
	spi->handle_err = handle_fifo_timeout;

	init_completion(&spi_geni->xfer_done);
	pm_runtime_enable(&pdev->dev);
	ret = devm_spi_register_master(&pdev->dev, spi);
	if (ret)
		goto spi_geni_probe_unmap;

	return ret;
spi_geni_probe_unmap:
	devm_iounmap(&pdev->dev, se->base);
spi_geni_probe_err:
	spi_master_put(spi);
	return ret;
}

static int spi_geni_remove(struct platform_device *pdev)
{
	struct spi_master *spi = platform_get_drvdata(pdev);
	struct spi_geni_master *spi_geni = spi_master_get_devdata(spi);

	geni_se_resources_off(&spi_geni->se);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return 0;
}

static int __maybe_unused spi_geni_runtime_suspend(struct device *dev)
{
	struct spi_master *spi = dev_get_drvdata(dev);
	struct spi_geni_master *spi_geni = spi_master_get_devdata(spi);

	return geni_se_resources_off(&spi_geni->se);
}

static int __maybe_unused spi_geni_runtime_resume(struct device *dev)
{
	struct spi_master *spi = dev_get_drvdata(dev);
	struct spi_geni_master *spi_geni = spi_master_get_devdata(spi);

	return geni_se_resources_on(&spi_geni->se);
}

static int __maybe_unused spi_geni_suspend(struct device *dev)
{
	if (!pm_runtime_status_suspended(dev))
		return -EBUSY;
	return 0;
}

static const struct dev_pm_ops spi_geni_pm_ops = {
	SET_RUNTIME_PM_OPS(spi_geni_runtime_suspend,
					spi_geni_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(spi_geni_suspend, NULL)
};

static const struct of_device_id spi_geni_dt_match[] = {
	{ .compatible = "qcom,geni-spi" },
	{}
};

static struct platform_driver spi_geni_driver = {
	.probe  = spi_geni_probe,
	.remove = spi_geni_remove,
	.driver = {
		.name = "geni_spi",
		.pm = &spi_geni_pm_ops,
		.of_match_table = spi_geni_dt_match,
	},
};
module_platform_driver(spi_geni_driver);

MODULE_DESCRIPTION("SPI driver for GENI based QUP cores");
MODULE_LICENSE("GPL v2");
