/*
 * Rockchip isp1 driver
 *
 * Copyright (C) 2017 Rockchip Electronics Co., Ltd.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include "capture.h"
#include "rkisp1.h"
#include "regs.h"

#define CIF_ISP_REQ_BUFS_MIN 1
#define CIF_ISP_REQ_BUFS_MAX 8

/*
 * mp sink source: isp
 * sp sink source : isp, no dma now
 * mp sink pad fmts: yuv 422, raw
 * sp sink pad fmts: yuv422( source isp), yuv420......
 * mp source fmts: yuv, raw, no jpeg now
 * sp source fmts: yuv, rgb
 */

#define STREAM_PAD_SINK				0
#define STREAM_PAD_SOURCE			1

#define STREAM_MAX_MP_RSZ_OUTPUT_WIDTH		4416
#define STREAM_MAX_MP_RSZ_OUTPUT_HEIGHT		3312
#define STREAM_MAX_SP_RSZ_OUTPUT_WIDTH		1920
#define STREAM_MAX_SP_RSZ_OUTPUT_HEIGHT		1080
#define STREAM_MIN_RSZ_OUTPUT_WIDTH		32
#define STREAM_MIN_RSZ_OUTPUT_HEIGHT		16

#define STREAM_MAX_MP_SP_INPUT_WIDTH STREAM_MAX_MP_RSZ_OUTPUT_WIDTH
#define STREAM_MAX_MP_SP_INPUT_HEIGHT STREAM_MAX_MP_RSZ_OUTPUT_HEIGHT
#define STREAM_MIN_MP_SP_INPUT_WIDTH		32
#define STREAM_MIN_MP_SP_INPUT_HEIGHT		32

/*
 * crop only accept yuv422 format
 * resizer can accept yuv444,yuv422,yuv420 format, can output yuv422, yuv420,
 * yuv444 format
 * sp resizer has tow data source: DMA-reader or crop
 * mp resizer has only one data source:  crop
 * if format is unsupported by path, crop and resizer should be bypassed
 * (disabled)
 */

/* Get xsubs and ysubs for fourcc formats
 *
 * @xsubs: horizon color samples in a 4*4 matrix, for yuv
 * @ysubs: vertical color samples in a 4*4 matrix, for yuv
 */
static int fcc_xysubs(u32 fcc, u32 *xsubs, u32 *ysubs)
{
	switch (fcc) {
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_YUV444M:
		*xsubs = 1;
		*ysubs = 1;
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_YVU422M:
		*xsubs = 2;
		*ysubs = 1;
		break;
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		*xsubs = 2;
		*ysubs = 2;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mbus_code_xysubs(u32 code, u32 *xsubs, u32 *ysubs)
{
	switch (code) {
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YVYU8_1X16:
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_1X16:
		*xsubs = 2;
		*ysubs = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;

}

static int mbus_code_sp_in_fmt(u32 code, u32 *format)
{
	switch (code) {
	case MEDIA_BUS_FMT_YUYV8_2X8:
		*format = MI_CTRL_SP_INPUT_YUV422;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct capture_fmt mp_fmts[] = {
	/* yuv422 */
	{
		.fourcc = V4L2_PIX_FMT_YUYV,
		.fmt_type = FMT_YUV,
		.bpp = { 16 },
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUVINT,
	}, {
		.fourcc = V4L2_PIX_FMT_YVYU,
		.fmt_type = FMT_YUV,
		.bpp = { 16 },
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_MP_WRITE_YUVINT,
	}, {
		.fourcc = V4L2_PIX_FMT_VYUY,
		.fmt_type = FMT_YUV,
		.bpp = { 16 },
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_MP_WRITE_YUVINT,
	}, {
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 4, 4 },
		.cplanes = 3,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	}, {
		.fourcc = V4L2_PIX_FMT_NV16,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 16 },
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUV_SPLA,
	}, {
		.fourcc = V4L2_PIX_FMT_NV61,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 16 },
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_SPLA,
	}, {
		.fourcc = V4L2_PIX_FMT_YVU422M,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 8, 8 },
		.cplanes = 3,
		.mplanes = 3,
		.uv_swap = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	},
	/* yuv420 */
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 16 },
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_SPLA,
	}, {
		.fourcc = V4L2_PIX_FMT_NV12,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 16 },
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUV_SPLA,
	}, {
		.fourcc = V4L2_PIX_FMT_NV21M,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 16 },
		.cplanes = 2,
		.mplanes = 2,
		.uv_swap = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_SPLA,
	}, {
		.fourcc = V4L2_PIX_FMT_NV12M,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 16 },
		.cplanes = 2,
		.mplanes = 2,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUV_SPLA,
	}, {
		.fourcc = V4L2_PIX_FMT_YUV420,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 8, 8 },
		.cplanes = 3,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	}, {
		.fourcc = V4L2_PIX_FMT_YVU420,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 8, 8 },
		.cplanes = 3,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	},
	/* yuv444 */
	{
		.fourcc = V4L2_PIX_FMT_YUV444M,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 8, 8 },
		.cplanes = 3,
		.mplanes = 3,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	},
	/* yuv400 */
	{
		.fourcc = V4L2_PIX_FMT_GREY,
		.fmt_type = FMT_YUV,
		.bpp = { 8 },
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_MP_WRITE_YUVINT,
	},
	/* raw */
	{
		.fourcc = V4L2_PIX_FMT_SRGGB8,
		.fmt_type = FMT_BAYER,
		.bpp = { 8 },
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG8,
		.fmt_type = FMT_BAYER,
		.bpp = { 8 },
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG8,
		.fmt_type = FMT_BAYER,
		.bpp = { 8 },
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR8,
		.fmt_type = FMT_BAYER,
		.bpp = { 8 },
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_YUV_PLA_OR_RAW8,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB8,
		.fmt_type = FMT_BAYER,
		.bpp = { 10 },
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG10,
		.fmt_type = FMT_BAYER,
		.bpp = { 10 },
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG10,
		.fmt_type = FMT_BAYER,
		.bpp = { 10 },
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR10,
		.fmt_type = FMT_BAYER,
		.bpp = { 10 },
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB12,
		.fmt_type = FMT_BAYER,
		.bpp = { 12 },
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG12,
		.fmt_type = FMT_BAYER,
		.bpp = { 12 },
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG12,
		.fmt_type = FMT_BAYER,
		.bpp = { 12 },
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR12,
		.fmt_type = FMT_BAYER,
		.bpp = { 12 },
		.mplanes = 1,
		.write_format = MI_CTRL_MP_WRITE_RAW12,
	},
};

static const struct capture_fmt sp_fmts[] = {
	/* yuv422 */
	{
		.fourcc = V4L2_PIX_FMT_YUYV,
		.fmt_type = FMT_YUV,
		.bpp = { 16 },
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_INT,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	}, {
		.fourcc = V4L2_PIX_FMT_YVYU,
		.fmt_type = FMT_YUV,
		.bpp = { 16 },
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_SP_WRITE_INT,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	}, {
		.fourcc = V4L2_PIX_FMT_VYUY,
		.fmt_type = FMT_YUV,
		.bpp = { 16 },
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_SP_WRITE_INT,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	}, {
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 8, 8 },
		.cplanes = 3,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	}, {
		.fourcc = V4L2_PIX_FMT_NV16,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 16 },
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_SPLA,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	}, {
		.fourcc = V4L2_PIX_FMT_NV61,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 16 },
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_SP_WRITE_SPLA,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	}, {
		.fourcc = V4L2_PIX_FMT_YVU422M,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 8, 8 },
		.cplanes = 3,
		.mplanes = 3,
		.uv_swap = 1,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.output_format = MI_CTRL_SP_OUTPUT_YUV422,
	},
	/* yuv420 */
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 16 },
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_SP_WRITE_SPLA,
		.output_format = MI_CTRL_SP_OUTPUT_YUV420,
	}, {
		.fourcc = V4L2_PIX_FMT_NV12,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 16 },
		.cplanes = 2,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_SPLA,
		.output_format = MI_CTRL_SP_OUTPUT_YUV420,
	}, {
		.fourcc = V4L2_PIX_FMT_NV21M,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 16 },
		.cplanes = 2,
		.mplanes = 2,
		.uv_swap = 1,
		.write_format = MI_CTRL_SP_WRITE_SPLA,
		.output_format = MI_CTRL_SP_OUTPUT_YUV420,
	}, {
		.fourcc = V4L2_PIX_FMT_NV12M,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 16 },
		.cplanes = 2,
		.mplanes = 2,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_SPLA,
		.output_format = MI_CTRL_SP_OUTPUT_YUV420,
	}, {
		.fourcc = V4L2_PIX_FMT_YUV420,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 8, 8 },
		.cplanes = 3,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.output_format = MI_CTRL_SP_OUTPUT_YUV420,
	}, {
		.fourcc = V4L2_PIX_FMT_YVU420,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 8, 8 },
		.cplanes = 3,
		.mplanes = 1,
		.uv_swap = 1,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.output_format = MI_CTRL_SP_OUTPUT_YUV420,
	},
	/* yuv444 */
	{
		.fourcc = V4L2_PIX_FMT_YUV444M,
		.fmt_type = FMT_YUV,
		.bpp = { 8, 8, 8 },
		.cplanes = 3,
		.mplanes = 3,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.output_format = MI_CTRL_SP_OUTPUT_YUV444,
	},
	/* yuv400 */
	{
		.fourcc = V4L2_PIX_FMT_GREY,
		.fmt_type = FMT_YUV,
		.bpp = { 8 },
		.cplanes = 1,
		.mplanes = 1,
		.uv_swap = 0,
		.write_format = MI_CTRL_SP_WRITE_INT,
		.output_format = MI_CTRL_SP_OUTPUT_YUV400,
	},
	/* rgb */
	{
		.fourcc = V4L2_PIX_FMT_RGB24,
		.fmt_type = FMT_RGB,
		.bpp = { 24 },
		.mplanes = 1,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.output_format = MI_CTRL_SP_OUTPUT_RGB888,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB565,
		.fmt_type = FMT_RGB,
		.bpp = { 16 },
		.mplanes = 1,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.output_format = MI_CTRL_SP_OUTPUT_RGB565,
	}, {
		.fourcc = V4L2_PIX_FMT_BGR666,
		.fmt_type = FMT_RGB,
		.bpp = { 18 },
		.mplanes = 1,
		.write_format = MI_CTRL_SP_WRITE_PLA,
		.output_format = MI_CTRL_SP_OUTPUT_RGB666,
	},
};

static struct stream_config rkisp1_mp_stream_config = {
	.fmts = mp_fmts,
	.fmt_size = ARRAY_SIZE(mp_fmts),
	.rsz = {
		.ctrl = CIF_MRSZ_CTRL,
		.scale_hy = CIF_MRSZ_SCALE_HY,
		.scale_hcr = CIF_MRSZ_SCALE_HCR,
		.scale_hcb = CIF_MRSZ_SCALE_HCB,
		.scale_vy = CIF_MRSZ_SCALE_VY,
		.scale_vc = CIF_MRSZ_SCALE_VC,
		.scale_lut = CIF_MRSZ_SCALE_LUT,
		.scale_lut_addr = CIF_MRSZ_SCALE_LUT_ADDR,
		.scale_hy_shd = CIF_MRSZ_SCALE_HY_SHD,
		.scale_hcr_shd = CIF_MRSZ_SCALE_HCR_SHD,
		.scale_hcb_shd = CIF_MRSZ_SCALE_HCB_SHD,
		.scale_vy_shd = CIF_MRSZ_SCALE_VY_SHD,
		.scale_vc_shd = CIF_MRSZ_SCALE_VC_SHD,
		.phase_hy = CIF_MRSZ_PHASE_HY,
		.phase_hc = CIF_MRSZ_PHASE_HC,
		.phase_vy = CIF_MRSZ_PHASE_VY,
		.phase_vc = CIF_MRSZ_PHASE_VC,
		.ctrl_shd = CIF_MRSZ_CTRL_SHD,
		.phase_hy_shd = CIF_MRSZ_PHASE_HY_SHD,
		.phase_hc_shd = CIF_MRSZ_PHASE_HC_SHD,
		.phase_vy_shd = CIF_MRSZ_PHASE_VY_SHD,
		.phase_vc_shd = CIF_MRSZ_PHASE_VC_SHD,
	},
	.dual_crop = {
		.ctrl = CIF_DUAL_CROP_CTRL,
		.yuvmode_mask = CIF_DUAL_CROP_MP_MODE_YUV,
		.rawmode_mask = CIF_DUAL_CROP_MP_MODE_RAW,
		.h_offset = CIF_DUAL_CROP_M_H_OFFS,
		.v_offset = CIF_DUAL_CROP_M_V_OFFS,
		.h_size = CIF_DUAL_CROP_M_H_SIZE,
		.v_size = CIF_DUAL_CROP_M_V_SIZE,
	},
	.mi = {
		.y_size_init = CIF_MI_MP_Y_SIZE_INIT,
		.cb_size_init = CIF_MI_MP_CB_SIZE_INIT,
		.cr_size_init = CIF_MI_MP_CR_SIZE_INIT,
		.y_base_ad_init = CIF_MI_MP_Y_BASE_AD_INIT,
		.cb_base_ad_init = CIF_MI_MP_CB_BASE_AD_INIT,
		.cr_base_ad_init = CIF_MI_MP_CR_BASE_AD_INIT,
		.y_offs_cnt_init = CIF_MI_MP_Y_OFFS_CNT_INIT,
		.cb_offs_cnt_init = CIF_MI_MP_CB_OFFS_CNT_INIT,
		.cr_offs_cnt_init = CIF_MI_MP_CR_OFFS_CNT_INIT,
	},
};

static struct stream_config rkisp1_sp_stream_config = {
	.fmts = sp_fmts,
	.fmt_size = ARRAY_SIZE(sp_fmts),
	.rsz = {
		.ctrl = CIF_SRSZ_CTRL,
		.scale_hy = CIF_SRSZ_SCALE_HY,
		.scale_hcr = CIF_SRSZ_SCALE_HCR,
		.scale_hcb = CIF_SRSZ_SCALE_HCB,
		.scale_vy = CIF_SRSZ_SCALE_VY,
		.scale_vc = CIF_SRSZ_SCALE_VC,
		.scale_lut = CIF_SRSZ_SCALE_LUT,
		.scale_lut_addr = CIF_SRSZ_SCALE_LUT_ADDR,
		.scale_hy_shd = CIF_SRSZ_SCALE_HY_SHD,
		.scale_hcr_shd = CIF_SRSZ_SCALE_HCR_SHD,
		.scale_hcb_shd = CIF_SRSZ_SCALE_HCB_SHD,
		.scale_vy_shd = CIF_SRSZ_SCALE_VY_SHD,
		.scale_vc_shd = CIF_SRSZ_SCALE_VC_SHD,
		.phase_hy = CIF_SRSZ_PHASE_HY,
		.phase_hc = CIF_SRSZ_PHASE_HC,
		.phase_vy = CIF_SRSZ_PHASE_VY,
		.phase_vc = CIF_SRSZ_PHASE_VC,
		.ctrl_shd = CIF_SRSZ_CTRL_SHD,
		.phase_hy_shd = CIF_SRSZ_PHASE_HY_SHD,
		.phase_hc_shd = CIF_SRSZ_PHASE_HC_SHD,
		.phase_vy_shd = CIF_SRSZ_PHASE_VY_SHD,
		.phase_vc_shd = CIF_SRSZ_PHASE_VC_SHD,
	},
	.dual_crop = {
		.ctrl = CIF_DUAL_CROP_CTRL,
		.yuvmode_mask = CIF_DUAL_CROP_SP_MODE_YUV,
		.rawmode_mask = CIF_DUAL_CROP_SP_MODE_RAW,
		.h_offset = CIF_DUAL_CROP_S_H_OFFS,
		.v_offset = CIF_DUAL_CROP_S_V_OFFS,
		.h_size = CIF_DUAL_CROP_S_H_SIZE,
		.v_size = CIF_DUAL_CROP_S_V_SIZE,
	},
	.mi = {
		.y_size_init = CIF_MI_SP_Y_SIZE_INIT,
		.cb_size_init = CIF_MI_SP_CB_SIZE_INIT,
		.cr_size_init = CIF_MI_SP_CR_SIZE_INIT,
		.y_base_ad_init = CIF_MI_SP_Y_BASE_AD_INIT,
		.cb_base_ad_init = CIF_MI_SP_CB_BASE_AD_INIT,
		.cr_base_ad_init = CIF_MI_SP_CR_BASE_AD_INIT,
		.y_offs_cnt_init = CIF_MI_SP_Y_OFFS_CNT_INIT,
		.cb_offs_cnt_init = CIF_MI_SP_CB_OFFS_CNT_INIT,
		.cr_offs_cnt_init = CIF_MI_SP_CR_OFFS_CNT_INIT,
	},
};

static const
struct capture_fmt *find_fmt(struct rkisp1_stream *stream, const u32 pixelfmt)
{
	const struct capture_fmt *fmt;
	int i;

	for (i = 0; i < stream->config->fmt_size; i++) {
		fmt = &stream->config->fmts[i];
		if (fmt->fourcc == pixelfmt)
			return fmt;
	}
	return NULL;
}

static int rkisp1_config_dcrop(struct rkisp1_stream *stream, bool async)
{
	struct rkisp1_device *dev = stream->ispdev;
	struct v4l2_rect *dcrop = &stream->dcrop;
	struct v4l2_rect *input_win;

	input_win = rkisp1_get_isp_sd_win(dev);

	if (dcrop->width == input_win->width &&
	    dcrop->height == input_win->height &&
	    dcrop->left == 0 && dcrop->top == 0) {
		disable_dcrop(stream, async);
		return 0;
	}

	config_dcrop(stream, dcrop, async);

	return 0;
}

static int rkisp1_config_rsz(struct rkisp1_stream *stream, bool async)
{
	struct rkisp1_device *dev = stream->ispdev;
	struct v4l2_pix_format_mplane output_fmt = stream->out_fmt;
	struct capture_fmt *output_isp_fmt = &stream->out_isp_fmt;
	struct ispsd_out_fmt *input_isp_fmt = rkisp1_get_ispsd_out_fmt(dev);
	struct rkisp1_win in_y, in_c, out_y, out_c;
	u32 xsubs_in, ysubs_in, xsubs_out, ysubs_out;

	if (input_isp_fmt->fmt_type == FMT_BAYER)
		goto disable;

	/* set input and output sizes for scale calculation */
	in_y.w = stream->dcrop.width;
	in_y.h = stream->dcrop.height;
	out_y.w = output_fmt.width;
	out_y.h = output_fmt.height;

	if (mbus_code_xysubs(input_isp_fmt->mbus_code, &xsubs_in, &ysubs_in)) {
		v4l2_err(&dev->v4l2_dev, "Not xsubs/ysubs found\n");
		return -EINVAL;
	}
	in_c.w = in_y.w / xsubs_in;
	in_c.h = in_y.h / ysubs_in;

	if (output_isp_fmt->fmt_type == FMT_YUV) {
		fcc_xysubs(output_isp_fmt->fourcc, &xsubs_out, &ysubs_out);
		out_c.w = out_y.w / xsubs_out;
		out_c.h = out_y.h / ysubs_out;
	} else {
		out_c.w = out_y.w / xsubs_in;
		out_c.h = out_y.h / ysubs_in;
	}

	if (in_c.w == out_c.w && in_c.h == out_c.h)
		goto disable;

	/* set RSZ input and output */
	v4l2_dbg(1, rkisp1_debug, &dev->v4l2_dev,
		  "stream: %d fmt: %08x %dx%d -> fmt: %08x %dx%d\n",
		  stream->id, input_isp_fmt->mbus_code,
		  stream->dcrop.width, stream->dcrop.height,
		  output_isp_fmt->fourcc, output_fmt.width,
		  output_fmt.height);
	v4l2_dbg(1, rkisp1_debug, &dev->v4l2_dev,
		 "chroma scaling %dx%d -> %dx%d\n",
		 in_c.w, in_c.h, out_c.w, out_c.h);

	/* calculate and set scale */
	config_rsz(stream, &in_y, &in_c, &out_y, &out_c, async);

	if (rkisp1_debug)
		dump_rsz_regs(stream);

	return 0;

disable:
	disable_rsz(stream, async);

	return 0;
}

/***************************** stream operations*******************************/
static int mp_config_mi(struct rkisp1_stream *stream)
{
	void __iomem *base = stream->ispdev->base_addr;

	mi_set_y_size(stream, stream->out_fmt.plane_fmt[0].bytesperline *
			 stream->out_fmt.height);
	mi_set_cb_size(stream, stream->out_fmt.plane_fmt[1].sizeimage);
	mi_set_cr_size(stream, stream->out_fmt.plane_fmt[2].sizeimage);

	mp_frame_end_int_enable(base);
	if (stream->out_isp_fmt.uv_swap)
		mp_set_uv_swap(base);

	config_mi_ctrl(stream);
	mp_mi_ctrl_set_format(base, stream->out_isp_fmt.write_format);
	mp_mi_ctrl_autoupdate_en(base);

	return 0;
}

static int sp_config_mi(struct rkisp1_stream *stream)
{
	void __iomem *base = stream->ispdev->base_addr;
	struct rkisp1_device *dev = stream->ispdev;
	struct capture_fmt *output_isp_fmt = &stream->out_isp_fmt;
	struct ispsd_out_fmt *input_isp_fmt = rkisp1_get_ispsd_out_fmt(dev);
	u32 sp_in_fmt;

	if (mbus_code_sp_in_fmt(input_isp_fmt->mbus_code, &sp_in_fmt)) {
		v4l2_err(&dev->v4l2_dev, "Can't find the input format\n");
		return -EINVAL;
	}
	mi_set_y_size(stream, stream->out_fmt.plane_fmt[0].bytesperline *
		      stream->out_fmt.height);
	mi_set_cb_size(stream, stream->out_fmt.plane_fmt[1].sizeimage);
	mi_set_cr_size(stream, stream->out_fmt.plane_fmt[2].sizeimage);

	sp_set_y_width(base, stream->out_fmt.width);
	sp_set_y_height(base, stream->out_fmt.height);
	sp_set_y_line_length(base, stream->u.sp.y_stride);

	sp_frame_end_int_enable(base);
	if (output_isp_fmt->uv_swap)
		sp_set_uv_swap(base);

	config_mi_ctrl(stream);
	sp_mi_ctrl_set_format(base, stream->out_isp_fmt.write_format |
			      sp_in_fmt | output_isp_fmt->output_format);

	sp_mi_ctrl_autoupdate_en(base);

	return 0;
}

static struct rkisp1_stream *get_other_stream(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;

	return &dev->stream[stream->id ^ 1];
}

static int mp_check_against(struct rkisp1_stream *stream)
{
	struct rkisp1_stream *sp = get_other_stream(stream);

	if (sp->state == RKISP1_STATE_STREAMING)
		return -EAGAIN;

	return 0;
}

static int sp_check_against(struct rkisp1_stream *stream)
{
	struct rkisp1_stream *mp = get_other_stream(stream);
	struct v4l2_device *v4l2_dev = &stream->ispdev->v4l2_dev;

	if (mp->u.mp.raw_enable && mp->state == RKISP1_STATE_STREAMING) {
		v4l2_err(v4l2_dev, "can't start SP path when MP RAW active\n");
		return -EBUSY;
	}

	if (mp->state == RKISP1_STATE_STREAMING)
		return -EAGAIN;

	return 0;
}

static void mp_enable_mi(struct rkisp1_stream *stream)
{
	void __iomem *base = stream->ispdev->base_addr;
	struct capture_fmt *isp_fmt = &stream->out_isp_fmt;

	mi_ctrl_mp_disable(base);
	if (isp_fmt->fmt_type == FMT_BAYER) {
		mi_ctrl_mpraw_enable(base);
	} else if (isp_fmt->fmt_type == FMT_YUV) {
		mi_ctrl_mpyuv_enable(base);
	}
}

static void sp_enable_mi(struct rkisp1_stream *stream)
{
	void __iomem *base = stream->ispdev->base_addr;

	mi_ctrl_spyuv_enable(base);
}

static void mp_disable_mi(struct rkisp1_stream *stream)
{
	void __iomem *base = stream->ispdev->base_addr;

	mi_ctrl_mp_disable(base);
}

static void sp_disable_mi(struct rkisp1_stream *stream)
{
	void __iomem *base = stream->ispdev->base_addr;

	mi_ctrl_spyuv_disable(base);
}

static void update_mi(struct rkisp1_stream *stream)
{
	mi_set_y_addr(stream,
		stream->next_buf->buff_addr[RKISP1_PLANE_Y]);
	mi_set_cb_addr(stream,
		stream->next_buf->buff_addr[RKISP1_PLANE_CB]);
	mi_set_cr_addr(stream,
		stream->next_buf->buff_addr[RKISP1_PLANE_CR]);

	mi_set_y_offset(stream, 0);
	mi_set_cb_offset(stream, 0);
	mi_set_cr_offset(stream, 0);
}

static void mp_stop_mi(struct rkisp1_stream *stream)
{
	void __iomem *base = stream->ispdev->base_addr;

	if (stream->state != RKISP1_STATE_STREAMING)
		return;
	/*mp_frame_end_int_disable(base);*/
	stream->ops->clr_frame_end_int(base);
	stream->ops->disable_mi(stream);
}

static void sp_stop_mi(struct rkisp1_stream *stream)
{
	void __iomem *base = stream->ispdev->base_addr;

	if (stream->state != RKISP1_STATE_STREAMING)
		return;
	/*sp_frame_end_int_disable(base);*/
	stream->ops->clr_frame_end_int(base);
	stream->ops->disable_mi(stream);
}

static struct streams_ops rkisp1_mp_streams_ops = {
	.check_against = mp_check_against,
	.config_mi = mp_config_mi,
	.enable_mi = mp_enable_mi,
	.disable_mi = mp_disable_mi,
	.stop_mi = mp_stop_mi,
	.set_data_path = mp_set_data_path,
	.clr_frame_end_int = mp_clr_frame_end_int,
	.is_frame_end_int_masked = mp_is_frame_end_int_masked,
};

static struct streams_ops rkisp1_sp_streams_ops = {
	.check_against = sp_check_against,
	.config_mi = sp_config_mi,
	.enable_mi = sp_enable_mi,
	.disable_mi = sp_disable_mi,
	.stop_mi = sp_stop_mi,
	.set_data_path = sp_set_data_path,
	.clr_frame_end_int = sp_clr_frame_end_int,
	.is_frame_end_int_masked = sp_is_frame_end_int_masked,
};

static void rkisp1_stream_stop(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	int ret = 0;

	stream->stop = true;
	ret = wait_event_timeout(stream->done,
				 stream->state != RKISP1_STATE_STREAMING,
				 msecs_to_jiffies(1000));
	if (!ret) {
		v4l2_warn(v4l2_dev, "waiting on event return error %d\n", ret);
		stream->ops->stop_mi(stream);
		stream->stop = false;
		stream->state = RKISP1_STATE_READY;
	}
	disable_dcrop(stream, true);
	disable_rsz(stream, true);
}

static int mi_frame_end(struct rkisp1_stream *stream)
{
	struct capture_fmt *isp_fmt = &stream->out_isp_fmt;
	unsigned long lock_flags = 0;
	int i = 0;

	if (stream->curr_buf) {
		/* Dequeue a filled buffer */
		for (i = 0; i < isp_fmt->mplanes; i++) {
			u32 payload_size =
				stream->out_fmt.plane_fmt[i].sizeimage;
			vb2_set_plane_payload(
				&stream->curr_buf->vb.vb2_buf, i,
				payload_size);
		}
		vb2_buffer_done(&stream->curr_buf->vb.vb2_buf,
				VB2_BUF_STATE_DONE);
		wake_up(&stream->curr_buf->vb.vb2_buf.
			vb2_queue->done_wq);
	}

	/* Next frame is writing to it */
	stream->curr_buf = stream->next_buf;
	stream->next_buf = NULL;

	/* Set up an empty buffer for the next-next frame */
	spin_lock_irqsave(&stream->vbq_lock, lock_flags);
	if (!list_empty(&stream->buf_queue)) {
		stream->next_buf = list_first_entry(&stream->buf_queue,
				     struct rkisp1_buffer, queue);
		list_del(&stream->next_buf->queue);
	} else {
		/* Hold current buffer */
		stream->next_buf = stream->curr_buf;
		stream->curr_buf = NULL;
	}
	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);

	update_mi(stream);

	return 0;
}

/*
 * Videobuf2 operations.
 */
static int rkisp1_start(struct rkisp1_stream *stream)
{
	void __iomem *base = stream->ispdev->base_addr;
	unsigned long lock_flags = 0;
	int ret;

	stream->ops->set_data_path(base);
	ret = stream->ops->config_mi(stream);
	if (ret)
		return ret;

	spin_lock_irqsave(&stream->vbq_lock, lock_flags);
	stream->curr_buf = NULL;
	stream->next_buf = list_first_entry(&stream->buf_queue,
		struct rkisp1_buffer, queue);
	list_del(&stream->next_buf->queue);
	update_mi(stream);
	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);

	stream->ops->enable_mi(stream);
	force_cfg_update(base);
	stream->state = RKISP1_STATE_STREAMING;

	return 0;
}

static int rkisp1_restart(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	int ret;

	rkisp1_stream_stop(stream);

	ret = dev->pipe.set_stream(&dev->pipe, false);
	if (ret < 0)
		return ret;

	ret = rkisp1_start(stream);
	if (ret < 0)
		return ret;

	return 0;
}

static int rkisp1_queue_setup(struct vb2_queue *queue,
					  const void *parg,
					  unsigned int *num_buffers,
					  unsigned int *num_planes,
					  unsigned int sizes[],
					  void *alloc_ctxs[])
{
	struct rkisp1_stream *stream = queue->drv_priv;
	struct rkisp1_device *dev = stream->ispdev;
	const struct v4l2_format *pfmt = parg;
	const struct v4l2_pix_format_mplane *pixm = NULL;
	const struct capture_fmt *isp_fmt = NULL;
	u32 i;

	if (pfmt) {
		pixm = &pfmt->fmt.pix_mp;
		isp_fmt = find_fmt(stream, pixm->pixelformat);
	} else {
		pixm = &stream->out_fmt;
		isp_fmt = &stream->out_isp_fmt;
	}

	*num_buffers = clamp_t(u32, *num_buffers, CIF_ISP_REQ_BUFS_MIN,
			       CIF_ISP_REQ_BUFS_MAX);
	*num_planes = isp_fmt->mplanes;

	for (i = 0; i < isp_fmt->mplanes; i++) {
		const struct v4l2_plane_pix_format *plane_fmt;

		plane_fmt = &pixm->plane_fmt[i];

		sizes[i] = plane_fmt->sizeimage;
		alloc_ctxs[i] = dev->alloc_ctx;
	}

	v4l2_info(&dev->v4l2_dev, "%s count %d, size %d\n",
		  v4l2_type_names[queue->type], *num_buffers, sizes[0]);

	return 0;
}

static void rkisp1_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rkisp1_buffer *ispbuf = to_rkisp1_buffer(vbuf);
	struct vb2_queue *queue = vb->vb2_queue;
	struct rkisp1_stream *stream = queue->drv_priv;
	unsigned long lock_flags = 0;
	struct v4l2_pix_format_mplane *pixm = &stream->out_fmt;
	struct capture_fmt *isp_fmt = &stream->out_isp_fmt;
	int i;

	memset(ispbuf->buff_addr, 0, sizeof(ispbuf->buff_addr));
	for (i = 0; i < isp_fmt->mplanes; i++)
		ispbuf->buff_addr[i] = vb2_dma_contig_plane_dma_addr(vb, i);

	if (isp_fmt->mplanes == 1) {
		for (i = 0; i < isp_fmt->cplanes - 1; i++) {
			ispbuf->buff_addr[i + 1] =
				ispbuf->buff_addr[i] +
				pixm->plane_fmt[i].bytesperline *
				pixm->height;
		}
	}

	spin_lock_irqsave(&stream->vbq_lock, lock_flags);
	list_add_tail(&ispbuf->queue, &stream->buf_queue);
	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
}

static void rkisp1_config_clk(struct rkisp1_device *dev)
{
	/* TODO: remove CIF_CCL_CIF_CLK_ENA, this is default */
	/*writel(CIF_CCL_CIF_CLK_ENA, dev->base_addr + CIF_CCL);*/
	u32 val = CIF_ICCL_ISP_CLK | CIF_ICCL_CP_CLK | CIF_ICCL_MRSZ_CLK |
		CIF_ICCL_SRSZ_CLK | CIF_ICCL_JPEG_CLK | CIF_ICCL_MI_CLK |
		CIF_ICCL_MIPI_CLK | CIF_ICCL_DCROP_CLK;

	writel(val, dev->base_addr + CIF_ICCL);
}

static int rkisp1_set_power(struct rkisp1_device *dev, int on)
{
	int ret;

	v4l2_dbg(1, rkisp1_debug, &dev->v4l2_dev,
		 "streaming count %d, s_power: %d\n",
		 atomic_read(&dev->poweron_cnt), on);

	if (on) {
		if (atomic_inc_return(&dev->poweron_cnt) > 1)
			return 0;

		ret = pm_runtime_get_sync(dev->dev);
		if (ret < 0)
			return ret;

		/* TODO: Reset the isp would clear iommu too */
		/* writel(CIF_IRCL_CIF_SW_RST, dev->base_addr + CIF_IRCL); */
		/* udelay(1); wait at least 10 module clock cycles */

		rkisp1_config_clk(dev);
	} else {
		if (atomic_dec_and_test(&dev->poweron_cnt)) {
			ret = pm_runtime_put(dev->dev);
			if (ret < 0)
				return ret;
		}
	}

	return 0;
}

static void rkisp1_stop_streaming(struct vb2_queue *queue)
{
	struct rkisp1_stream *stream = queue->drv_priv;
	struct rkisp1_vdev_node *node = &stream->vnode;
	struct rkisp1_device *dev = stream->ispdev;
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct rkisp1_buffer *buf;
	unsigned long lock_flags = 0;
	int ret, i;

	if (stream->state != RKISP1_STATE_STREAMING)
		return;

	rkisp1_stream_stop(stream);
	ret = dev->pipe.set_stream(&dev->pipe, false);
	if (ret < 0)
		return;

	stream->state = RKISP1_STATE_READY;

	spin_lock_irqsave(&stream->vbq_lock, lock_flags);
	buf = stream->curr_buf;
	stream->curr_buf = NULL;
	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
	if (buf)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);

	spin_lock_irqsave(&stream->vbq_lock, lock_flags);
	buf = stream->next_buf;
	stream->next_buf = NULL;
	spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
	if (buf)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);

	for (i = 0; i < CIF_ISP_REQ_BUFS_MAX; i++) {
		spin_lock_irqsave(&stream->vbq_lock, lock_flags);
		if (list_empty(&stream->buf_queue)) {
			spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
			break;
		}
		buf = list_first_entry(&stream->buf_queue,
				struct rkisp1_buffer, queue);
		list_del(&buf->queue);
		spin_unlock_irqrestore(&stream->vbq_lock, lock_flags);
		if (buf)
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	ret = dev->pipe.close(&dev->pipe);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "pipeline close failed error:%d\n", ret);
		return;
	}
	rkisp1_set_power(dev, 0);
	media_entity_pipeline_stop(&node->vdev.entity);
}

static int
rkisp1_start_streaming(struct vb2_queue *queue, unsigned int count)
{
	struct rkisp1_stream *stream = queue->drv_priv;
	struct rkisp1_vdev_node *node = &stream->vnode;
	struct rkisp1_device *dev = stream->ispdev;
	struct v4l2_device *v4l2_dev = &stream->ispdev->v4l2_dev;
	int ret;

	ret = rkisp1_set_power(dev, 1);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "power on isp failed %d\n", ret);
		return ret;
	}

	ret = media_entity_pipeline_start(&node->vdev.entity, &dev->pipe.pipe);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "start pipeline failed %d\n", ret);
		return ret;
	}

	ret = dev->pipe.open(&dev->pipe, &node->vdev.entity, true);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "open cif pipeline failed %d\n", ret);
		return ret;
	}

	if (stream->state != RKISP1_STATE_READY) {
		v4l2_err(v4l2_dev, "stream not enabled\n");
		return -EBUSY;
	}

	ret = stream->ops->check_against(stream);
	if (ret == -EAGAIN) {
		struct rkisp1_stream *other;

		other = get_other_stream(stream);
		if (other)
			rkisp1_restart(other);
	} else if (ret == -EBUSY) {
		return -EBUSY;
	}

	ret = rkisp1_start(stream);
	if (ret < 0)
		return ret;

	ret = rkisp1_config_rsz(stream, false);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "failed with error %d\n", ret);
		return ret;
	}

	ret = rkisp1_config_dcrop(stream, false);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "failed with error %d\n", ret);
		return ret;
	}

	/* pipeline stream on */
	ret = dev->pipe.set_stream(&dev->pipe, true);
	if (ret < 0)
		return ret;
	return 0;
}

static struct vb2_ops rkisp1_vb2_ops = {
	.queue_setup = rkisp1_queue_setup,
	.buf_queue = rkisp1_buf_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.stop_streaming = rkisp1_stop_streaming,
	.start_streaming = rkisp1_start_streaming,
};

static int rkisp_init_vb2_queue(struct vb2_queue *q,
				    struct rkisp1_stream *stream,
				    enum v4l2_buf_type buf_type)
{
	struct rkisp1_vdev_node *node;

	node = queue_to_node(q);

	q->type = buf_type;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->drv_priv = stream;
	q->ops = &rkisp1_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct rkisp1_buffer);
	q->min_buffers_needed = CIF_ISP_REQ_BUFS_MIN;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &node->vlock;

	return vb2_queue_init(q);
}

static int rkisp1_set_fmt(struct rkisp1_stream *stream,
			  struct v4l2_pix_format_mplane *pixm,
			  bool try)
{
	const struct capture_fmt *fmt;
	unsigned int imagsize = 0;
	unsigned int planes;
	u32 xsubs = 1, ysubs = 1;
	int i;

	fmt = find_fmt(stream, pixm->pixelformat);
	if (!fmt)
		fmt = stream->config->fmts;
	/* TODO: do more checks on resolution */
	/* TODO: quantization */
	pixm->num_planes = fmt->mplanes;
	pixm->colorspace = rkisp1_get_colorspace(fmt->fmt_type);
	pixm->field = V4L2_FIELD_NONE;
	pixm->quantization = 0;

	fcc_xysubs(fmt->fourcc, &xsubs, &ysubs);
	planes = fmt->cplanes ? fmt->cplanes : fmt->mplanes;
	for (i = 0; i < planes; i++) {
		struct v4l2_plane_pix_format *plane_fmt;
		int width, height, bytesperline;

		plane_fmt = pixm->plane_fmt + i;

		if (i == 0) {
			width = pixm->width;
			height = pixm->height;
		} else {
			width = pixm->width / xsubs;
			height = pixm->height / ysubs;
		}

		bytesperline = width * DIV_ROUND_UP(fmt->bpp[i], 8);
		/* stride is only available for sp stream and y plane */
		if (stream->id != RKISP1_STREAM_SP || i != 0 ||
			plane_fmt->bytesperline < bytesperline)
			plane_fmt->bytesperline = bytesperline;

		plane_fmt->sizeimage = plane_fmt->bytesperline * height;

		imagsize += plane_fmt->sizeimage;
	}

	/* convert to non-MPLANE format */
	if (fmt->mplanes == 1)
		pixm->plane_fmt[0].sizeimage = imagsize;

	if (!try) {
		stream->out_isp_fmt = *fmt;
		stream->out_fmt = *pixm;

		if (stream->id == RKISP1_STREAM_SP) {
			stream->u.sp.y_stride =
				pixm->plane_fmt[0].bytesperline /
				DIV_ROUND_UP(fmt->bpp[0], 8);
		} else {
			stream->u.mp.raw_enable =
				fmt->fmt_type == FMT_BAYER ? true : false;
		}
	}

	return 0;
}

/************************* v4l2_file_operations***************************/
int rkisp1_stream_init(struct rkisp1_device *dev, u32 id)
{
	struct rkisp1_stream *stream = &dev->stream[id];
	struct v4l2_pix_format_mplane pixm;

	stream = &dev->stream[id];
	stream->id = id;
	stream->ispdev = dev;

	INIT_LIST_HEAD(&stream->buf_queue);
	stream->next_buf = NULL;
	stream->stop = false;
	stream->id = id;
	init_waitqueue_head(&stream->done);
	spin_lock_init(&stream->vbq_lock);
	if (stream->id == RKISP1_STREAM_SP) {
		stream->ops = &rkisp1_sp_streams_ops;
		stream->config = &rkisp1_sp_stream_config;
	} else {
		stream->ops = &rkisp1_mp_streams_ops;
		stream->config = &rkisp1_mp_stream_config;
	}

	stream->state = RKISP1_STATE_READY;

	memset(&pixm, 0, sizeof(struct v4l2_pix_format_mplane));
	pixm.pixelformat = V4L2_PIX_FMT_YUYV;
	pixm.width = RKISP1_DEFAULT_WIDTH;
	pixm.height = RKISP1_DEFAULT_HEIGHT;
	rkisp1_set_fmt(stream, &pixm, false);

	stream->dcrop.left = 0;
	stream->dcrop.top = 0;
	stream->dcrop.width = RKISP1_DEFAULT_WIDTH;
	stream->dcrop.height = RKISP1_DEFAULT_HEIGHT;

	return 0;
}

static const struct v4l2_file_operations rkisp1_fops = {
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

/*
 * mp and sp v4l2_ioctl_ops
 */

/* keep for compatibility */
static int rkisp1_enum_input(struct file *file, void *priv,
				     struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	return 0;
}

static int rkisp1_try_fmt_vid_cap_mplane(struct file *file, void *fh,
				       struct v4l2_format *f)
{
	struct rkisp1_stream *stream = video_drvdata(file);

	rkisp1_set_fmt(stream, &f->fmt.pix_mp, true);

	return 0;
}

static int rkisp1_enum_fmt_vid_cap_mplane(struct file *file, void *priv,
					      struct v4l2_fmtdesc *f)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	const struct capture_fmt *fmt = NULL;

	if (f->index >= stream->config->fmt_size) {
		return -EINVAL;
	}
	fmt = &stream->config->fmts[f->index];
	f->pixelformat = fmt->fourcc;

	return 0;
}

static int rkisp1_s_fmt_vid_cap_mplane(struct file *file,
				       void *priv, struct v4l2_format *f)
{
	struct rkisp1_stream *stream = video_drvdata(file);

	rkisp1_set_fmt(stream, &f->fmt.pix_mp, false);

	return 0;
}

static int rkisp1_g_fmt_vid_cap_mplane(struct file *file, void *fh,
				       struct v4l2_format *f)
{
	struct rkisp1_stream *stream = video_drvdata(file);

	f->fmt.pix_mp = stream->out_fmt;

	return 0;
}

static int rkisp1_g_selection(struct file *file, void *prv,
				       struct v4l2_selection *sel)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	struct rkisp1_device *dev = stream->ispdev;
	struct v4l2_rect *dcrop = &stream->dcrop;
	struct v4l2_rect *input_win;

	input_win = rkisp1_get_isp_sd_win(dev);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.width = input_win->width;
		sel->r.height = input_win->height;
		sel->r.left = 0;
		sel->r.top = 0;
		break;
	case V4L2_SEL_TGT_CROP:
		sel->r = *dcrop;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rkisp1_s_selection(struct file *file, void *prv,
				       struct v4l2_selection *sel)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	struct rkisp1_device *dev = stream->ispdev;
	struct v4l2_rect *dcrop = &stream->dcrop;
	struct v4l2_rect *input_win;

	input_win = rkisp1_get_isp_sd_win(dev);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if (sel->flags !=0 )
		return -EINVAL;

	if (sel->target == V4L2_SEL_TGT_CROP) {
		sel->r.left = ALIGN(sel->r.left, 2);
		sel->r.width = ALIGN(sel->r.width, 2);

		sel->r.left =
		    clamp_t(u32, sel->r.left, 0,
			input_win->width - STREAM_MIN_MP_SP_INPUT_WIDTH);
		sel->r.top =
		    clamp_t(u32, sel->r.top, 0,
			input_win->height - STREAM_MIN_MP_SP_INPUT_HEIGHT);
		sel->r.width =
		    clamp_t(u32, sel->r.width,
			    STREAM_MIN_MP_SP_INPUT_WIDTH,
			    input_win->width - sel->r.left);
		sel->r.height =
		    clamp_t(u32, sel->r.height,
			    STREAM_MIN_MP_SP_INPUT_HEIGHT,
			    input_win->height - sel->r.top);

		/* check whether input fmt is raw */
		if (stream->id == RKISP1_STREAM_MP &&
			stream->out_isp_fmt.fmt_type == FMT_BAYER) {
			sel->r.left = 0;
			sel->r.top = 0;
			sel->r.width = input_win->width;
			sel->r.height = input_win->height;
		}

		*dcrop = sel->r;
	}

	return 0;
}

static int rkisp1_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	struct rkisp1_stream *stream = video_drvdata(file);
	struct device *dev = stream->ispdev->dev;

	strlcpy(cap->driver, dev->driver->name, sizeof(cap->driver));
	strlcpy(cap->card, dev->driver->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:%s", dev_name(dev));

	return 0;
}

static const struct v4l2_ioctl_ops rkisp1_v4l2_ioctl_ops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_enum_input = rkisp1_enum_input,
	.vidioc_try_fmt_vid_cap_mplane = rkisp1_try_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_cap_mplane = rkisp1_enum_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane = rkisp1_s_fmt_vid_cap_mplane,
	.vidioc_g_fmt_vid_cap_mplane = rkisp1_g_fmt_vid_cap_mplane,
	.vidioc_s_selection = rkisp1_s_selection,
	.vidioc_g_selection = rkisp1_g_selection,
	.vidioc_querycap = rkisp1_querycap,
};

static void rkisp1_unregister_stream_vdev(struct rkisp1_stream *stream)
{
	media_entity_cleanup(&stream->vnode.vdev.entity);
	video_unregister_device(&stream->vnode.vdev);
}

void rkisp1_unregister_stream_vdevs(struct rkisp1_device *dev)
{
	struct rkisp1_stream *mp_stream = &dev->stream[RKISP1_STREAM_MP];
	struct rkisp1_stream *sp_stream = &dev->stream[RKISP1_STREAM_SP];

	rkisp1_unregister_stream_vdev(mp_stream);
	rkisp1_unregister_stream_vdev(sp_stream);
}

static int rkisp1_register_stream_vdev(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct video_device *vdev = &stream->vnode.vdev;
	struct rkisp1_vdev_node *node;
	int ret;

	strlcpy(vdev->name,
		stream->id == RKISP1_STREAM_SP ? SP_VDEV_NAME : MP_VDEV_NAME,
		sizeof(vdev->name));
	node = vdev_to_node(vdev);
	mutex_init(&node->vlock);

	vdev->ioctl_ops = &rkisp1_v4l2_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->fops = &rkisp1_fops;
	vdev->minor = -1;
	vdev->v4l2_dev = v4l2_dev;
	vdev->lock = &node->vlock;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
				V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, stream);
	vdev->vfl_dir = VFL_DIR_RX;
	node->pad.flags = MEDIA_PAD_FL_SINK;

	dev->alloc_ctx = vb2_dma_contig_init_ctx(v4l2_dev->dev);
	rkisp_init_vb2_queue(&node->buf_queue, stream,
			     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	vdev->queue = &node->buf_queue;

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		v4l2_err(v4l2_dev,
			 "video_register_device failed with error %d\n", ret);
		return ret;
	}

	ret = media_entity_init(&vdev->entity, 1, &node->pad, 0);
	if (ret < 0)
		return ret;

	return 0;
}

int rkisp1_register_stream_vdevs(struct rkisp1_device *dev)
{
	struct rkisp1_stream *stream;
	int i, j, ret;

	for (i = 0; i < RKISP1_MAX_STREAM; i++) {
		stream = &dev->stream[i];
		stream->ispdev = dev;
		ret = rkisp1_register_stream_vdev(stream);
		if (ret < 0)
			goto err;
	}

	return 0;
err:
	for (j = 0; j < i; j++) {
		stream = &dev->stream[j];
		rkisp1_unregister_stream_vdev(stream);
	}

	return ret;
}

void rkisp1_mi_isr(struct rkisp1_stream *stream)
{
	struct rkisp1_device *dev = stream->ispdev;
	void __iomem *base = stream->ispdev->base_addr;
	u32 val;

	stream->ops->clr_frame_end_int(base);
	val = stream->ops->is_frame_end_int_masked(base);
	if (val) {
		val = mi_get_masked_int_status(base);
		v4l2_err(&dev->v4l2_dev, "icr err: 0x%x\n", val);
	}

	mi_frame_end(stream);

	if (stream->stop) {
		stream->ops->stop_mi(stream);
		stream->stop = false;
		stream->state = RKISP1_STATE_READY;
		v4l2_info(&dev->v4l2_dev, "stream %d stopped\n", stream->id);
		wake_up(&stream->done);
	}
}
