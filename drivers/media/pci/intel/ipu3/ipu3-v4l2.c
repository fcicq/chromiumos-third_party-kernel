/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-sg.h>

#include "ipu3.h"

/******************** v4l2_subdev_ops ********************/

static int ipu3_subdev_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ipu3_mem2mem2_device *m2m2 =
		container_of(sd, struct ipu3_mem2mem2_device, subdev);
	int r = 0;

	if (m2m2->ops && m2m2->ops->s_stream)
		r = m2m2->ops->s_stream(m2m2, enable);

	if (!r)
		m2m2->streaming = enable;

	return r;
}

static int ipu3_subdev_get_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *fmt)
{
	struct ipu3_mem2mem2_device *m2m2 =
		container_of(sd, struct ipu3_mem2mem2_device, subdev);
	struct v4l2_mbus_framefmt *mf;
	u32 pad = fmt->pad;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		fmt->format = m2m2->nodes[pad].pad_fmt;
	} else {
		mf = v4l2_subdev_get_try_format(sd, cfg, pad);
		fmt->format = *mf;
	}

	return 0;
}

static int ipu3_subdev_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *fmt)
{
	struct ipu3_mem2mem2_device *m2m2 =
		container_of(sd, struct ipu3_mem2mem2_device, subdev);
	struct v4l2_mbus_framefmt *mf;
	u32 pad = fmt->pad;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		mf = v4l2_subdev_get_try_format(sd, cfg, pad);
	else
		mf = &m2m2->nodes[pad].pad_fmt;

	/* Clamp the w and h based on the hardware capabilities */
	if (m2m2->subdev_pads[pad].flags & MEDIA_PAD_FL_SOURCE) {

		fmt->format.width = clamp(fmt->format.width,
					IPU3_OUTPUT_MIN_WIDTH,
					IPU3_OUTPUT_MAX_WIDTH);
		fmt->format.height = clamp(fmt->format.height,
					IPU3_OUTPUT_MIN_HEIGHT,
					IPU3_OUTPUT_MAX_HEIGHT);
	} else {
		fmt->format.width = clamp(fmt->format.width,
					IPU3_INPUT_MIN_WIDTH,
					IPU3_INPUT_MAX_WIDTH);
		fmt->format.height = clamp(fmt->format.height,
					IPU3_INPUT_MIN_HEIGHT,
					IPU3_INPUT_MAX_HEIGHT);
	}

	*mf = fmt->format;

	return 0;
}

static int ipu3_subdev_get_selection(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_selection *sel)
{
	struct ipu3_mem2mem2_device *m2m2 =
		container_of(sd, struct ipu3_mem2mem2_device, subdev);
	struct imgu_device *imgu =
		container_of(m2m2, struct imgu_device, mem2mem2);
	struct v4l2_rect *try_sel, *rect;

	if (sel->pad != IMGU_NODE_IN)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		try_sel = v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		rect = &imgu->rect.eff;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		try_sel = v4l2_subdev_get_try_compose(sd, cfg, sel->pad);
		rect = &imgu->rect.bds;
		break;
	default:
		return -EINVAL;
	}

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY)
		sel->r = *try_sel;
	else
		sel->r = *rect;

	return 0;
}

static int ipu3_subdev_set_selection(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_selection *sel)
{
	struct ipu3_mem2mem2_device *m2m2 =
		container_of(sd, struct ipu3_mem2mem2_device, subdev);
	struct imgu_device *imgu =
		container_of(m2m2, struct imgu_device, mem2mem2);
	struct v4l2_rect *try_sel, *rect;

	if (sel->pad != IMGU_NODE_IN)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		try_sel = v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		rect = &imgu->rect.eff;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		try_sel = v4l2_subdev_get_try_compose(sd, cfg, sel->pad);
		rect = &imgu->rect.bds;
		break;
	default:
		return -EINVAL;
	}

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY)
		*try_sel = sel->r;
	else
		*rect = sel->r;

	return 0;
}

/******************** media_entity_operations ********************/

static int ipu3_link_setup(struct media_entity *entity,
				     const struct media_pad *local,
				     const struct media_pad *remote, u32 flags)
{
	struct ipu3_mem2mem2_device *m2m2 =
	    container_of(entity, struct ipu3_mem2mem2_device, subdev.entity);
	u32 pad = local->index;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	WARN_ON(entity->type != MEDIA_ENT_T_V4L2_SUBDEV);
#endif
	WARN_ON(pad >= m2m2->num_nodes);

	m2m2->nodes[pad].enabled = !!(flags & MEDIA_LNK_FL_ENABLED);

	return 0;
}

/******************** vb2_ops ********************/

/* Transfer buffer ownership to me */
static void ipu3_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct ipu3_mem2mem2_device *m2m2 = vb2_get_drv_priv(vb->vb2_queue);
	struct imgu_device *imgu =
		container_of(m2m2, struct imgu_device, mem2mem2);
	struct imgu_video_device *node =
		container_of(vb->vb2_queue, struct imgu_video_device, vbq);
	int queue;

	queue = imgu_node_to_queue(node - m2m2->nodes);

	if (queue < 0) {
		dev_err(&imgu->pci_dev->dev, "Invalid imgu node.\n");
		return;
	}

	if (queue == IPU3_CSS_QUEUE_PARAMS) {
		unsigned long need_bytes = sizeof(struct ipu3_uapi_params);
		unsigned long payload = vb2_get_plane_payload(vb, 0);
		struct vb2_v4l2_buffer *buf =
			container_of(vb, struct vb2_v4l2_buffer, vb2_buf);
		int r = -EINVAL;

		if (payload == 0) {
			payload = need_bytes;
			vb2_set_plane_payload(vb, 0, payload);
		}
		if (payload >= need_bytes)
			r = ipu3_css_set_parameters(&imgu->css,
						vb2_plane_vaddr(vb, 0));
		buf->flags = V4L2_BUF_FLAG_DONE;
		vb2_buffer_done(vb, r == 0 ? VB2_BUF_STATE_DONE
					   : VB2_BUF_STATE_ERROR);

	} else {
		struct imgu_buffer *buf = container_of(vb,
				struct imgu_buffer, m2m2_buf.vbb.vb2_buf);
		struct sg_table *sg = vb2_dma_sg_plane_desc(vb, 0);

		mutex_lock(&imgu->lock);
		ipu3_css_buf_init(&buf->css_buf, queue,
				sg_dma_address(sg->sgl));
		list_add_tail(&buf->m2m2_buf.list,
			&m2m2->nodes[node - m2m2->nodes].buffers);
		mutex_unlock(&imgu->lock);

		if (imgu->mem2mem2.streaming)
			imgu_queue_buffers(imgu, false);
	}
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
static int ipu3_vb2_queue_setup(struct vb2_queue *vq, const void *parg,
				    unsigned int *num_buffers,
				    unsigned int *num_planes,
				    unsigned int sizes[], void *alloc_ctxs[])
#else
static int ipu3_vb2_queue_setup(struct vb2_queue *vq,
				unsigned int *num_buffers,
				unsigned int *num_planes,
				unsigned int sizes[],
				struct device *alloc_devs[])
#endif
{
	struct ipu3_mem2mem2_device *m2m2 = vb2_get_drv_priv(vq);
	struct imgu_video_device *node =
		container_of(vq, struct imgu_video_device, vbq);
	const struct v4l2_format *fmt = &node->vdev_fmt;

	*num_planes = 1;
	*num_buffers = clamp_val(*num_buffers, 1, VB2_MAX_FRAME);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	alloc_ctxs[0] = m2m2->vb2_alloc_ctx;
#else
	alloc_devs[0] = m2m2->vb2_alloc_dev;
#endif
	if (vq->type == V4L2_BUF_TYPE_META_CAPTURE ||
	    vq->type == V4L2_BUF_TYPE_META_OUTPUT) {
		sizes[0] = fmt->fmt.meta.buffersize;
	} else {
		sizes[0] = fmt->fmt.pix_mp.plane_fmt[0].sizeimage;
	}

	/* Initialize buffer queue */
	INIT_LIST_HEAD(&node->buffers);

	return 0;
}

/* Check if all enabled video nodes are streaming, exception ignored */
static bool ipu3_all_nodes_streaming(struct ipu3_mem2mem2_device *m2m2,
					 struct imgu_video_device *except)
{
	int i;

	for (i = 0; i < m2m2->num_nodes; i++) {
		struct imgu_video_device *node = &m2m2->nodes[i];

		if (node == except)
			continue;
		if (node->enabled && !vb2_start_streaming_called(&node->vbq))
			return false;
	}

	return true;
}

static void ipu3_return_all_buffers(struct ipu3_mem2mem2_device *m2m2,
					struct imgu_video_device *node,
					enum vb2_buffer_state state)
{
	struct imgu_device *imgu =
			container_of(m2m2, struct imgu_device, mem2mem2);
	struct ipu3_mem2mem2_buffer *b, *b0;

	/* Return all buffers */
	mutex_lock(&imgu->lock);
	list_for_each_entry_safe(b, b0, &node->buffers, list) {
		list_del(&b->list);
		vb2_buffer_done(&b->vbb.vb2_buf, state);
	}
	mutex_unlock(&imgu->lock);
}

static int ipu3_vb2_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct ipu3_mem2mem2_device *m2m2 = vb2_get_drv_priv(vq);
	struct imgu_video_device *node =
		container_of(vq, struct imgu_video_device, vbq);
	int r;

	if (m2m2->streaming) {
		r = -EBUSY;
		goto fail_return_bufs;
	}

	if (!node->enabled) {
		r = -EINVAL;
		goto fail_return_bufs;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	r = media_entity_pipeline_start(&node->vdev.entity, &m2m2->pipeline);
#else
	r = media_pipeline_start(&node->vdev.entity, &m2m2->pipeline);
#endif
	if (r < 0)
		goto fail_return_bufs;

	if (!ipu3_all_nodes_streaming(m2m2, node))
		return 0;

	/* Start streaming of the whole pipeline now */

	r = v4l2_subdev_call(&m2m2->subdev, video, s_stream, 1);
	if (r < 0)
		goto fail_stop_pipeline;

	return 0;

fail_stop_pipeline:
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	media_entity_pipeline_stop(&node->vdev.entity);
#else
	media_pipeline_stop(&node->vdev.entity);
#endif
fail_return_bufs:
	ipu3_return_all_buffers(m2m2, node, VB2_BUF_STATE_QUEUED);

	return r;
}

static void ipu3_vb2_stop_streaming(struct vb2_queue *vq)
{
	struct ipu3_mem2mem2_device *m2m2 = vb2_get_drv_priv(vq);
	struct imgu_video_device *node =
		container_of(vq, struct imgu_video_device, vbq);
	int r;

	WARN_ON(!node->enabled);

	/* Was this the first node with streaming disabled? */
	if (ipu3_all_nodes_streaming(m2m2, node)) {
		/* Yes, really stop streaming now */
		r = v4l2_subdev_call(&m2m2->subdev, video, s_stream, 0);
		if (r)
			dev_err(m2m2->dev, "failed to stop streaming\n");
	}

	ipu3_return_all_buffers(m2m2, node, VB2_BUF_STATE_ERROR);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	media_entity_pipeline_stop(&node->vdev.entity);
#else
	media_pipeline_stop(&node->vdev.entity);
#endif
}

/******************** v4l2_ioctl_ops ********************/

static int ipu3_videoc_querycap(struct file *file, void *fh,
				  struct v4l2_capability *cap)
{
	struct ipu3_mem2mem2_device *m2m2 = video_drvdata(file);
	struct imgu_video_device *node = file_to_intel_ipu3_node(file);

	strlcpy(cap->driver, m2m2->name, sizeof(cap->driver));
	strlcpy(cap->card, m2m2->model, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s", node->name);

	return 0;
}

/* Propagate forward always the format from the CIO2 subdev */
static int ipu3_videoc_g_fmt(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	struct imgu_video_device *node = file_to_intel_ipu3_node(file);

	f->fmt = node->vdev_fmt.fmt;

	return 0;
}

/*
 * Set input/output format. Unless it is just a try, this also resets
 * selections (ie. effective and BDS resolutions) to defaults.
 */
static int mem2mem2_fmt(struct ipu3_mem2mem2_device *m2m2_dev,
			int node, struct v4l2_format *f, bool try)
{
	struct imgu_device *imgu =
		container_of(m2m2_dev, struct imgu_device, mem2mem2);
	struct v4l2_pix_format_mplane try_fmts[IPU3_CSS_QUEUES];
	struct v4l2_pix_format_mplane *fmts[IPU3_CSS_QUEUES] = { NULL };
	struct v4l2_rect *rects[IPU3_CSS_RECTS] = { NULL };
	unsigned int i;
	int css_q, r;
	struct v4l2_mbus_framefmt pad_fmt;

	if (m2m2_dev->nodes[IMGU_NODE_PV].enabled &&
		m2m2_dev->nodes[IMGU_NODE_VF].enabled) {
		dev_err(&imgu->pci_dev->dev,
				"Postview and vf are not supported simultaneously\n");
		return -EINVAL;
	}
	/*
	 * Tell css that the vf q is used for PV
	 */
	if (m2m2_dev->nodes[IMGU_NODE_PV].enabled)
		imgu->css.vf_output_en = IPU3_NODE_PV_ENABLED;
	else
		imgu->css.vf_output_en = IPU3_NODE_VF_ENABLED;

	for (i = 0; i < IPU3_CSS_QUEUES; i++) {
		int inode = imgu_map_node(imgu, i);

		if (inode < 0)
			return -EINVAL;

		/* Skip the meta node */
		if (inode == IMGU_NODE_STAT_3A || inode == IMGU_NODE_PARAMS)
			continue;

		if (try) {
			try_fmts[i] =
				m2m2_dev->nodes[inode].vdev_fmt.fmt.pix_mp;
			fmts[i] = &try_fmts[i];
		} else {
			fmts[i] = &m2m2_dev->nodes[inode].vdev_fmt.fmt.pix_mp;
		}

		/* CSS expects some format on OUT queue */
		if (i != IPU3_CSS_QUEUE_OUT &&
			!m2m2_dev->nodes[inode].enabled && inode != node)
			fmts[i] = NULL;
	}

	if (!try) {
		/* eff and bds res got by m2m2_s_sel */
		rects[IPU3_CSS_RECT_EFFECTIVE] = &imgu->rect.eff;
		rects[IPU3_CSS_RECT_BDS] = &imgu->rect.bds;
		rects[IPU3_CSS_RECT_GDC] = &imgu->rect.gdc;
	}

	/* suppose that pad fmt was set by subdev s_fmt before */
	pad_fmt = m2m2_dev->nodes[IMGU_NODE_IN].pad_fmt;
	rects[IPU3_CSS_RECT_GDC]->width = pad_fmt.width;
	rects[IPU3_CSS_RECT_GDC]->height = pad_fmt.height;

	/*
	 * ipu3_mem2mem2 doesn't set the node to the value given by user
	 * before we return success from this function, so set it here.
	 */
	css_q = imgu_node_to_queue(node);
	*fmts[css_q] = f->fmt.pix_mp;

	if (try)
		r = ipu3_css_fmt_try(&imgu->css, fmts, rects);
	else
		r = ipu3_css_fmt_set(&imgu->css, fmts, rects);

	/* fmt_try returns the binary number in the firmware blob */
	return r < 0 ? r : 0;
}

static int ipu3_videoc_try_fmt(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct ipu3_mem2mem2_device *m2m2 = video_drvdata(file);
	struct imgu_video_device *node = file_to_intel_ipu3_node(file);

	return mem2mem2_fmt(m2m2, node - m2m2->nodes, f, true);
}

static int ipu3_videoc_s_fmt(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	struct ipu3_mem2mem2_device *m2m2 = video_drvdata(file);
	struct imgu_video_device *node = file_to_intel_ipu3_node(file);
	int r;

	r = mem2mem2_fmt(m2m2, node - m2m2->nodes, f, false);
	if (!r)
		f->fmt = node->vdev_fmt.fmt;

	return r;
}

static int ipu3_meta_enum_format(struct file *file, void *fh,
				  struct v4l2_fmtdesc *f)
{
	struct imgu_video_device *node = file_to_intel_ipu3_node(file);

	/* Each node is dedicated to only one meta format */
	if (f->index > 0 || f->type != node->vbq.type)
		return -EINVAL;

	f->pixelformat = node->vdev_fmt.fmt.meta.dataformat;

	return 0;
}

static int ipu3_videoc_g_meta_fmt(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct imgu_video_device *node = file_to_intel_ipu3_node(file);

	if (f->type != node->vbq.type)
		return -EINVAL;

	f->fmt = node->vdev_fmt.fmt;

	return 0;
}

/******************** function pointers ********************/

static const struct v4l2_subdev_video_ops ipu3_subdev_video_ops = {
	.s_stream = ipu3_subdev_s_stream,
};

static const struct v4l2_subdev_pad_ops ipu3_subdev_pad_ops = {
	.link_validate = v4l2_subdev_link_validate_default,
	.get_fmt = ipu3_subdev_get_fmt,
	.set_fmt = ipu3_subdev_set_fmt,

	.get_selection = ipu3_subdev_get_selection,
	.set_selection = ipu3_subdev_set_selection,
};

static const struct v4l2_subdev_ops ipu3_subdev_ops = {
	.video = &ipu3_subdev_video_ops,
	.pad = &ipu3_subdev_pad_ops,
};

static const struct media_entity_operations ipu3_media_ops = {
	.link_setup = ipu3_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

/****************** vb2_ops of the Q ********************/

static const struct vb2_ops ipu3_vb2_ops = {
	.buf_queue = ipu3_vb2_buf_queue,
	.queue_setup = ipu3_vb2_queue_setup,
	.start_streaming = ipu3_vb2_start_streaming,
	.stop_streaming = ipu3_vb2_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

/****************** v4l2_file_operations *****************/

static const struct v4l2_file_operations ipu3_v4l2_fops = {
	.unlocked_ioctl = video_ioctl2,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

/******************** v4l2_ioctl_ops ********************/

static const struct v4l2_ioctl_ops ipu3_v4l2_ioctl_ops = {
	.vidioc_querycap = ipu3_videoc_querycap,

	.vidioc_g_fmt_vid_cap_mplane = ipu3_videoc_g_fmt,
	.vidioc_s_fmt_vid_cap_mplane = ipu3_videoc_s_fmt,
	.vidioc_try_fmt_vid_cap_mplane = ipu3_videoc_try_fmt,

	.vidioc_g_fmt_vid_out_mplane = ipu3_videoc_g_fmt,
	.vidioc_s_fmt_vid_out_mplane = ipu3_videoc_s_fmt,
	.vidioc_try_fmt_vid_out_mplane = ipu3_videoc_try_fmt,

	/* buffer queue management */
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_expbuf = vb2_ioctl_expbuf,
};

static const struct v4l2_ioctl_ops ipu3_v4l2_meta_ioctl_ops = {
	.vidioc_querycap = ipu3_videoc_querycap,

	/* meta capture */
	.vidioc_enum_fmt_meta_cap = ipu3_meta_enum_format,
	.vidioc_g_fmt_meta_cap = ipu3_videoc_g_meta_fmt,
	.vidioc_s_fmt_meta_cap = ipu3_videoc_g_meta_fmt,
	.vidioc_try_fmt_meta_cap = ipu3_videoc_g_meta_fmt,

	/* meta output */
	.vidioc_enum_fmt_meta_out = ipu3_meta_enum_format,
	.vidioc_g_fmt_meta_out = ipu3_videoc_g_meta_fmt,
	.vidioc_s_fmt_meta_out = ipu3_videoc_g_meta_fmt,
	.vidioc_try_fmt_meta_out = ipu3_videoc_g_meta_fmt,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_expbuf = vb2_ioctl_expbuf,
};

/******************** Framework registration ********************/

/* helper function to config node's video properties */
static void ipu3_node_to_v4l2(u32 node, struct video_device *vdev,
			  struct v4l2_format *f)
{
	u32 cap;

	/* Should not happen */
	WARN_ON(node >= IMGU_NODE_NUM);

	switch (node) {
	case IMGU_NODE_IN:
		cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE;
		f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		vdev->ioctl_ops = &ipu3_v4l2_ioctl_ops;
		break;
	case IMGU_NODE_PARAMS:
		cap = V4L2_CAP_META_OUTPUT;
		f->type = V4L2_BUF_TYPE_META_OUTPUT;
		f->fmt.meta.dataformat = V4L2_META_FMT_IPU3_PARAMS;
		vdev->ioctl_ops = &ipu3_v4l2_meta_ioctl_ops;
		ipu3_css_meta_fmt_set(&f->fmt.meta);
		break;
	case IMGU_NODE_STAT_3A:
		cap = V4L2_CAP_META_CAPTURE;
		f->type = V4L2_BUF_TYPE_META_CAPTURE;
		f->fmt.meta.dataformat = V4L2_META_FMT_IPU3_STAT_3A;
		vdev->ioctl_ops = &ipu3_v4l2_meta_ioctl_ops;
		ipu3_css_meta_fmt_set(&f->fmt.meta);
		break;
	default:
		cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE;
		f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		vdev->ioctl_ops = &ipu3_v4l2_ioctl_ops;
	}

	vdev->device_caps = V4L2_CAP_STREAMING | cap;
}

int ipu3_v4l2_register(struct imgu_device *dev)
{
	struct ipu3_mem2mem2_device *m2m2 = &dev->mem2mem2;
	struct v4l2_mbus_framefmt def_bus_fmt;
	struct v4l2_pix_format_mplane def_pix_fmt;

	int i, r;

	/* Initialize miscellaneous variables */
	m2m2->streaming = false;
	m2m2->v4l2_file_ops = ipu3_v4l2_fops;

	/* Set up media device */
	m2m2->media_dev.dev = m2m2->dev;
	strlcpy(m2m2->media_dev.model, m2m2->model,
		sizeof(m2m2->media_dev.model));
	snprintf(m2m2->media_dev.bus_info, sizeof(m2m2->media_dev.bus_info),
		 "%s", dev_name(m2m2->dev));
	m2m2->media_dev.driver_version = LINUX_VERSION_CODE;
	m2m2->media_dev.hw_revision = 0;
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 5, 0)
	media_device_init(&m2m2->media_dev);
#endif
	r = media_device_register(&m2m2->media_dev);
	if (r) {
		dev_err(m2m2->dev, "failed to register media device (%d)\n", r);
		goto fail_media_dev;
	}

	/* Set up v4l2 device */
	m2m2->v4l2_dev.mdev = &m2m2->media_dev;
	m2m2->v4l2_dev.ctrl_handler = m2m2->ctrl_handler;
	r = v4l2_device_register(m2m2->dev, &m2m2->v4l2_dev);
	if (r) {
		dev_err(m2m2->dev, "failed to register V4L2 device (%d)\n", r);
		goto fail_v4l2_dev;
	}

	/* Initialize subdev media entity */
	m2m2->subdev_pads = kzalloc(sizeof(*m2m2->subdev_pads) *
					m2m2->num_nodes, GFP_KERNEL);
	if (!m2m2->subdev_pads) {
		r = -ENOMEM;
		goto fail_subdev_pads;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	r = media_entity_init(&m2m2->subdev.entity, m2m2->num_nodes,
			      m2m2->subdev_pads, 0);
#else
	r = media_entity_pads_init(&m2m2->subdev.entity, m2m2->num_nodes,
				   m2m2->subdev_pads);
#endif
	if (r) {
		dev_err(m2m2->dev,
			"failed initialize subdev media entity (%d)\n", r);
		goto fail_media_entity;
	}
	m2m2->subdev.entity.ops = &ipu3_media_ops;
	for (i = 0; i < m2m2->num_nodes; i++) {
		m2m2->subdev_pads[i].flags = m2m2->nodes[i].output ?
			MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;
	}

	/* Initialize subdev */
	v4l2_subdev_init(&m2m2->subdev, &ipu3_subdev_ops);
	m2m2->subdev.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(m2m2->subdev.name, sizeof(m2m2->subdev.name),
		 "%s", m2m2->name);
	v4l2_set_subdevdata(&m2m2->subdev, m2m2);
	m2m2->subdev.ctrl_handler = m2m2->ctrl_handler;
	r = v4l2_device_register_subdev(&m2m2->v4l2_dev, &m2m2->subdev);
	if (r) {
		dev_err(m2m2->dev, "failed initialize subdev (%d)\n", r);
		goto fail_subdev;
	}
	r = v4l2_device_register_subdev_nodes(&m2m2->v4l2_dev);
	if (r) {
		dev_err(m2m2->dev, "failed to register subdevs (%d)\n", r);
		goto fail_subdevs;
	}

	/* Initialize formats to default values */
	def_bus_fmt.width = 352;
	def_bus_fmt.height = 288;
	def_bus_fmt.code = MEDIA_BUS_FMT_UYVY8_2X8;
	def_bus_fmt.field = V4L2_FIELD_NONE;
	def_bus_fmt.colorspace = V4L2_COLORSPACE_RAW;
	def_bus_fmt.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	def_bus_fmt.quantization = V4L2_QUANTIZATION_DEFAULT;
	def_bus_fmt.xfer_func = V4L2_XFER_FUNC_DEFAULT;

	def_pix_fmt.width = def_bus_fmt.width;
	def_pix_fmt.height = def_bus_fmt.height;
	def_pix_fmt.pixelformat = V4L2_PIX_FMT_YUYV;
	def_pix_fmt.field = def_bus_fmt.field;
	def_pix_fmt.num_planes = 1;
	def_pix_fmt.plane_fmt[0].bytesperline = def_pix_fmt.width * 2;
	def_pix_fmt.plane_fmt[0].sizeimage =
		def_pix_fmt.height * def_pix_fmt.plane_fmt[0].bytesperline;
	def_pix_fmt.flags = 0;
	def_pix_fmt.colorspace = def_bus_fmt.colorspace;
	def_pix_fmt.ycbcr_enc = def_bus_fmt.ycbcr_enc;
	def_pix_fmt.quantization = def_bus_fmt.quantization;
	def_pix_fmt.xfer_func = def_bus_fmt.xfer_func;

	/* Create video nodes and links */
	for (i = 0; i < m2m2->num_nodes; i++) {
		struct imgu_video_device *node = &m2m2->nodes[i];
		struct video_device *vdev = &node->vdev;
		struct vb2_queue *vbq = &node->vbq;
		u32 flags;

		/* Initialize miscellaneous variables */
		mutex_init(&node->lock);
		INIT_LIST_HEAD(&node->buffers);

		/* Initialize formats to default values */
		node->pad_fmt = def_bus_fmt;
		ipu3_node_to_v4l2(i, vdev, &node->vdev_fmt);
		if (node->vdev_fmt.type == V4L2_BUF_TYPE_VIDEO_OUTPUT ||
			node->vdev_fmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			node->vdev_fmt.fmt.pix_mp = def_pix_fmt;

		/* Initialize media entities */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
		r = media_entity_init(&vdev->entity, 1, &node->vdev_pad, 0);
#else
		r = media_entity_pads_init(&vdev->entity, 1, &node->vdev_pad);
#endif
		if (r) {
			dev_err(m2m2->dev,
				"failed initialize media entity (%d)\n", r);
			goto fail_vdev_media_entity;
		}
		node->vdev_pad.flags = node->output ?
			MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;
		vdev->entity.ops = NULL;

		/* Initialize vbq */
		vbq->type = node->vdev_fmt.type;
		vbq->io_modes = VB2_USERPTR | VB2_MMAP | VB2_DMABUF;
		vbq->ops = &ipu3_vb2_ops;
		vbq->mem_ops = m2m2->vb2_mem_ops;
		if (m2m2->buf_struct_size <= 0)
			m2m2->buf_struct_size =
				sizeof(struct ipu3_mem2mem2_buffer);
		vbq->buf_struct_size = m2m2->buf_struct_size;
		vbq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
		vbq->min_buffers_needed = 0;	/* Can streamon w/o buffers */
		vbq->drv_priv = m2m2;
		vbq->lock = &node->lock;
		r = vb2_queue_init(vbq);
		if (r) {
			dev_err(m2m2->dev,
				"failed to initialize video queue (%d)\n", r);
			goto fail_vdev;
		}

		/* Initialize vdev */
		snprintf(vdev->name, sizeof(vdev->name), "%s %s",
			 m2m2->name, node->name);
		vdev->release = video_device_release_empty;
		vdev->fops = &m2m2->v4l2_file_ops;
		vdev->lock = &node->lock;
		vdev->v4l2_dev = &m2m2->v4l2_dev;
		vdev->queue = &node->vbq;
		vdev->vfl_dir = node->output ? VFL_DIR_TX : VFL_DIR_RX;
		video_set_drvdata(vdev, m2m2);
		r = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
		if (r) {
			dev_err(m2m2->dev,
				"failed to register video device (%d)\n", r);
			goto fail_vdev;
		}

		/* Create link between video node and the subdev pad */
		flags = 0;
		if (node->enabled)
			flags |= MEDIA_LNK_FL_ENABLED;
		if (node->immutable)
			flags |= MEDIA_LNK_FL_IMMUTABLE;
		if (node->output) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
			r = media_entity_create_link(
#else
			r = media_create_pad_link(
#endif
						 &vdev->entity, 0,
						 &m2m2->subdev.entity,
						 i, flags);
		} else {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
			r = media_entity_create_link(
#else
			r = media_create_pad_link(
#endif
						 &m2m2->subdev.entity,
						 i, &vdev->entity, 0,
						 flags);
		}
		if (r)
			goto fail_link;

	}

	return 0;

	for (; i >= 0; i--) {
fail_link:
		video_unregister_device(&m2m2->nodes[i].vdev);
fail_vdev:
		media_entity_cleanup(&m2m2->nodes[i].vdev.entity);
fail_vdev_media_entity:
		mutex_destroy(&m2m2->nodes[i].lock);
	}
fail_subdevs:
	v4l2_device_unregister_subdev(&m2m2->subdev);
fail_subdev:
	media_entity_cleanup(&m2m2->subdev.entity);
fail_media_entity:
	kfree(m2m2->subdev_pads);
fail_subdev_pads:
	v4l2_device_unregister(&m2m2->v4l2_dev);
fail_v4l2_dev:
	media_device_unregister(&m2m2->media_dev);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 5, 0)
	media_device_cleanup(&m2m2->media_dev);
#endif
fail_media_dev:

	return r;
}
EXPORT_SYMBOL_GPL(ipu3_v4l2_register);

int ipu3_v4l2_unregister(struct imgu_device *dev)
{
	struct ipu3_mem2mem2_device *m2m2 = &dev->mem2mem2;
	unsigned int i;

	for (i = 0; i < m2m2->num_nodes; i++) {
		video_unregister_device(&m2m2->nodes[i].vdev);
		media_entity_cleanup(&m2m2->nodes[i].vdev.entity);
		mutex_destroy(&m2m2->nodes[i].lock);
	}

	v4l2_device_unregister_subdev(&m2m2->subdev);
	media_entity_cleanup(&m2m2->subdev.entity);
	kfree(m2m2->subdev_pads);
	v4l2_device_unregister(&m2m2->v4l2_dev);
	media_device_unregister(&m2m2->media_dev);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 5, 0)
	media_device_cleanup(&m2m2->media_dev);
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(ipu3_v4l2_unregister);

void ipu3_v4l2_buffer_done(struct vb2_buffer *vb,
			   enum vb2_buffer_state state)
{
	struct ipu3_mem2mem2_buffer *b =
		container_of(vb, struct ipu3_mem2mem2_buffer, vbb.vb2_buf);

	list_del(&b->list);
	vb2_buffer_done(&b->vbb.vb2_buf, state);
}
EXPORT_SYMBOL_GPL(ipu3_v4l2_buffer_done);
