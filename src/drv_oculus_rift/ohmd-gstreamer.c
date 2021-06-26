// Copyright 2021, Jan Schmidt <jan@centricular.com>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#include <strings.h>
#include <stdlib.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include "ohmd-gstreamer.h"

struct ohmd_gst_pipeline_s {
	GstElement *pipeline;
	GstElement *muxer;

	guint bus_watch_id;
	GMainLoop *loop;
	GThread *bus_thread;

	gboolean at_eos;
	GMutex lock;
	GCond cond;

	GList *sinkpads;

	uint64_t base_ts;

	ohmd_gst_debug_stream *global_metadata;
};

struct ohmd_gst_video_stream_s {
	GstPad *sinkpad;
	GstVideoInfo info;

	gint width;
	gint height;

	uint64_t base_ts;
	uint64_t last_ts;
};

struct ohmd_gst_debug_stream_s {
	GstPad *sinkpad;
	GstElement *src_q;

	uint64_t base_ts;
	uint64_t last_ts;
};

static gboolean
bus_handler (GstBus * bus, GstMessage * message, gpointer data)
{
	ohmd_gst_pipeline *p = (ohmd_gst_pipeline *)(data);

	switch (GST_MESSAGE_TYPE (message)) {
		case GST_MESSAGE_EOS:
			/* Make sure the waiter gets woken */
			g_mutex_lock(&p->lock);
			p->at_eos = TRUE;
			g_cond_broadcast(&p->cond);
			g_mutex_unlock(&p->lock);
			break;
		case GST_MESSAGE_ERROR:
		{
			GError *err = NULL;
			gchar *name;
			
			name = gst_object_get_path_string (message->src);
			gst_message_parse_error (message, &err, NULL);

			LOGE("Error from GStreamer pipeline. Element %s: %s", name, err->message);
			
			g_clear_error (&err);
			g_free (name);

			/* Make sure the waiter gets woken */
			g_mutex_lock(&p->lock);
			p->at_eos = TRUE;
			g_cond_broadcast(&p->cond);
			g_mutex_unlock(&p->lock);

			gst_element_set_state(p->pipeline, GST_STATE_NULL);
			break;
		}
		default:
			break;
	}

	return TRUE;
}


gpointer bus_thread (gpointer data)
{
	ohmd_gst_pipeline *pipe = (ohmd_gst_pipeline *)(data);
	GMainLoop *loop = g_main_loop_new(NULL, FALSE);

	g_mutex_lock(&pipe->lock);
	pipe->loop = loop;
	g_mutex_unlock(&pipe->lock);

	g_main_loop_run(loop);

	g_mutex_lock(&pipe->lock);
	pipe->loop = NULL;
	g_mutex_unlock(&pipe->lock);

	g_main_loop_unref(loop);

	return NULL;
}

/* Takes ownership of the sinkpad ref and the caps */
static void ohmd_gst_pipeline_add_source(ohmd_gst_pipeline *pipe, GstPad *sinkpad, GstCaps *caps, const gchar *stream_id)
{
	GstEvent *event;
	GstTagList *tags;
	GstSegment segment;

	event = gst_event_new_stream_start(stream_id);
	gst_pad_send_event (sinkpad, event);

	event = gst_event_new_caps(caps);
	gst_pad_send_event (sinkpad, event);
	gst_caps_unref(caps);

	gst_segment_init(&segment, GST_FORMAT_TIME);
	event = gst_event_new_segment(&segment);
	gst_pad_send_event (sinkpad, event);

	tags = gst_tag_list_new (GST_TAG_TITLE, stream_id, NULL);
	gst_tag_list_set_scope(tags, GST_TAG_SCOPE_STREAM);
	event = gst_event_new_tag(tags);
	gst_pad_send_event (sinkpad, event);

	g_mutex_lock(&pipe->lock);
	pipe->sinkpads = g_list_prepend(pipe->sinkpads, sinkpad);
	g_mutex_unlock(&pipe->lock);
}

ohmd_gst_pipeline *ohmd_gst_pipeline_new (const char *stream_id, uint64_t base_ts)
{
	ohmd_gst_pipeline *ret = NULL;
	const char *homedir = g_getenv("HOME");
	char *path = NULL, *fname = NULL;
	GstBus *bus = NULL;
	GstElement *sink = NULL;

	gst_init(NULL, NULL);

	ret = calloc(sizeof(ohmd_gst_pipeline), 1);

	ret->pipeline = gst_pipeline_new(NULL);
	if (ret->pipeline == NULL)
		goto fail;

	g_mutex_init(&ret->lock);
	g_cond_init(&ret->cond);

	ret->base_ts = base_ts;

	bus = gst_element_get_bus (ret->pipeline);
	ret->bus_watch_id = gst_bus_add_watch (bus, bus_handler, ret);
	gst_object_unref (bus);

	ret->muxer = gst_element_factory_make("matroskamux", NULL);
	if (ret->muxer == NULL) {
		LOGE("Failed to create matroskamux");
		goto fail;
	}
	// g_object_set(ret->muxer, "streamable", TRUE, NULL);

	gst_bin_add(GST_BIN (ret->pipeline), ret->muxer);

	sink = gst_element_factory_make("filesink", NULL);
	if (sink == NULL) {
		LOGE("Failed to create filesink");
		goto fail;
	}
	gst_bin_add(GST_BIN (ret->pipeline), sink);

	if (!gst_element_link(ret->muxer, sink)) {
		LOGE("Failed to link muxer and filesink");
		goto fail;
	}

	fname = g_strdup_printf("openhmd-%s.mkv", stream_id);
	if (homedir) {
		path = g_build_path(G_DIR_SEPARATOR_S, homedir, fname, NULL);
		g_free(fname);
	} else {
		LOGW("No HOME environment variable - writing recording to %s", fname);
		path = fname;
	}
	g_object_set(sink, "location", path, NULL);
	g_free(path);

	if (gst_element_set_state(ret->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
		LOGW("Failed to set GStreamer recording pipeline to PLAYING");
	}

	ret->global_metadata = ohmd_gst_debug_stream_new (ret, "global");

	ret->bus_thread = g_thread_new("GStreamer bus", bus_thread, ret);

	return ret;

fail:
	if (ret) {
		if (ret->global_metadata) {
			ohmd_gst_debug_stream_free (ret->global_metadata);
		}
		if (ret->pipeline) {
			gst_element_set_state(ret->pipeline, GST_STATE_NULL);
		}
		gst_clear_object(&ret->pipeline);
		free(ret);
	}

	return NULL;
}

static void send_eos(gpointer data, gpointer user_data) {
	GstEvent *eos = gst_event_new_eos();
	gst_pad_send_event((GstPad *)(data), eos);
}

void ohmd_gst_pipeline_free(ohmd_gst_pipeline *pipe)
{
	if (pipe == NULL)
		return;

	/* Shut down the recording pipeline */
	if (pipe->pipeline) {
		LOGI("Finalising GStreamer recording");

		g_mutex_lock(&pipe->lock);
		g_list_foreach(pipe->sinkpads, send_eos, NULL);

		while (pipe->at_eos == FALSE)
				g_cond_wait(&pipe->cond, &pipe->lock);
		g_mutex_unlock(&pipe->lock);

		gst_element_set_state(pipe->pipeline, GST_STATE_NULL);
		gst_clear_object(&pipe->pipeline);
	}

	if (pipe->global_metadata) {
		ohmd_gst_debug_stream_free (pipe->global_metadata);
	}

	g_mutex_lock(&pipe->lock);
	if (pipe->loop != NULL)
		g_main_loop_quit (pipe->loop);
	g_mutex_unlock(&pipe->lock);

	g_thread_join(pipe->bus_thread);

	g_source_remove(pipe->bus_watch_id);

	g_mutex_clear(&pipe->lock);
	g_cond_clear(&pipe->cond);

	free(pipe);
}

void ohmd_gst_pipeline_push_metadata (ohmd_gst_pipeline *pipe, int64_t pts, const char *debug_str)
{
	ohmd_gst_debug_stream_push (pipe->global_metadata, pts, debug_str);
}

ohmd_gst_video_stream *ohmd_gst_video_stream_new (ohmd_gst_pipeline *pipe, const char *stream_id, ohmd_pw_video_format format, uint16_t w, uint16_t h, uint16_t fps_n, uint16_t fps_d)
{
	ohmd_gst_video_stream *ret = NULL;
	GstElement *input_bin = NULL;
	GstPad *input_pad = NULL, *muxer_pad = NULL;
	GstCaps *caps = NULL;
	GError *err = NULL;

	ret = calloc(sizeof(ohmd_gst_video_stream), 1);

	ret->width = w;
	ret->height = h;
	ret->base_ts = pipe->base_ts;

	input_bin = gst_parse_bin_from_description("queue name=src ! jpegenc", TRUE, &err);
	if (err != NULL) {
		fprintf(stderr, "Failed to create GStreamer recording pipeline: %s\n", err->message);
		g_clear_error(&err);
		free(ret);
		return NULL;
	}

	caps = gst_caps_new_simple("video/x-raw",
						"format", G_TYPE_STRING, format == OHMD_PW_VIDEO_FORMAT_GRAY8 ? "GRAY8" : "RGB",
						"width", G_TYPE_INT, w,
						"height", G_TYPE_INT, h,
						"framerate", GST_TYPE_FRACTION, fps_n, fps_d,
						NULL);
	if (!gst_video_info_from_caps(&ret->info, caps)) {
		gchar *caps_str = gst_caps_to_string(caps);
		LOGE("Failed to parse video info from caps %s", caps_str);
		g_free(caps_str);
		gst_caps_unref(caps);
		goto fail;
	}

	gst_bin_add(GST_BIN(pipe->pipeline), input_bin);
	input_pad = gst_element_get_static_pad(input_bin, "src");

	muxer_pad = gst_element_get_request_pad(pipe->muxer, "video_%u");
	if (gst_pad_link(input_pad, muxer_pad) != GST_PAD_LINK_OK) {
		LOGW("Failed to link video input stream to recording pipeline");
		gst_caps_unref(caps);
		goto fail;
	}
	gst_object_unref(input_pad);
	gst_object_unref(muxer_pad);

	gst_element_set_state(input_bin, GST_STATE_PLAYING);

	ret->sinkpad = gst_element_get_static_pad(input_bin, "sink");

	ohmd_gst_pipeline_add_source(pipe, ret->sinkpad, caps, stream_id);

	return ret;

fail:
	if (ret) {
		free(ret);
	}

	gst_clear_object(&input_pad);
	gst_clear_object(&muxer_pad);

	return NULL;
}

void ohmd_gst_video_stream_push (ohmd_gst_video_stream *v, int64_t pts, const uint8_t *pixels)
{
	GstBuffer *buf = gst_buffer_new_allocate(NULL, v->info.size, NULL);
	GstVideoFrame frame;
	const guint8 *in = pixels;
	guint8 *out;
	guint y = 0;

	if (!gst_video_frame_map(&frame, &v->info, buf, GST_MAP_WRITE)) {
		LOGW("Could not record video frame to GStreamer");
		gst_buffer_unref(buf);
		return;
	}

	out = frame.data[0];
	for (y = 0; y < v->height; y++) {
		memcpy (out, in, v->width);
		out += GST_VIDEO_INFO_PLANE_STRIDE (&v->info, 0);
		in += v->width;
	}

	gst_video_frame_unmap(&frame);

	v->last_ts = GST_BUFFER_PTS(buf) = pts - v->base_ts;

	if (gst_pad_chain(v->sinkpad, buf) != GST_FLOW_OK) {
		LOGW("Failed to push buffer to GStreamer recording pipeline");
	}
}

void ohmd_gst_video_stream_free (ohmd_gst_video_stream *v)
{
	GstEvent *eos = gst_event_new_eos();

	gst_pad_send_event(v->sinkpad, eos);
	free(v);
}

ohmd_gst_debug_stream *ohmd_gst_debug_stream_new (ohmd_gst_pipeline *pipe, const char *stream_id)
{
	ohmd_gst_debug_stream *ret = NULL;
	GstElement *input_bin = NULL;
	GstPad *input_pad = NULL, *muxer_pad = NULL;
	GstCaps *caps = NULL;
	GError *err = NULL;

	ret = calloc(sizeof(ohmd_gst_debug_stream), 1);

	ret->base_ts = pipe->base_ts;

	input_bin = gst_parse_bin_from_description("queue", TRUE, &err);
	if (err != NULL) {
		fprintf(stderr, "Failed to create GStreamer recording pipeline: %s\n", err->message);
		g_clear_error(&err);
		free(ret);
		return NULL;
	}

	gst_bin_add(GST_BIN(pipe->pipeline), input_bin);
	input_pad = gst_element_get_static_pad(input_bin, "src");

	muxer_pad = gst_element_get_request_pad(pipe->muxer, "subtitle_%u");
	if (gst_pad_link(input_pad, muxer_pad) != GST_PAD_LINK_OK) {
		LOGW("Failed to link subtitle input stream to recording pipeline");
		goto fail;
	}
	gst_object_unref(input_pad);
	gst_object_unref(muxer_pad);

	gst_element_set_state(input_bin, GST_STATE_PLAYING);

	ret->sinkpad = gst_element_get_static_pad(input_bin, "sink");

	caps = gst_caps_new_simple("text/x-raw", "format", G_TYPE_STRING, "utf8", NULL);

	ohmd_gst_pipeline_add_source(pipe, ret->sinkpad, caps, stream_id);

	return ret;

fail:
	if (ret)
		free(ret);

	gst_clear_object(&input_pad);
	gst_clear_object(&muxer_pad);

	return NULL;
		return NULL;
}

void ohmd_gst_debug_stream_push (ohmd_gst_debug_stream *s, int64_t pts, const char *debug_str)
{
	GstBuffer *buf;
	gchar *buf_content = g_strdup(debug_str);

	buf = gst_buffer_new_wrapped(buf_content, strlen(buf_content) + 1);
	s->last_ts = GST_BUFFER_PTS(buf) = pts - s->base_ts;

	if (gst_pad_chain(s->sinkpad, buf) != GST_FLOW_OK) {
		LOGW("Failed to push buffer to GStreamer recording pipeline");
	}
}

void
ohmd_gst_debug_stream_advance_to (ohmd_gst_debug_stream *s, int64_t pts)
{
	const uint64_t advance_threshold = 20 * GST_MSECOND;

	uint64_t new_pts = pts - s->base_ts;
	if (new_pts < advance_threshold)
		return;

	/* Advance to the indicated time minus the threshold */
	new_pts -= advance_threshold;
	if (new_pts - s->last_ts > 0) {
		GstEvent *gap = gst_event_new_gap (new_pts, GST_CLOCK_TIME_NONE);
		gst_pad_send_event(s->sinkpad, gap);

		s->last_ts = new_pts;
	}
}

void ohmd_gst_debug_stream_free (ohmd_gst_debug_stream *s)
{
	GstEvent *eos = gst_event_new_eos();

	gst_pad_send_event(s->sinkpad, eos);
	free(s);
}

