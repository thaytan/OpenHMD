// Copyright 2019, Jan Schmidt <jan@centricular.com>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#include <strings.h>

#include <spa/param/video/format-utils.h>
#include <spa/param/props.h>
#include <spa/node/io.h>
#include <spa/node/utils.h>
#include <spa/pod/filter.h>

#include <pipewire/pipewire.h>

#include "ohmd-pipewire.h"

#define MAX_DEBUG_BLOB_SIZE 8192

typedef struct ohmd_pw_global_data_s ohmd_pw_global_data;
typedef struct ohmd_pw_base_stream_s ohmd_pw_base_stream;

struct ohmd_pw_global_data_s {
	int use_count;

	struct pw_loop *loop;
	struct pw_thread_loop *thread;

	struct pw_context *context;
	struct pw_core *pw_core;

	struct spa_hook remote_listener;
};

struct ohmd_pw_base_stream_s {
	ohmd_pw_global_data *gdata;
	struct pw_stream *stream;
	struct spa_hook listener;
	enum pw_stream_state state;
	uint32_t seq;
	bool connected;
};

struct ohmd_pw_video_stream_s {
	ohmd_pw_base_stream base;

	ohmd_pw_video_format format;

	uint32_t stride;
	uint16_t width, height;
	uint16_t fps_n, fps_d;
};

struct ohmd_pw_debug_stream_s {
		ohmd_pw_base_stream base;
};

static ohmd_pw_global_data *ohmd_pw_global_data_ptr = NULL;

static void ohmd_pipewire_global_deinit();
static ohmd_pw_global_data *ohmd_pipewire_global_init();

static void on_stream_state_changed(void *_data, enum pw_stream_state old, enum pw_stream_state state,
				const char *error)
{
	ohmd_pw_base_stream *s = _data;

	printf("stream state: \"%s\"\n", pw_stream_state_as_string(state));
	s->state = state;

	switch (state) {
	case PW_STREAM_STATE_STREAMING:
	{
		s->connected = true;
		break;
	}
	case PW_STREAM_STATE_ERROR:
	case PW_STREAM_STATE_UNCONNECTED:
	default:
		s->connected = false;
		break;
	}
}

static void
on_video_stream_param_changed(void *_data, uint32_t id, const struct spa_pod *param)
{
	ohmd_pw_video_stream *v = _data;
	//ohmd_pw_global_data *gdata = v->base.gdata;
	struct pw_stream *stream = v->base.stream;
	uint8_t params_buffer[1024];
	struct spa_pod_builder b = SPA_POD_BUILDER_INIT(params_buffer, sizeof(params_buffer));
	const struct spa_pod *params[2];
	int stride, h;

	if (param == NULL || id != SPA_PARAM_Format)
		return;

	stride = v->stride;
	h = v->height;

	params[0] = spa_pod_builder_add_object(&b,
		SPA_TYPE_OBJECT_ParamBuffers, SPA_PARAM_Buffers,
		SPA_PARAM_BUFFERS_size,    SPA_POD_Int(stride * h),
		SPA_PARAM_BUFFERS_stride,  SPA_POD_Int(stride),
		SPA_PARAM_BUFFERS_buffers, SPA_POD_CHOICE_RANGE_Int(2, 1, 32),
		SPA_PARAM_BUFFERS_align,   SPA_POD_Int(16));

	params[1] = spa_pod_builder_add_object(&b,
		SPA_TYPE_OBJECT_ParamMeta, SPA_PARAM_Meta,
		SPA_PARAM_META_type, SPA_POD_Id(SPA_META_Header),
		SPA_PARAM_META_size, SPA_POD_Int(sizeof (struct spa_meta_header)));

	pw_stream_update_params(stream, params, 2);
}

static const struct pw_stream_events video_stream_events = {
	PW_VERSION_STREAM_EVENTS,
	.state_changed = on_stream_state_changed,
	.param_changed = on_video_stream_param_changed,
};

static ohmd_pw_global_data *ohmd_pipewire_global_init()
{
	ohmd_pw_global_data *gdata;

	if (ohmd_pw_global_data_ptr != NULL) {
			ohmd_pw_global_data_ptr->use_count++;
			return ohmd_pw_global_data_ptr;
	}

	gdata = ohmd_pw_global_data_ptr = calloc(sizeof(ohmd_pw_global_data), 1);
	gdata->use_count = 1;

	pw_init(NULL, NULL);

	gdata->loop = pw_loop_new(NULL);
	gdata->thread = pw_thread_loop_new_full(gdata->loop, "pipewire-loop", NULL);
	gdata->context = pw_context_new(gdata->loop, NULL, 0);
	gdata->pw_core = pw_context_connect(gdata->context, NULL, 0);

	if (pw_thread_loop_start(gdata->thread) != 0) {
			ohmd_pipewire_global_deinit();
			return NULL;
	}

	return gdata;
}

static void ohmd_pipewire_global_deinit()
{
	ohmd_pw_global_data *gdata = ohmd_pw_global_data_ptr;

	if (gdata == NULL || gdata->use_count < 1) {
		LOGE ("Pipewire debugging init/deinit mismatch!");
		return;
	}

	gdata->use_count--;
	if (gdata->use_count > 0)
		return;

	pw_thread_loop_stop(gdata->thread);

	pw_context_destroy(gdata->context);
	pw_thread_loop_destroy(gdata->thread);
	pw_loop_destroy(gdata->loop);

	free (gdata);
	ohmd_pw_global_data_ptr = NULL;

	return;
}

ohmd_pw_video_stream *
ohmd_pw_video_stream_new (const char *stream_id, const char *stream_role, ohmd_pw_video_format format,
                          uint16_t w, uint16_t h, uint16_t fps_n, uint16_t fps_d)
{
	ohmd_pw_video_stream *ret = NULL;
	ohmd_pw_base_stream *base = NULL;
	ohmd_pw_global_data *gdata;

	const struct spa_pod *params[1];
	uint8_t buffer[1024];
	struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));

	/* Start pipewire connection if needed */
	gdata = ohmd_pipewire_global_init();
	if (gdata == NULL)
		goto fail;

	/* Create data store */
	ret = calloc (sizeof(ohmd_pw_video_stream), 1);
	base = (ohmd_pw_base_stream *)(ret);

	base->gdata = gdata;
	ret->format = format;
	ret->width = w;
	ret->height = h;
	ret->fps_n = fps_n;
	ret->fps_d = fps_d;

	/* Publish the stream */
	pw_thread_loop_lock(gdata->thread);

	base->stream = pw_stream_new(gdata->pw_core, stream_id,
				pw_properties_new(PW_KEY_MEDIA_CLASS, "Video/Source",
					PW_KEY_MEDIA_TYPE, "Video",
					PW_KEY_MEDIA_CATEGORY, "Source",
					PW_KEY_MEDIA_ROLE, stream_role,
					NULL));

	enum spa_video_format spa_format;

	switch (format) {
		case OHMD_PW_VIDEO_FORMAT_GRAY8:
			ret->stride = (uint32_t) ret->width;
			spa_format = SPA_VIDEO_FORMAT_GRAY8;
			break;
		case OHMD_PW_VIDEO_FORMAT_RGB:
			ret->stride = 3 * (uint32_t) ret->width;
			spa_format = SPA_VIDEO_FORMAT_RGB;
			break;
		default:
			return NULL;
	}

	params[0] = spa_pod_builder_add_object(&b,
			SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat,
			SPA_FORMAT_mediaType,       SPA_POD_Id(SPA_MEDIA_TYPE_video),
			SPA_FORMAT_mediaSubtype,    SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw),
			SPA_FORMAT_VIDEO_format,    SPA_POD_Id(spa_format),
			SPA_FORMAT_VIDEO_size,      SPA_POD_Rectangle(&SPA_RECTANGLE(ret->width, ret->height)),
			SPA_FORMAT_VIDEO_framerate, SPA_POD_Fraction(&SPA_FRACTION(ret->fps_n, ret->fps_d)));

	pw_stream_add_listener(base->stream, &base->listener, &video_stream_events, ret);

	pw_stream_connect(base->stream, PW_DIRECTION_OUTPUT, PW_ID_ANY,
			PW_STREAM_FLAG_DRIVER | PW_STREAM_FLAG_MAP_BUFFERS, params, 1);

	pw_thread_loop_unlock(gdata->thread);
	return ret;

fail:
	if (ret)
		free (ret);
	return NULL;
}

void ohmd_pw_stream_free_common (ohmd_pw_base_stream *s)
{
	ohmd_pw_global_data *gdata;

	 if (s == NULL)
		return;

	gdata = s->gdata;

	/* Stop and unpublish this source */
	pw_thread_loop_lock (gdata->thread);
	pw_stream_disconnect(s->stream);
	pw_stream_destroy(s->stream);
	pw_thread_loop_unlock(gdata->thread);

	/* Free the struct */
	free (s);

	/* Release the global pipewire state */
	ohmd_pipewire_global_deinit();
}

void ohmd_pw_video_stream_free (ohmd_pw_video_stream *v)
{
	ohmd_pw_stream_free_common ((ohmd_pw_base_stream *) v);
}

void ohmd_pw_stream_push_common (ohmd_pw_base_stream *base, int64_t pts, const uint8_t *data, size_t data_len)
{
	/* Get a buffer from pipewire and copy into it */
	ohmd_pw_global_data *gdata = base->gdata;

	struct spa_meta_header *h;
	struct pw_buffer *buf;
	struct spa_buffer *b;
	void *p;

	pw_thread_loop_lock (gdata->thread);
	buf = pw_stream_dequeue_buffer(base->stream);
	pw_thread_loop_unlock (gdata->thread);
	if (buf == NULL)
		return;

	b = buf->buffer;

	if ((p = b->datas[0].data) == NULL)
			goto done;

	if ((h = spa_buffer_find_meta_data(b, SPA_META_Header, sizeof(*h)))) {
			h->pts = pts;
			h->flags = 0;
			h->seq = base->seq++;
			h->dts_offset = 0;
	}

	/* FIXME: When using pipewire support, we could capture directly into pipewire buffers with
	 * some API to alloc / return buffers */
	memcpy (p, data, data_len);
	b->datas[0].chunk->size = OHMD_MIN (data_len, b->datas[0].maxsize);

done:
	pw_thread_loop_lock (gdata->thread);
	pw_stream_queue_buffer(base->stream, buf);
	pw_thread_loop_unlock (gdata->thread);
}

bool ohmd_pw_video_stream_connected (ohmd_pw_video_stream *v)
{
	ohmd_pw_base_stream *base = (ohmd_pw_base_stream *)(v);
	return base != NULL && base->connected;
}

void ohmd_pw_video_stream_push (ohmd_pw_video_stream *v, int64_t pts, const uint8_t *pixels)
{
	ohmd_pw_stream_push_common((ohmd_pw_base_stream *) v, pts, pixels, v->stride * v->height);
}

static void
on_debug_stream_param_changed(void *_data, uint32_t id, const struct spa_pod *format)
{
	ohmd_pw_debug_stream *s = _data;
	//ohmd_pw_global_data *gdata = s->base.gdata;
	struct pw_stream *stream = s->base.stream;
	uint8_t params_buffer[1024];
	struct spa_pod_builder b = SPA_POD_BUILDER_INIT(params_buffer, sizeof(params_buffer));
	const struct spa_pod *params[2];

	if (format == NULL) {
		pw_stream_update_params(stream, NULL, 0);
		return;
	}

	params[0] = spa_pod_builder_add_object(&b,
		SPA_TYPE_OBJECT_ParamBuffers, SPA_PARAM_Buffers,
		SPA_PARAM_BUFFERS_buffers, SPA_POD_CHOICE_RANGE_Int(2, 1, 32),
		SPA_PARAM_BUFFERS_blocks,  SPA_POD_Int(1),
		SPA_PARAM_BUFFERS_size,    SPA_POD_Int(MAX_DEBUG_BLOB_SIZE),
		SPA_PARAM_BUFFERS_stride,  SPA_POD_Int(1),
		SPA_PARAM_BUFFERS_align,   SPA_POD_Int(16));

	params[1] = spa_pod_builder_add_object(&b,
		SPA_TYPE_OBJECT_ParamMeta, SPA_PARAM_Meta,
		SPA_PARAM_META_type, SPA_POD_Id(SPA_META_Header),
		SPA_PARAM_META_size, SPA_POD_Int(sizeof(struct spa_meta_header)));

	pw_stream_update_params(stream, params, 2);
}

static const struct pw_stream_events debug_stream_events = {
	PW_VERSION_STREAM_EVENTS,
	.state_changed = on_stream_state_changed,
	.param_changed = on_debug_stream_param_changed,
};

ohmd_pw_debug_stream *ohmd_pw_debug_stream_new (const char *stream_id, const char *stream_role)
{
	ohmd_pw_debug_stream *ret = NULL;
	ohmd_pw_base_stream *base = NULL;
	ohmd_pw_global_data *gdata;
	uint32_t text_type;

	const struct spa_pod *params[1];
	uint8_t buffer[1024];
	struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));

	/* Start pipewire connection if needed */
	gdata = ohmd_pipewire_global_init();
	if (gdata == NULL)
		goto fail;

	text_type = SPA_FORMAT_START_Application + 0x7972; /* "OH" ascii value */

	/* Create data store */
	ret = calloc (sizeof(ohmd_pw_debug_stream), 1);
	base = (ohmd_pw_base_stream *)(ret);

	base->gdata = gdata;

	/* Publish the stream */
	pw_thread_loop_lock(gdata->thread);

	base->stream = pw_stream_new(gdata->pw_core, stream_id,
				pw_properties_new(PW_KEY_MEDIA_CLASS, "Text/Source",
					PW_KEY_MEDIA_TYPE, "Text",
					PW_KEY_MEDIA_CATEGORY, "Source",
					PW_KEY_MEDIA_ROLE, stream_role,
					NULL));

	params[0] = spa_pod_builder_add_object(&b,
			SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat,
			SPA_FORMAT_mediaType, SPA_POD_Id(text_type),
			SPA_FORMAT_mediaSubtype, SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw));

	pw_stream_add_listener(base->stream, &base->listener, &debug_stream_events, ret);

	pw_stream_connect(base->stream, PW_DIRECTION_OUTPUT, PW_ID_ANY,
			PW_STREAM_FLAG_DRIVER | PW_STREAM_FLAG_MAP_BUFFERS, params, 1);

	pw_thread_loop_unlock(gdata->thread);
	return ret;

fail:
	if (ret)
		free (ret);
	return NULL;
}

bool ohmd_pw_debug_stream_connected (ohmd_pw_debug_stream *s)
{
	ohmd_pw_base_stream *base = (ohmd_pw_base_stream *)(s);
	return base != NULL && base->connected;
}

void ohmd_pw_debug_stream_push (ohmd_pw_debug_stream *s, int64_t pts, const char *debug_str)
{
	size_t debug_blob_size = strlen (debug_str);

	if (debug_blob_size > MAX_DEBUG_BLOB_SIZE) {
		LOGE ("Debug blob of length %u > max %u\n", (unsigned int)(debug_blob_size), (unsigned int)(MAX_DEBUG_BLOB_SIZE));
	}
	ohmd_pw_stream_push_common((ohmd_pw_base_stream *) s, pts, (const uint8_t *) debug_str, debug_blob_size);
}

void ohmd_pw_debug_stream_free (ohmd_pw_debug_stream *s)
{
	ohmd_pw_stream_free_common ((ohmd_pw_base_stream *) s);
}
