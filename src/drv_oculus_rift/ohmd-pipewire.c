// Copyright 2019, Jan Schmidt <jan@centricular.com>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

#include <spa/support/type-map.h>
#include <spa/param/format-utils.h>
#include <spa/param/video/format-utils.h>
#include <spa/param/props.h>

#include <pipewire/pipewire.h>

#include "ohmd-pipewire.h"

#define MAX_DEBUG_BLOB_SIZE 8192

typedef struct ohmd_pw_global_data_s ohmd_pw_global_data;
typedef struct ohmd_pw_base_stream_s ohmd_pw_base_stream;

struct ohmd_pw_types_cache {
	struct spa_type_media_type media_type;
	struct spa_type_media_subtype media_subtype;
	struct spa_type_format_video format_video;
	struct spa_type_video_format video_format;
};

struct ohmd_pw_global_data_s {
	int use_count;

	struct ohmd_pw_types_cache types;

	struct pw_loop *loop;
	struct pw_thread_loop *thread;

	struct pw_core *core;
	struct pw_type *type_map;
	struct pw_remote *remote;
	struct spa_hook remote_listener;
};

struct ohmd_pw_base_stream_s {
	ohmd_pw_global_data *gdata;
	struct pw_stream *stream;
	struct spa_hook listener;
	enum pw_stream_state state;
	uint32_t seq;
};

struct ohmd_pw_video_stream_s {
	ohmd_pw_base_stream base;

	int width;
	int height;
	int fps_n, fps_d;

	struct spa_video_info_raw format;
};

struct ohmd_pw_debug_stream_s {
		ohmd_pw_base_stream base;
};

static ohmd_pw_global_data *ohmd_pw_global_data_ptr = NULL;

static void ohmd_pipewire_global_deinit();
static ohmd_pw_global_data *ohmd_pipewire_global_init();

static void init_types(struct ohmd_pw_types_cache *types, struct spa_type_map *map)
{
		spa_type_media_type_map(map, &types->media_type);
		spa_type_media_subtype_map(map, &types->media_subtype);
		spa_type_format_video_map(map, &types->format_video);
		spa_type_video_format_map(map, &types->video_format);
}

static void on_stream_state_changed(void *_data, enum pw_stream_state old, enum pw_stream_state state,
				    const char *error)
{
	printf("stream state: \"%s\"\n", pw_stream_state_as_string(state));

	switch (state) {
	case PW_STREAM_STATE_STREAMING:
	{
		break;
	}
	default:
		break;
	}
}

static void
on_video_stream_format_changed(void *_data, const struct spa_pod *format)
{
	ohmd_pw_video_stream *v = _data;
	ohmd_pw_global_data *gdata = v->base.gdata;
	struct pw_stream *stream = v->base.stream;
	struct pw_type *types = gdata->type_map;
	uint8_t params_buffer[1024];
	struct spa_pod_builder b = SPA_POD_BUILDER_INIT(params_buffer, sizeof(params_buffer));
	const struct spa_pod *params[2];
	int stride, h;

	if (format == NULL) {
		pw_stream_finish_format(stream, 0, NULL, 0);
		return;
	}
	spa_format_video_raw_parse(format, &v->format, &gdata->types.format_video);

	stride = v->width;
	h = v->height;

	params[0] = spa_pod_builder_object(&b,
		types->param.idBuffers, types->param_buffers.Buffers,
		":", types->param_buffers.size,    "i", stride * h,
		":", types->param_buffers.stride,  "i", stride,
		":", types->param_buffers.buffers, "iru", 2,
			SPA_POD_PROP_MIN_MAX(1, 32),
		":", types->param_buffers.align,   "i", 16);

	params[1] = spa_pod_builder_object(&b,
		types->param.idMeta, types->param_meta.Meta,
		":", types->param_meta.type, "I", types->meta.Header,
		":", types->param_meta.size, "i", sizeof(struct spa_meta_header));

	pw_stream_finish_format(stream, 0, params, 2);
}

static const struct pw_stream_events video_stream_events = {
	PW_VERSION_STREAM_EVENTS,
	.state_changed = on_stream_state_changed,
	.format_changed = on_video_stream_format_changed,
};

static void on_state_changed(void *_data, enum pw_remote_state old, enum pw_remote_state state, const char *error)
{
	//struct ohmd_pw_global_data *data = _data;
	//struct pw_remote *remote = data->remote;

	switch (state) {
	case PW_REMOTE_STATE_ERROR:
		LOGE("remote error: %s\n", error);
		break;

	case PW_REMOTE_STATE_CONNECTED:
	{

		break;
	}
	default:
		printf("remote state: \"%s\"\n", pw_remote_state_as_string(state));
		break;
	}
}

static const struct pw_remote_events remote_events = {
	PW_VERSION_REMOTE_EVENTS,
	.state_changed = on_state_changed,
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
	gdata->thread = pw_thread_loop_new(gdata->loop, "pipewire-loop");
	gdata->core = pw_core_new(gdata->loop, NULL);
	gdata->type_map = pw_core_get_type(gdata->core);
	gdata->remote = pw_remote_new(gdata->core, NULL, 0);

	init_types(&gdata->types, gdata->type_map->map);

	pw_remote_add_listener(gdata->remote, &gdata->remote_listener, &remote_events, &gdata);

	pw_remote_connect(gdata->remote);

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

	pw_core_destroy(gdata->core);
	pw_thread_loop_destroy(gdata->thread);
	pw_loop_destroy(gdata->loop);

	free (gdata);
	ohmd_pw_global_data_ptr = NULL;

	return;
}

ohmd_pw_video_stream *
ohmd_pw_video_stream_new (const char *stream_id, int w, int h, int fps_n, int fps_d)
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
	ret->width = w;
	ret->height = h;
	ret->fps_n = fps_n;
	ret->fps_d = fps_d;

	/* Publish the stream */
	pw_thread_loop_lock(gdata->thread);

	base->stream = pw_stream_new(gdata->remote, stream_id,
				pw_properties_new("media.class", "Video/Source",
					PW_NODE_PROP_MEDIA, "Video",
					PW_NODE_PROP_CATEGORY, "Source",
					PW_NODE_PROP_ROLE, "Rift Sensor",
					NULL));

	params[0] = spa_pod_builder_object(&b,
			gdata->type_map->param.idEnumFormat, gdata->type_map->spa_format,
			"I", gdata->types.media_type.video,
			"I", gdata->types.media_subtype.raw,
			":", gdata->types.format_video.format,    "I", gdata->types.video_format.GRAY8, /* FIXME: Support other colorspace? */
			":", gdata->types.format_video.size,      "R", &SPA_RECTANGLE(ret->width, ret->height),
			":", gdata->types.format_video.framerate, "F", &SPA_FRACTION(ret->fps_n, ret->fps_d));

	pw_stream_add_listener(base->stream, &base->listener, &video_stream_events, ret);

	pw_stream_connect(base->stream, PW_DIRECTION_OUTPUT, NULL,
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
	/* FIXME: When using pipewire support, we could capture directly into pipewire buffers */
	ohmd_pw_global_data *gdata = base->gdata;
	struct pw_type *types = gdata->type_map;

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

	if ((h = spa_buffer_find_meta(b, types->meta.Header))) {
			h->pts = pts;
			h->flags = 0;
			h->seq = base->seq++;
			h->dts_offset = 0;
	}

	memcpy (p, data, data_len);
	b->datas[0].chunk->size = b->datas[0].maxsize;

done:
	pw_thread_loop_lock (gdata->thread);
	pw_stream_queue_buffer(base->stream, buf);
	pw_thread_loop_unlock (gdata->thread);
}

void ohmd_pw_video_stream_push (ohmd_pw_video_stream *v, int64_t pts, const uint8_t *pixels)
{
	/* FIXME: Other frame formats */
	ohmd_pw_stream_push_common((ohmd_pw_base_stream *) v, pts, pixels, v->width * v->height);
}

static void
on_debug_stream_format_changed(void *_data, const struct spa_pod *format)
{
	ohmd_pw_debug_stream *s = _data;
	ohmd_pw_global_data *gdata = s->base.gdata;
	struct pw_stream *stream = s->base.stream;
	struct pw_type *types = gdata->type_map;
	uint8_t params_buffer[1024];
	struct spa_pod_builder b = SPA_POD_BUILDER_INIT(params_buffer, sizeof(params_buffer));
	const struct spa_pod *params[2];

	if (format == NULL) {
		pw_stream_finish_format(stream, 0, NULL, 0);
		return;
	}

	params[0] = spa_pod_builder_object(&b,
		types->param.idBuffers, types->param_buffers.Buffers,
		":", types->param_buffers.size,    "i", MAX_DEBUG_BLOB_SIZE,
		":", types->param_buffers.stride,  "i", 1,
		":", types->param_buffers.buffers, "iru", 2,
			SPA_POD_PROP_MIN_MAX(1, 32),
		":", types->param_buffers.align,   "i", 16);

	params[1] = spa_pod_builder_object(&b,
		types->param.idMeta, types->param_meta.Meta,
		":", types->param_meta.type, "I", types->meta.Header,
		":", types->param_meta.size, "i", sizeof(struct spa_meta_header));

	pw_stream_finish_format(stream, 0, params, 2);
}

static const struct pw_stream_events debug_stream_events = {
	PW_VERSION_STREAM_EVENTS,
	.state_changed = on_stream_state_changed,
	.format_changed = on_debug_stream_format_changed,
};

ohmd_pw_debug_stream *ohmd_pw_debug_stream_new (const char *stream_id)
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

	text_type = spa_type_map_get_id(gdata->type_map->map, "text");

	/* Create data store */
	ret = calloc (sizeof(ohmd_pw_debug_stream), 1);
	base = (ohmd_pw_base_stream *)(ret);

	base->gdata = gdata;

	/* Publish the stream */
	pw_thread_loop_lock(gdata->thread);

	base->stream = pw_stream_new(gdata->remote, stream_id,
				pw_properties_new("media.class", "Text/Source",
					PW_NODE_PROP_MEDIA, "Text",
					PW_NODE_PROP_CATEGORY, "Source",
					PW_NODE_PROP_ROLE, "OpenHMD Debug",
					NULL));

	params[0] = spa_pod_builder_object(&b,
			gdata->type_map->param.idEnumFormat, gdata->type_map->spa_format,
			"I", text_type, "I", gdata->types.media_subtype.raw);

	pw_stream_add_listener(base->stream, &base->listener, &debug_stream_events, ret);

	pw_stream_connect(base->stream, PW_DIRECTION_OUTPUT, NULL,
			  PW_STREAM_FLAG_DRIVER | PW_STREAM_FLAG_MAP_BUFFERS, params, 1);

	pw_thread_loop_unlock(gdata->thread);
	return ret;

fail:
	if (ret)
		free (ret);
	return NULL;
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
