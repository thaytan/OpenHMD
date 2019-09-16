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

struct pw_types_cache {
	struct spa_type_media_type media_type;
	struct spa_type_media_subtype media_subtype;
	struct spa_type_format_video format_video;
	struct spa_type_video_format video_format;
};

struct ohmd_pw_global_data {
	int use_count;

	struct pw_types_cache types;

	struct pw_loop *loop;
	struct pw_thread_loop *thread;

	struct pw_core *core;
	struct pw_type *type_map;
	struct pw_remote *remote;
	struct spa_hook remote_listener;

	struct pw_stream *stream;
	struct spa_hook stream_listener;

	struct spa_video_info_raw format;
};

struct ohmd_pw_global_data *ohmd_pw_global_data = NULL;

static void init_types(struct pw_types_cache *types, struct spa_type_map *map)
{
		spa_type_media_type_map(map, &types->media_type);
		spa_type_media_subtype_map(map, &types->media_subtype);
		spa_type_format_video_map(map, &types->format_video);
		spa_type_video_format_map(map, &types->video_format);
}

static void on_stream_state_changed(void *_data, enum pw_stream_state old, enum pw_stream_state state,
				    const char *error)
{
	//struct ohmd_pw_global_data *data = _data;

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
on_stream_format_changed(void *_data, const struct spa_pod *format)
{
	struct ohmd_pw_global_data *data = _data;
	struct pw_stream *stream = data->stream;
	struct pw_type *types = data->type_map;
	uint8_t params_buffer[1024];
	struct spa_pod_builder b = SPA_POD_BUILDER_INIT(params_buffer, sizeof(params_buffer));
	const struct spa_pod *params[2];
  int stride, h;

	if (format == NULL) {
		pw_stream_finish_format(stream, 0, NULL, 0);
		return;
	}
	spa_format_video_raw_parse(format, &data->format, &data->types.format_video);

	stride = data->format.size.width;
	h = data->format.size.height;


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

static const struct pw_stream_events stream_events = {
	PW_VERSION_STREAM_EVENTS,
	.state_changed = on_stream_state_changed,
	.format_changed = on_stream_format_changed,
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
#if 0
		const struct spa_pod *params[1];
		uint8_t buffer[1024];
		struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));

		LOGV("remote state: \"%s\"\n",
		       pw_remote_state_as_string(state));

		data->stream = pw_stream_new(remote,
				"video-src",
				pw_properties_new(
					"media.class", "Video/Source",
					PW_NODE_PROP_MEDIA, "Video",
					PW_NODE_PROP_CATEGORY, "Source",
					PW_NODE_PROP_ROLE, "Screen",
					NULL));

		params[0] = spa_pod_builder_object(&b,
			data->t->param.idEnumFormat, data->t->spa_format,
			"I", data->types.media_type.video,
			"I", data->types.media_subtype.raw,
			":", data->types.format_video.format,    "I", data->types.video_format.RGB,
			":", data->types.format_video.size,      "Rru", &SPA_RECTANGLE(WIDTH, HEIGHT),
				SPA_POD_PROP_MIN_MAX(&SPA_RECTANGLE(1, 1),
						     &SPA_RECTANGLE(4096, 4096)),
			":", data->types.format_video.framerate, "F", &SPA_FRACTION(25, 1));

		pw_stream_add_listener(data->stream,
				       &data->stream_listener,
				       &stream_events,
				       data);

		pw_stream_connect(data->stream,
				  PW_DIRECTION_OUTPUT,
				  NULL,
				  PW_STREAM_FLAG_DRIVER |
				  PW_STREAM_FLAG_MAP_BUFFERS,
				  params, 1);
#endif
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

static int ohmd_pipewire_global_init()
{
  struct ohmd_pw_global_data *data;

  if (ohmd_pw_global_data != NULL) {
			ohmd_pw_global_data->use_count++;
			return 0;
	}

	data = ohmd_pw_global_data = calloc(sizeof(struct ohmd_pw_global_data), 1);
	data->use_count = 1;

	pw_init(NULL, NULL);

	data->loop = pw_loop_new(NULL);
  data->thread = pw_thread_loop_new(data->loop, "pipewire-loop");
	data->core = pw_core_new(data->loop, NULL);
	data->type_map = pw_core_get_type(data->core);
	data->remote = pw_remote_new(data->core, NULL, 0);

	init_types(&data->types, data->type_map->map);

	pw_remote_add_listener(data->remote, &data->remote_listener, &remote_events, &data);

	pw_remote_connect(data->remote);

	return pw_thread_loop_start(data->thread);
}

static void ohmd_pipewire_global_deinit()
{
  struct ohmd_pw_global_data *data = ohmd_pw_global_data;

  if (data == NULL || data->use_count < 1) {
		LOGE ("Pipewire debugging init/deinit mismatch!");
		return;
	}

	data->use_count--;
	if (data->use_count > 0)
		return;

  pw_thread_loop_stop(data->thread);

	pw_core_destroy(data->core);
  pw_thread_loop_destroy(data->thread);
	pw_loop_destroy(data->loop);

	free (data);
	ohmd_pw_global_data = NULL;

	return;
}
