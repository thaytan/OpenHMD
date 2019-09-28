// Copyright 2019, Jan Schmidt <jan@centricular.com>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

#pragma once

#include "../openhmdi.h"

typedef struct ohmd_pw_video_stream_s ohmd_pw_video_stream;
typedef struct ohmd_pw_debug_stream_s ohmd_pw_debug_stream;

#ifdef HAVE_PIPEWIRE
ohmd_pw_video_stream *ohmd_pw_video_stream_new (const char *stream_id, int w, int h, int fps_n, int fps_d);
void ohmd_pw_video_stream_push (ohmd_pw_video_stream *v, int64_t pts, const uint8_t *pixels);
void ohmd_pw_video_stream_free (ohmd_pw_video_stream *v);

ohmd_pw_debug_stream *ohmd_pw_debug_stream_new (const char *stream_id);
void ohmd_pw_debug_stream_push (ohmd_pw_debug_stream *s, int64_t pts, const char *debug_str);
void ohmd_pw_debug_stream_free (ohmd_pw_debug_stream *s);

#else

#define ohmd_pw_video_stream_new(streamid,w,h,fps_n,fps_d) NULL
#define ohmd_pw_video_stream_push(v,pts,pixels)
#define ohmd_pw_video_stream_free(v)

#define ohmd_pw_debug_stream_new(stream_id) NULL
#define ohmd_pw_debug_stream_push(s,pts,debug_str)
#define ohmd_pw_debug_stream_free(s)
#endif

