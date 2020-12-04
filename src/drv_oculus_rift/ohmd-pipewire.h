// Copyright 2019, Jan Schmidt <jan@centricular.com>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

#pragma once

#include "../openhmdi.h"

typedef struct ohmd_pw_video_stream_s ohmd_pw_video_stream;
typedef struct ohmd_pw_debug_stream_s ohmd_pw_debug_stream;
typedef enum ohmd_pw_video_format ohmd_pw_video_format;

enum ohmd_pw_video_format {
  OHMD_PW_VIDEO_FORMAT_GRAY8,
  OHMD_PW_VIDEO_FORMAT_RGB,
};

#ifdef HAVE_PIPEWIRE
ohmd_pw_video_stream *ohmd_pw_video_stream_new (const char *stream_id, ohmd_pw_video_format format,
        uint16_t w, uint16_t h, uint16_t fps_n, uint16_t fps_d);
bool ohmd_pw_video_stream_connected (ohmd_pw_video_stream *v);
void ohmd_pw_video_stream_push (ohmd_pw_video_stream *v, int64_t pts, const uint8_t *pixels);
void ohmd_pw_video_stream_free (ohmd_pw_video_stream *v);

ohmd_pw_debug_stream *ohmd_pw_debug_stream_new (const char *stream_id);
bool ohmd_pw_debug_stream_connected (ohmd_pw_debug_stream *s);
void ohmd_pw_debug_stream_push (ohmd_pw_debug_stream *s, int64_t pts, const char *debug_str);
void ohmd_pw_debug_stream_free (ohmd_pw_debug_stream *s);

#else

#define ohmd_pw_video_stream_new(streamid,f,w,h,fps_n,fps_d) NULL
#define ohmd_pw_video_stream_connected(v) false
#define ohmd_pw_video_stream_push(v,pts,pixels)
#define ohmd_pw_video_stream_free(v)

#define ohmd_pw_debug_stream_new(stream_id) NULL
#define ohmd_pw_debug_stream_connected(s) false
#define ohmd_pw_debug_stream_push(s,pts,debug_str)
#define ohmd_pw_debug_stream_free(s)
#endif

