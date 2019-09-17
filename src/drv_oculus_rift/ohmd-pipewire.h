// Copyright 2019, Jan Schmidt <jan@centricular.com>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

#pragma once

#include "../openhmdi.h"

typedef struct ohmd_pw_video_stream_s ohmd_pw_video_stream;

#ifdef HAVE_PIPEWIRE
ohmd_pw_video_stream *ohmd_pw_video_stream_new (int w, int h, int fps_n, int fps_d);
void ohmd_pw_video_stream_push (ohmd_pw_video_stream *v, uint8_t *pixels);
void ohmd_pw_video_stream_free (ohmd_pw_video_stream *v);
#else
#define ohmd_pw_video_stream_new(w,h,fps_n,fps_d) NULL
#define ohmd_pw_video_stream_push(v, pixels)
#define ohmd_pw_video_stream_free(v);
#endif

