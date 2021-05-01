// Copyright 2021, Jan Schmidt <jan@centricular.com>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

#pragma once

#include "../openhmdi.h"
#include "ohmd-pipewire.h"

typedef struct ohmd_gst_pipeline_s ohmd_gst_pipeline;
typedef struct ohmd_gst_video_stream_s ohmd_gst_video_stream;
typedef struct ohmd_gst_debug_stream_s ohmd_gst_debug_stream;

#ifdef HAVE_GSTREAMER
ohmd_gst_pipeline *ohmd_gst_pipeline_new (const char *stream_id, uint64_t base_ts);
void ohmd_gst_pipeline_free(ohmd_gst_pipeline *pipe);

ohmd_gst_video_stream *ohmd_gst_video_stream_new (ohmd_gst_pipeline *pipe, ohmd_pw_video_format format, uint16_t w, uint16_t h, uint16_t fps_n, uint16_t fps_d);
void ohmd_gst_video_stream_push (ohmd_gst_video_stream *v, int64_t pts, const uint8_t *pixels);
void ohmd_gst_video_stream_free (ohmd_gst_video_stream *v);

ohmd_gst_debug_stream *ohmd_gst_debug_stream_new (ohmd_gst_pipeline *pipe);
void ohmd_gst_debug_stream_push (ohmd_gst_debug_stream *s, int64_t pts, const char *debug_str);
void ohmd_gst_debug_stream_free (ohmd_gst_debug_stream *s);

#else

#define ohmd_gst_pipeline_new(s) NULL
#define ohmd_gst_pipeline_free(p)

#define ohmd_gst_video_stream_new(p,f,w,h,fps_n,fps_d) NULL
#define ohmd_gst_video_stream_push(v,pts,pixels)
#define ohmd_gst_video_stream_free(v)

#define ohmd_gst_debug_stream_new(p) NULL
#define ohmd_gst_debug_stream_push(s,pts,debug_str)
#define ohmd_gst_debug_stream_free(s)
#endif


