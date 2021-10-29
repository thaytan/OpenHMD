// Copyright 2021, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* OpenHMD JPEG decoder */
#ifndef __OHMD_JPEG_H__
#define __OHMD_JPEG_H__

#include <stdbool.h>
#include "../openhmdi.h"
#include "ohmd-video.h"

typedef struct ohmd_jpeg_decoder ohmd_jpeg_decoder;

ohmd_jpeg_decoder *ohmd_jpeg_decoder_new(ohmd_context *ohmd_ctx);
void ohmd_jpeg_decoder_free(ohmd_jpeg_decoder *jd);
bool ohmd_jpeg_decoder_decode(ohmd_jpeg_decoder *jd, ohmd_video_frame *in, ohmd_video_frame **out);

#endif
