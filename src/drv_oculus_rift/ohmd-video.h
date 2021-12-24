// Copyright 2021, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - video frame handling */

#include <stdint.h>

#ifndef __OHMD_VIDEO_H__
#define __OHMD_VIDEO_H__

typedef struct ohmd_video_frame ohmd_video_frame;
typedef void (*ohmd_video_frame_release_func)(ohmd_video_frame *frame, void *owner);

typedef enum {
	OHMD_VIDEO_FRAME_FORMAT_GRAY8,
	OHMD_VIDEO_FRAME_FORMAT_RGB,
	OHMD_VIDEO_FRAME_FORMAT_JPEG,
} ohmd_video_frame_format;

struct ohmd_video_frame {
	ohmd_video_frame_format format;

	/* Source specific PTS from the camera data */
	uint64_t pts;
	/* Posix monotonic time of frame start */
	uint64_t start_ts;

	/* Pixel data pointer and size to be set by the allocator */
	unsigned char *data;
	size_t data_block_size;
	/* bytes of the data block that are filled / valid (must be less than or
	 * equal to data_block_size) */
	size_t data_size;

	int stride;
	int width;
	int height;

	void *owner;
	ohmd_video_frame_release_func releasefn;
};

void ohmd_video_frame_release(ohmd_video_frame *frame);

#endif
