// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - positional tracking interface */

#include <stdint.h>

#ifndef __RIFT_SENSOR_COMMON_H__
#define __RIFT_SENSOR_COMMON_H__

typedef struct rift_sensor_uvc_frame rift_sensor_uvc_frame;

struct rift_sensor_uvc_frame {
	/* Pixel data pointer and size to be set by the allocator */
	unsigned char *data;
	int data_size;

	int stride;
	int width;
	int height;
	/* PTS from the camera data */
	uint32_t pts;
	/* Posix monotonic time of frame start */
	uint64_t start_ts;
};

#endif
