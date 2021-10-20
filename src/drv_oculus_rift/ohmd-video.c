/*
 * OpenHMD video frame handling
 * Copyright 2021 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */
#include <stddef.h>
#include <stdlib.h>
#include "ohmd-video.h"

void ohmd_video_frame_release(ohmd_video_frame *frame)
{
	if (frame->releasefn != NULL) {
		frame->releasefn(frame, frame->owner);
	}
	else {
		if (frame->data)
			free(frame->data);
		free(frame);
	}
}
