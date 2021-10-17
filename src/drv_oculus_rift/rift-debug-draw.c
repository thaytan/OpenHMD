/*
 * Rift debug output functions
 * Copyright 2020 Jan Schmidt <thaytan@noraisin.net>
 * SPDX-License-Identifier: BSL-1.0
 */
#include <assert.h>
#include <string.h>

#include "rift-debug-draw.h"
#include "rift-sensor-pose-helper.h"
#include "rift-sensor-opencv.h"

/* This will draw RGB flipped on big-endian */
#define WRITE_UINT24_BE(dest, colour) dest[2] = colour & 0xff; dest[1] = colour >> 8 & 0xff; dest[0] = colour >> 16 & 0xff

#define MAX(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })
#define MIN(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b; })

static void
draw_rgb_marker (uint8_t * pixels, int width, int stride, int height,
    int x_pos, int y_pos, int mark_width, int mark_height, uint32_t colour)
{
  int x, y;
  int min_x = MAX (0, x_pos - mark_width / 2);
  int max_x = MIN (width, x_pos + mark_width / 2 + 1.5);
  int min_y = MAX (0, y_pos - mark_height / 2);
  int max_y = MIN (height, y_pos + mark_height / 2 + 1.5);

  if (y_pos < 0 || y_pos >= height)
    return;
  if (x_pos < 0 || x_pos >= width)
    return;

  /* Horizontal line */
  uint8_t *dest = pixels + stride * y_pos + 3 * min_x;
  for (x = 0; x < max_x - min_x; x++) {
    WRITE_UINT24_BE (dest, colour);
    dest += 3;
  }

  /* Vertical line */
  dest = pixels + stride * min_y + 3 * x_pos;
  for (y = 0; y < max_y - min_y; y++) {
    WRITE_UINT24_BE (dest, colour);
    dest += stride;
  }
}

static void
clamp (int * val, int max)
{
  if (*val < 0)
    *val = 0;
  if (*val >= max)
    *val = max - 1;
}

static void
clamp_rect (int * x, int * y, int * rw, int * rh, int width, int height)
{
  clamp (x, width);
  clamp (y, height);
  clamp (rw, width - *x);
  clamp (rh, height - *y);
}

#if 0
/* Draw a single-pixel rect outline */
static void
draw_rgb_rect (uint8_t * pixels, int width, int stride, int height,
    int start_x, int start_y, int box_width, int box_height, uint32_t colour)
{
  clamp_rect (&start_x, &start_y, &box_width, &box_height, width, height);

  int x, y;
  uint8_t *dest = pixels + stride * start_y + 3 * start_x;
  for (x = 0; x < box_width; x++) {
    WRITE_UINT24_BE (dest, colour);
    dest += 3;
  }

  for (y = 1; y < box_height - 1; y++) {
    dest = pixels + stride * (start_y + y) + 3 * start_x;

    WRITE_UINT24_BE (dest, colour);
    dest += 3 * (box_width - 1);
    WRITE_UINT24_BE (dest, colour);
  }

  dest = pixels + stride * (start_y + box_height - 1) + 3 * start_x;
  for (x = 0; x < box_width; x++) {
    WRITE_UINT24_BE (dest, colour);
    dest += 3;
  }
}
#endif

static void
colour_rgb_rect (uint8_t *pixels, int width, int stride,
    int height, int start_x, int start_y, int box_width,
    int box_height, uint32_t mask_colour)
{
  clamp_rect (&start_x, &start_y, &box_width, &box_height, width, height);

  int x, y;
  uint8_t *dest;
	/* This will draw RGB flipped on big-endian */
  int r = mask_colour >> 16;
  int g = (mask_colour >> 8) & 0xFF;
  int b = mask_colour & 0xFF;

  for (y = 0; y < box_height; y++) {
    dest = pixels + stride * (start_y + y) + 3 * start_x;
    for (x = 0; x < box_width; x++) {
      dest[0] = (r * dest[0]) >> 8;
      dest[1] = (g * dest[1]) >> 8;
      dest[2] = (b * dest[2]) >> 8;
      dest += 3;
    }
  }
}

static void
draw_rgb_filled_rect (uint8_t * pixels, int width, int stride, int height,
    int start_x, int start_y, int box_width, int box_height, uint32_t colour)
{
  clamp_rect (&start_x, &start_y, &box_width, &box_height, width, height);

  int x, y;
  uint8_t *dest;
  for (y = 0; y < box_height; y++) {
    dest = pixels + stride * (start_y + y) + 3 * start_x;
    for (x = 0; x < box_width; x++) {
      WRITE_UINT24_BE (dest, colour);
      dest += 3;
    }
  }
}

int
compare_blobs (const void *elem1, const void *elem2)
{
  const struct blob *b1 = *(struct blob **) elem1;
  const struct blob *b2 = *(struct blob **) elem2;

  if (b1->y > b2->y)
    return 1;
  if (b1->y < b2->y)
    return -1;

  if (b1->x > b2->x)
    return 1;
  if (b1->x < b2->x)
    return -1;

  return 0;
}

void rift_debug_draw_frame (uint8_t *pixels, struct blobservation* bwobs,
	correspondence_search_t *cs, struct rift_sensor_analysis_frame *frame,
	uint8_t n_devs, rift_tracked_device **devs,
	rift_sensor_camera_params *calib, posef *camera_pose)
{
	ohmd_video_frame *vframe = frame->vframe;
	uint8_t *src = vframe->data;
	int x, y;
	int width = vframe->width;
	int height = vframe->height;
	int in_stride = vframe->stride;
	int out_stride = vframe->width*3*2;

	uint8_t *lhs_dest = pixels;
	uint8_t *rhs_dest = pixels + width * 3;

	uint8_t *dest_line = lhs_dest;
	for (y = 0; y < height; y++) {
		/* Expand to RGB and copy the source to the LHS, paint the
		 * RHS black, and later draw only the pixels that were
		 * detected as blobs */

		uint8_t *d = dest_line;
		for (x = 0; x < width; x++) {
			/* Expand GRAY8 to RGB */
			d[0] = d[1] = d[2] = src[x];
			d += 3;
		}
		memset (dest_line + width * 3, 0, width*3);

		dest_line += out_stride;
		src += in_stride;
	}

	const int colours[] = { 0xFF0000, 0x00FF00, 0x0000FF };

	if (bwobs) {
		/* Draw the blobs in the video */
		for (int index = 0; index < bwobs->num_blobs; index++) {
			struct blob *b = bwobs->blobs + index;
			int start_x, start_y, w, h;

			start_x = b->left;
			start_y = b->top;
			w = b->width;
			h = b->height;
			clamp_rect (&start_x, &start_y, &w, &h, width, height);

			/* Draw the original pixels in the RHS that are within blobs */
			uint8_t *dest = rhs_dest + start_y * out_stride + start_x*3;
			uint8_t *src = vframe->data + start_y * in_stride;
			for (y = 0; y < h; y++) {
				uint8_t *d = dest;
				for (x = start_x; x < start_x + w; x++) {
					/* Expand GRAY8 to RGB */
					d[0] = d[1] = d[2] = src[x];
					d += 3;
				}

				dest += out_stride;
				src += in_stride;
			}

			/* Tint known blobs by their device ID in the RHS image */
			if (b->led_id != LED_INVALID_ID) {
				int d = LED_OBJECT_ID (b->led_id);
				assert (d < 3);

				colour_rgb_rect (rhs_dest, width, out_stride, height, start_x,
					start_y, b->width, b->height, colours[d]);

				/* Draw a dot at the weighted center */
				draw_rgb_marker (rhs_dest, width, out_stride, height,
						round(b->x), round(b->y), 0, 0, colours[d]);
			}
			else {
				/* Draw a dot at the weighted center in purple */
				draw_rgb_marker (rhs_dest, width, out_stride, height,
						round(b->x), round(b->y), 0, 0, 0xFF00FF);
			}
		}

		int d;
		vec3f led_out_points[MAX_OBJECT_LEDS];

		for (d = 0; d < n_devs; d++) {
			int i;
			posef pose;
			rift_tracked_device *dev = devs[d];
			rift_sensor_frame_device_state *dev_state = frame->capture_state + d;

			assert(dev->leds != NULL);

			/* Draw the capture pose blobs in yellow */
			oposef_apply_inverse(&frame->capture_state[d].capture_world_pose, camera_pose, &pose);

			rift_project_points (dev->leds->points,
				dev->leds->num_points, calib, &pose, led_out_points);

			for (i = 0; i < dev->leds->num_points; i++) {
				rift_led *leds = dev->leds->points;
				vec3f *p = led_out_points + i;
				vec3f normal, position;
				double facing_dot;
				int x = round (p->x);
				int y = round (p->y);

				oquatf_get_rotated(&pose.orient, &leds[i].pos, &position);
				ovec3f_add (&pose.pos, &position, &position);

				ovec3f_normalize_me (&position);
				oquatf_get_rotated(&pose.orient, &leds[i].dir, &normal);
				facing_dot = ovec3f_get_dot (&position, &normal);

				if (facing_dot < cos(DEG_TO_RAD(180.0 - RIFT_LED_ANGLE))) {
					/* Camera facing */
					draw_rgb_marker (lhs_dest, width, out_stride, height, x,
						y, 3, 3, 0x008080);
				} else {
					/* Draw non-visible LEDs */
					draw_rgb_marker (lhs_dest, width, out_stride, height, x,
						y, 3, 3, 0x404040);
				}
			}

			if (!dev_state->found_device_pose)
				continue;

			/* Draw a marker that says we think we saw this device */
			draw_rgb_filled_rect (pixels, width, out_stride, height,
				16 * dev->id, 0, 16, 16, colours[dev->id]);

			/* Loop over the LED points a final time and draw markers
			 * for the detected pose into the video over the top of the RHS */

			/* Project HMD LEDs using the extracted pose into the RHS */
			pose = dev_state->final_cam_pose;
			rift_project_points (dev->leds->points,
					dev->leds->num_points, calib, &pose, led_out_points);

			for (i = 0; i < dev->leds->num_points; i++) {
				rift_led *leds = dev->leds->points;
				vec3f *p = led_out_points + i;
				vec3f normal, position;
				double facing_dot;
				int x = round (p->x);
				int y = round (p->y);

				oquatf_get_rotated(&pose.orient, &leds[i].pos, &position);
				ovec3f_add (&pose.pos, &position, &position);

				ovec3f_normalize_me (&position);
				oquatf_get_rotated(&pose.orient, &leds[i].dir, &normal);
				facing_dot = ovec3f_get_dot (&position, &normal);

				if (facing_dot < cos(DEG_TO_RAD(180.0 - RIFT_LED_ANGLE))) {
					/* Camera facing */
					draw_rgb_marker (rhs_dest, width, out_stride, height, x,
							y, 3, 3, colours[dev->id]);
				} else {
#if 0 /* Don't draw occluded LEDs for the extracted pose */
					draw_rgb_marker (rhs_dest, width, out_stride, height, x,
							y, 3, 3, 0x202000);
#endif
				}
			}
		}
	}
}
