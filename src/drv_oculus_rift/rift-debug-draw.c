/*
 * Rift debug output functions
 * Copyright 2020 Jan Schmidt <thaytan@noraisin.net>
 * SPDX-License-Identifier: BSL-1.0
 */
#include <assert.h>

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
  int max_x = MIN (width, x_pos + mark_width / 2);
  int min_y = MAX (0, y_pos - mark_height / 2);
  int max_y = MIN (height, y_pos + mark_height / 2);

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
  correspondence_search_t *cs, struct rift_sensor_uvc_stream * stream,
	rift_tracked_device *devs, bool is_cv1,
  dmat3 camera_matrix, bool dist_fisheye, double dist_coeffs[5])
{
	uint8_t *src = stream->frame;
	
	int x, y;
	int width = stream->width;
	int height = stream->height;
	int in_stride = stream->stride;
	int out_stride = stream->width*3*2;

	uint8_t *dest = pixels;
  for (y = 0; y < height; y++) {
    /* Expand to RGB and copy the source to both halves */
    uint8_t *d = dest;
    for (x = 0; x < width; x++) {
      /* Expand GRAY8 to yellow */
      d[0] = d[1] = d[2] = src[x];
      d += 3;
    }

    d = dest + width * 3;
    for (x = 0; x < width; x++) {
      /* Expand GRAY8 to yellow */
      d[0] = d[1] = d[2] = src[x];
      d += 3;
    }

    dest += out_stride;
    src += in_stride;
  }

  const int colours[] = { 0xFF0000, 0x00FF00, 0x0000FF };

  if (bwobs) {
    /* Draw the blobs in the video on the RHS */
    for (int index = 0; index < bwobs->num_blobs; index++) {
			struct blob *b = bwobs->blobs + index;
      int start_x, start_y, w, h;

      start_x = b->x - b->width / 2.0;
      start_y = b->y - b->height / 2.0;
      w = b->width;
      h = b->height;
      clamp_rect (&start_x, &start_y, &w, &h, width, height);

      /* Tint known blobs by their device ID in the RHS image */
			if (b->led_id != LED_INVALID_ID) {
				int d = LED_OBJECT_ID (b->led_id);
				assert (d < 3);

				colour_rgb_rect (pixels + 3*width, width, out_stride, height, start_x,
          start_y, b->width, b->height, colours[d]);
			}
    }

    int d;
		vec3f led_out_points[MAX_OBJECT_LEDS];

    for (d = 0; d < 3; d++) {
      int i;
      rift_pose_metrics score;
      vec3f pose_pos;
      quatf pose_orient;
			rift_tracked_device *dev = devs + d;

      if (!correspondence_search_have_pose (cs, d,
              &pose_orient, &pose_pos, &score))
        continue;

      /* Draw a marker that says we think we saw this device */
			if (score.good_pose_match) {
				draw_rgb_filled_rect (pixels, width, out_stride, height,
						16 * d, 0, 16, 16, colours[d]);
			}

      /* Loop over the LED points a final time and draw markers
       * into the video over the top */

      /* Project HMD LEDs into the image again doh */
      rift_project_points (dev->leds->points,
          dev->leds->num_points, &camera_matrix,
          dist_coeffs, is_cv1, &pose_orient,
          &pose_pos, led_out_points);

      for (i = 0; i < dev->leds->num_points; i++) {
        vec3f *p = led_out_points + i;
        vec3f facing;
        int x = round (p->x);
        int y = round (p->y);

        oquatf_get_rotated (&pose_orient,
            &dev->leds->points[i].dir, &facing);

        if (facing.z < -0.25) {
          /* Camera facing */
          draw_rgb_marker (pixels, width, out_stride, height, x,
              y, 6, 6, colours[d]);
        } else {
          draw_rgb_marker (pixels, width, out_stride, height, x,
              y, 4, 4, 0x202000);
        }
      }
    }
  }
}
