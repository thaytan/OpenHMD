/*
 * Rift position tracking
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

#include <libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "rift-sensor-tracker.h"

#include "rift-sensor-blobwatch.h"
#include "rift-sensor-ar0134.h"
#include "rift-sensor-esp770u.h"
#include "rift-sensor-uvc.h"

#define ASSERT_MSG(_v, ...) if(!(_v)){ fprintf(stderr, __VA_ARGS__); exit(1); }

/* change this to the Rift HMD's radio id */
#define RIFT_RADIO_ID 0x3c309307

struct rift_sensor_ctx_s
{
  libusb_context *usb_ctx;
	libusb_device_handle *usb_devh;

  int stream_started;
  struct rift_sensor_uvc_stream stream;
  struct blobwatch* bw;
	struct blobservation* bwobs;
};

#define min(a,b) ((a) < (b) ? (a) : (b))

static void new_frame_cb(struct rift_sensor_uvc_stream *stream)
{
	int width = stream->width;
	int height = stream->height;

	printf(".");
	if(stream->payload_size != width * height) {
		printf("bad frame: %d\n", (int)stream->payload_size);
	}

	rift_sensor_ctx *sensor_ctx = stream->user_data;
	assert (sensor_ctx);

#if 1
	blobwatch_process(sensor_ctx->bw, stream->frame, width, height, 0, NULL, &sensor_ctx->bwobs);

	if (sensor_ctx->bwobs)
	{
		if (sensor_ctx->bwobs->num_blobs > 0)
		{
			printf("Blobs: %d\n", sensor_ctx->bwobs->num_blobs);

			for (int index = 0; index < sensor_ctx->bwobs->num_blobs; index++)
			{
				printf("Blob[%d]: %d,%d\n",
					index,
					sensor_ctx->bwobs->blobs[index].x,
					sensor_ctx->bwobs->blobs[index].y);
			}
		}
	}
#endif
}

int
rift_sensor_tracker_init (rift_sensor_ctx **ctx)
{
  rift_sensor_ctx *sensor_ctx = NULL;
  int ret;

  sensor_ctx = calloc(1, sizeof (rift_sensor_ctx));

  ret = libusb_init(&sensor_ctx->usb_ctx);
  ASSERT_MSG(ret >= 0, "could not initialize libusb\n");

  /* FIXME: Traverse USB devices with libusb_get_device_list() */
  sensor_ctx->usb_devh = libusb_open_device_with_vid_pid(sensor_ctx->usb_ctx, 0x2833, CV1_PID);
  ASSERT_MSG(sensor_ctx->usb_devh, "could not find or open the Rift sensor camera\n");

  sensor_ctx->stream.frame_cb = new_frame_cb;
  sensor_ctx->stream.user_data = sensor_ctx;

  ret = rift_sensor_uvc_stream_start(sensor_ctx->usb_ctx, sensor_ctx->usb_devh, &sensor_ctx->stream);
  ASSERT_MSG(ret >= 0, "could not start streaming\n");
  sensor_ctx->stream_started = 1;

  sensor_ctx->bw = blobwatch_new(sensor_ctx->stream.width, sensor_ctx->stream.height);

  ret = rift_sensor_ar0134_init(sensor_ctx->usb_devh);
  if (ret < 0)
    goto fail;

  ret = rift_sensor_esp770u_setup_radio(sensor_ctx->usb_devh, RIFT_RADIO_ID);
  if (ret < 0)
    goto fail;

  *ctx = sensor_ctx;
  return 0;

fail:
  if (sensor_ctx)
    rift_sensor_tracker_free (sensor_ctx);
  return ret;
}

void
rift_sensor_tracker_free (rift_sensor_ctx *sensor_ctx)
{
  if (!sensor_ctx)
    return;

  if (sensor_ctx->stream_started)
    rift_sensor_uvc_stream_stop(&sensor_ctx->stream);

  if (sensor_ctx->usb_devh)
    libusb_close (sensor_ctx->usb_devh);
  if (sensor_ctx->usb_ctx)
    libusb_exit (sensor_ctx->usb_ctx);
  free (sensor_ctx);
}
