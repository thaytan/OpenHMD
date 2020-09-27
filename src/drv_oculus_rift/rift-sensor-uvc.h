#ifndef __UVC_H__
#define __UVC_H__

#include <libusb.h>
#include <stdbool.h>
#include <stdint.h>

#define DK2_PID		0x0201
#define CV1_PID		0x0211

typedef struct rift_sensor_uvc_frame rift_sensor_uvc_frame;
typedef struct rift_sensor_uvc_stream rift_sensor_uvc_stream;

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

struct rift_sensor_uvc_stream {
	int stride;
	int width;
	int height;

	/* Frame data destination */
	rift_sensor_uvc_frame *cur_frame;
	/* Total size of a full frame in bytes */
	int frame_size;
	/* Current frame ID */
	int frame_id;
	/* Current PTS being accumulated */
	uint32_t cur_pts;
	/* Number of bytes collected from the current frame */
	int frame_collected;
	/* true if we're skipping the current frame */
	bool skip_frame;

	int num_transfers;
	struct libusb_transfer **transfer;
	int completed;
	int active_transfers;
	void (*sof_cb)(struct rift_sensor_uvc_stream *stream, uint64_t start_ts);
	void (*frame_cb)(struct rift_sensor_uvc_stream *stream, rift_sensor_uvc_frame *frame);
	libusb_context *ctx;
	libusb_device_handle *devh;
	void *user_data;
};

int rift_sensor_uvc_set_cur(libusb_device_handle *devh, uint8_t interface, uint8_t entity,
		uint8_t selector, void *data, uint16_t wLength);
int rift_sensor_uvc_get_cur(libusb_device_handle *devh, uint8_t interface, uint8_t entity,
		uint8_t selector, void *data, uint16_t wLength);

int rift_sensor_uvc_stream_setup (libusb_context *ctx, libusb_device_handle *devh,
		rift_sensor_uvc_stream *stream);
int rift_sensor_uvc_stream_clear (rift_sensor_uvc_stream *stream);

int rift_sensor_uvc_stream_start(rift_sensor_uvc_stream *stream);
int rift_sensor_uvc_stream_stop(rift_sensor_uvc_stream *stream);

/* This can only be called safely either before the stream is started
 * or from one of the callbacks */
bool rift_sensor_uvc_stream_set_frame(rift_sensor_uvc_stream *stream, rift_sensor_uvc_frame *frame);

#endif /* __UVC_H__ */
