#ifndef __UVC_H__
#define __UVC_H__

#include <libusb.h>
#include <stdbool.h>
#include <stdint.h>
#include "../ohmd-video.h"
#include "../rift-sensor-usb.h"

typedef struct rift_sensor_uvc_stream rift_sensor_uvc_stream;
typedef void (*rift_sensor_uvc_stream_frame_cb)(struct rift_sensor_uvc_stream *stream, ohmd_video_frame *frame, void *cb_data);

struct rift_sensor_uvc_stream {
	ohmd_context* ohmd_ctx;

	ohmd_video_frame_format format;
	int stride;
	int width;
	int height;

	ohmd_mutex *frames_lock;
	int n_free_frames;
	ohmd_video_frame **free_frames;

	/* Frame data destination */
	ohmd_video_frame *cur_frame;

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

	/* Time at which we started skipping frames */
	uint64_t skip_frame_start;

	/* USB streaming alt_setting */
	int alt_setting;

	int num_transfers;
	struct libusb_transfer **transfer;
	int active_transfers;

	rift_sensor_uvc_stream_frame_cb frame_cb;
	void *frame_cb_data;

	libusb_context *usb_ctx;
	libusb_device_handle *devh;

	bool video_running;
	ohmd_video_frame **alloced_frames;
	int n_alloced_frames;
};

int rift_sensor_uvc_set_cur(libusb_device_handle *devh, uint8_t interface, uint8_t entity,
		uint8_t selector, void *data, uint16_t wLength);
int rift_sensor_uvc_get_cur(libusb_device_handle *devh, uint8_t interface, uint8_t entity,
		uint8_t selector, void *data, uint16_t wLength);

int rift_sensor_uvc_stream_setup (ohmd_context* ohmd_ctx, libusb_context *usb_ctx, libusb_device_handle *devh,
		struct libusb_device_descriptor *desc,
		rift_sensor_uvc_stream *stream);
int rift_sensor_uvc_stream_clear (rift_sensor_uvc_stream *stream);

int rift_sensor_uvc_stream_start(rift_sensor_uvc_stream *stream, uint8_t min_frames,
	rift_sensor_uvc_stream_frame_cb frame_cb, void *frame_cb_data);

int rift_sensor_uvc_stream_stop(rift_sensor_uvc_stream *stream);

/* This can only be called safely either before the stream is started
 * or from one of the callbacks */
bool rift_sensor_uvc_stream_set_frame(rift_sensor_uvc_stream *stream, ohmd_video_frame *frame);

#endif /* __UVC_H__ */
