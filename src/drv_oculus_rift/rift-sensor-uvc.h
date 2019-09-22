#ifndef __UVC_H__
#define __UVC_H__

#include <stdbool.h>
#include <stdint.h>

#define CV1_PID		0x0211

struct rift_sensor_uvc_stream {
	int stride;
	int width;
	int height;
	unsigned char *frame;
	int frame_size;
	int payload_size;
	int frame_id;
	uint32_t pts;
	uint64_t time;
	int64_t dt;
	int num_transfers;
	struct libusb_transfer **transfer;
	int completed;
	int active_transfers;
	void (*sof_cb)(struct rift_sensor_uvc_stream *stream);
	void (*frame_cb)(struct rift_sensor_uvc_stream *stream);
	libusb_context *ctx;
	libusb_device_handle *devh;
	void *user_data;
};

int rift_sensor_uvc_set_cur(libusb_device_handle *devh, uint8_t interface, uint8_t entity,
		uint8_t selector, void *data, uint16_t wLength);
int rift_sensor_uvc_get_cur(libusb_device_handle *devh, uint8_t interface, uint8_t entity,
		uint8_t selector, void *data, uint16_t wLength);

int rift_sensor_uvc_stream_setup (libusb_context *ctx, libusb_device_handle *devh,
         struct rift_sensor_uvc_stream *stream);
int rift_sensor_uvc_stream_clear (struct rift_sensor_uvc_stream *stream);

int rift_sensor_uvc_stream_start(struct rift_sensor_uvc_stream *stream);
int rift_sensor_uvc_stream_stop(struct rift_sensor_uvc_stream *stream);

#endif /* __UVC_H__ */
