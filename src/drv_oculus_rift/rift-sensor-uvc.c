#include <asm/byteorder.h>
#include <errno.h>
#include <libusb.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rift-sensor-uvc.h"

#define SET_CUR 0x01
#define GET_CUR 0x81
#define TIMEOUT 1000

#define CONTROL_IFACE   0

#define VS_PROBE_CONTROL	1
#define VS_COMMIT_CONTROL	2

#define RIFT_SENSOR_CLOCK_FREQ	(40000000)
#define RIFT_SENSOR_WIDTH       1280
#define RIFT_SENSOR_HEIGHT      960
#define RIFT_SENSOR_FRAME_SIZE  (RIFT_SENSOR_WIDTH * RIFT_SENSOR_HEIGHT)

struct uvc_probe_commit_control {
	__le16 bmHint;
	__u8 bFormatIndex;
	__u8 bFrameIndex;
	__le32 dwFrameInterval;
	__le16 wKeyFrameRate;
	__le16 wPFrameRate;
	__le16 wCompQuality;
	__le16 wCompWindowSize;
	__le16 wDelay;
	__le32 dwMaxVideoFrameSize;
	__le32 dwMaxPayloadTransferSize;
	__le32 dwClockFrequency;
	__u8 bmFramingInfo;
} __attribute__((packed));

struct uvc_payload_header {
	__u8 bHeaderLength;
	__u8 bmHeaderInfo;
	__le32 dwPresentationTime;
	__le16 wSofCounter;
	__le32 scrSourceClock;
} __attribute__((packed));

int rift_sensor_uvc_set_cur(libusb_device_handle *dev, uint8_t interface, uint8_t entity,
		uint8_t selector, void *data, uint16_t wLength)
{
	uint8_t bmRequestType = LIBUSB_REQUEST_TYPE_CLASS |
		LIBUSB_RECIPIENT_INTERFACE;
	uint8_t bRequest = SET_CUR;
	uint16_t wValue = selector << 8;
	uint16_t wIndex = entity << 8 | interface;
	unsigned int timeout = TIMEOUT;
	int ret;

	ret = libusb_control_transfer(dev, bmRequestType, bRequest, wValue,
			wIndex, data, wLength, timeout);
	if (ret < 0) {
		fprintf(stderr, "failed to transfer SET CUR %u %u: %d %d %m\n",
			entity, selector, ret, errno);
	}
	return ret;
}

int rift_sensor_uvc_get_cur(libusb_device_handle *dev, uint8_t interface, uint8_t entity,
		uint8_t selector, void *data, uint16_t wLength)
{
	uint8_t bmRequestType = 0x80 | LIBUSB_REQUEST_TYPE_CLASS |
		LIBUSB_RECIPIENT_INTERFACE;
	uint8_t bRequest = GET_CUR;
	uint16_t wValue = selector << 8;
	uint16_t wIndex = entity << 8 | interface;
	unsigned int timeout = TIMEOUT;
	int ret;

	ret = libusb_control_transfer(dev, bmRequestType, bRequest, wValue,
			wIndex, data, wLength, timeout);
	if (ret < 0) {
		fprintf(stderr, "failed to transfer GET CUR %u %u: %d %d %m\n",
			entity, selector, ret, errno);
	}
	return ret;
}

void process_payload(struct rift_sensor_uvc_stream *stream, unsigned char *payload,
		     size_t len)
{
	struct uvc_payload_header *h = (struct uvc_payload_header *)payload;
	int payload_len;
	int frame_id;
	uint32_t pts = (uint32_t)(-1);
	uint32_t scr = (uint32_t)(-1);
	bool error, have_pts, have_scr, is_eof;

	if (len == 0)
		return;
	if (len == 12)
		return;

	if (h->bHeaderLength != 12) {
		printf("invalid header: len %u/%ld\n", h->bHeaderLength, len);
		return;
	}

	payload += h->bHeaderLength;
	payload_len = len - h->bHeaderLength;
	frame_id = h->bmHeaderInfo & 0x01;
	is_eof = h->bmHeaderInfo & 0x02;
	have_pts = h->bmHeaderInfo & 0x04;
	have_scr = h->bmHeaderInfo & 0x08;
	error = h->bmHeaderInfo & 0x40;

	if (error) {
		printf("UVC frame error\n");
		return;
	}

	if (have_pts) {
		pts = __le32_to_cpu(h->dwPresentationTime);
		if (stream->payload_size != 0 && pts != stream->pts) {
			printf("UVC PTS changed in-frame at %u bytes. Lost %u ms\n",
			    stream->payload_size,
			    (pts - stream->pts * 1000) / RIFT_SENSOR_CLOCK_FREQ);
			stream->pts = pts;
		}
	}
	if (have_scr) {
		scr = __le32_to_cpu(h->scrSourceClock);
	}

	if (frame_id != stream->frame_id) {
		struct timespec ts;
		uint64_t time;

		if (stream->payload_size > 0) {
			printf("UVC Dropping short frame: %u < %u (%d lost)\n",
			    stream->payload_size, stream->frame_size,
			    stream->frame_size - stream->payload_size);
		}

		/* Start of new frame */
		clock_gettime(CLOCK_MONOTONIC, &ts);
		time = ts.tv_sec * 1000000000 + ts.tv_nsec;
		stream->dt = time - stream->time;

		stream->frame_id = frame_id;
		stream->pts = pts;
		stream->time = time;
		stream->payload_size = 0;

#if 0
		printf ("UVC dt %f PTS %f SCR %f delta %d\n",
		    (double) (stream->dt) / (1000000000.0),
		    (double) (pts) / RIFT_SENSOR_CLOCK_FREQ,
		    (double) (scr) / RIFT_SENSOR_CLOCK_FREQ,
		    scr - pts);
#endif
	} else {
		if (stream->payload_size > 0 && have_pts && pts != stream->pts) {
			//printf("UVC PTS changed in-frame at %u!\n",
			       //stream->payload_size);
			stream->pts = pts;
		}
	}

	if (stream->payload_size + payload_len > stream->frame_size) {
		printf("UVC frame buffer overflow: %u + %u > %u\n",
		       stream->payload_size, payload_len, stream->frame_size);
		return;
	}

	memcpy(stream->frame + stream->payload_size, payload,
	       payload_len);
	stream->payload_size += payload_len;

	if (stream->payload_size == stream->frame_size) {
		if (stream->frame_cb)
			stream->frame_cb(stream);
		stream->payload_size = 0;
	}

	if (is_eof) {
		/* Always restart a frame after eof.
		 * CV1 sensor never seems to set this
		 * bit, but others might in the future */
		stream->payload_size = 0;
	}
}

static void iso_transfer_cb(struct libusb_transfer *transfer)
{
	struct rift_sensor_uvc_stream *stream = transfer->user_data;
	int ret;
	int i;

	/* Handle error conditions */
	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		printf("transfer error: %u\n", transfer->status);
		return;
	}

	/* Handle contained isochronous packets */
	for (i = 0; i < transfer->num_iso_packets; i++) {
		unsigned char *payload;
		size_t payload_len;

		payload = libusb_get_iso_packet_buffer_simple(transfer, i);
		payload_len = transfer->iso_packet_desc[i].actual_length;
		process_payload(stream, payload, payload_len);
	}

	if (stream->completed)
		return;

	/* Resubmit transfer */
	ret = libusb_submit_transfer(transfer);
	if (ret < 0) {
		printf("failed to resubmit\n");
		stream->completed = true;
	}
}

static void *uvc_handle_events(void *arg)
{
	struct rift_sensor_uvc_stream *stream = arg;

	while (!stream->completed)
		libusb_handle_events_completed(stream->ctx, &stream->completed);

	return NULL;
}

int rift_sensor_uvc_stream_start(libusb_context *ctx, libusb_device_handle *devh,
                     struct rift_sensor_uvc_stream *stream)
{
	libusb_device *dev = libusb_get_device(devh);
	struct libusb_device_descriptor desc;
	struct uvc_probe_commit_control control = {
		.bFormatIndex = 1,
		.bFrameIndex = 1,
	};
	int alt_setting;
	int ret;
	int num_packets;
	int packet_size;

	ret = libusb_set_auto_detach_kernel_driver(devh, 1);
	if (ret < 0) {
		printf("could not detach uvcvideo driver\n");
		return ret;
	}

	ret = libusb_claim_interface(devh, 0);
	if (ret < 0) {
		printf("could not claim control interface\n");
		return ret;
	}

	ret = libusb_claim_interface(devh, 1);
	if (ret < 0) {
		printf("could not claim UVC data interface\n");
		return ret;
	}

	ret = libusb_get_device_descriptor(dev, &desc);
	if (ret < 0)
		return ret;

	switch (desc.idProduct) {
	case CV1_PID:
		control.bFrameIndex = 4;
		control.dwFrameInterval = __cpu_to_le32(192000);
		control.dwMaxVideoFrameSize = __cpu_to_le32(RIFT_SENSOR_FRAME_SIZE);
		control.dwMaxPayloadTransferSize = __cpu_to_le16(3072);
		control.dwClockFrequency = __cpu_to_le32(RIFT_SENSOR_CLOCK_FREQ);

		stream->stride = RIFT_SENSOR_WIDTH;
		stream->width = RIFT_SENSOR_WIDTH;
		stream->height = RIFT_SENSOR_HEIGHT;

		alt_setting = 2;
		break;
	}

	ret = rift_sensor_uvc_set_cur(devh, 1, 0, VS_PROBE_CONTROL, &control,
			sizeof control);
	if (ret < 0)
		return ret;

	ret = rift_sensor_uvc_get_cur(devh, 1, 0, VS_PROBE_CONTROL, &control,
			sizeof control);
	if (ret < 0) {
		printf("failed to get PROBE\n");
		return ret;
	}

	ret = rift_sensor_uvc_set_cur(devh, 1, 0, VS_COMMIT_CONTROL, &control,
			sizeof control);
	if (ret < 0) {
		printf("failed to set COMMIT\n");
		return ret;
	}

	control.dwFrameInterval = __le32_to_cpu(control.dwFrameInterval);
	control.wDelay = __le16_to_cpu(control.wDelay);
	control.dwMaxVideoFrameSize = __le32_to_cpu(control.dwMaxVideoFrameSize);
	control.dwClockFrequency = __le32_to_cpu(control.dwClockFrequency);

	control.dwMaxPayloadTransferSize = __le32_to_cpu(control.dwMaxPayloadTransferSize);
	packet_size = 16384;

	ret = libusb_set_interface_alt_setting(devh, 1, alt_setting);
	if (ret) {
		printf("failed to set interface alt setting\n");
		return ret;
	}

	stream->frame_size = stream->stride * stream->height;
	stream->frame = malloc(stream->frame_size);
	if (!stream->frame)
		return -ENOMEM;

	num_packets = (stream->frame_size + packet_size - 1) / packet_size;
	stream->num_transfers = (num_packets + 31) / 32;
	num_packets = num_packets / stream->num_transfers;

	stream->transfer = calloc(stream->num_transfers,
			sizeof(*stream->transfer));
	if (!stream->transfer)
		return -ENOMEM;

	for (int i = 0; i < stream->num_transfers; i++) {
		stream->transfer[i] = libusb_alloc_transfer(32);
		if (!stream->transfer[i]) {
			fprintf(stderr, "failed to allocate isochronous transfer\n");
			return -ENOMEM;
		}

		uint8_t bEndpointAddress = 0x81;
		int transfer_size = num_packets * packet_size;
		void *buf = malloc(transfer_size);
		libusb_fill_iso_transfer(stream->transfer[i], devh,
					 bEndpointAddress, buf, transfer_size,
					 num_packets, iso_transfer_cb, stream,
					 TIMEOUT);
		libusb_set_iso_packet_lengths(stream->transfer[i], packet_size);

		ret = libusb_submit_transfer(stream->transfer[i]);
		if (ret < 0) {
			fprintf(stderr, "failed to submit iso transfer %d\n", i);
			return ret;
		}
	}

	stream->ctx = ctx;
	stream->devh = devh;

	pthread_create(&stream->thread, NULL, uvc_handle_events, stream);

	stream->completed = false;

	return 0;
}

int rift_sensor_uvc_stream_stop(struct rift_sensor_uvc_stream *stream)
{
	int ret;
	int i;

	ret = libusb_set_interface_alt_setting(stream->devh, 1, 0);
	if (ret)
		return ret;

	stream->completed = true;

	pthread_join(stream->thread, NULL);

	for (i = 0; i < stream->num_transfers; i++)
		libusb_free_transfer(stream->transfer[i]);
	free(stream->transfer);
	free(stream->frame);

	return 0;

}
