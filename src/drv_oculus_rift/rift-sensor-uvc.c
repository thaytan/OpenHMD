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
} __attribute__((packed));

struct uvc_payload_header {
	__u8 bHeaderLength;
	__u8 bmHeaderInfo;
	__le32 dwPresentationTime;
	__u8 scrSourceClock[6];
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
	uint32_t pts;
	bool error;

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
	error = h->bmHeaderInfo & 0x40;

	if (error) {
		printf("frame error\n");
		return;
	}

	pts = __le32_to_cpu(h->dwPresentationTime);
	if (stream->payload_size == 0) {
		stream->pts = pts;
	}

	if (frame_id != stream->frame_id) {
		struct timespec ts;
		uint64_t time;

		if (stream->payload_size != stream->frame_size) {
			printf("Dropping short frame: %u < %u\n",
					stream->payload_size, stream->frame_size);
		}

		/* Start of new frame */
		clock_gettime(CLOCK_MONOTONIC, &ts);
		time = ts.tv_sec * 1000000000 + ts.tv_nsec;
		stream->dt = time - stream->time;

		stream->frame_id = frame_id;
		stream->pts = pts;
		stream->time = time;
		stream->payload_size = 0;
	} else {
		if (pts != stream->pts) {
			printf("PTS changed in-frame at %u!\n",
			       stream->payload_size);
			stream->pts = pts;
		}
	}

	if (stream->payload_size + payload_len > stream->frame_size) {
		printf("frame buffer overflow: %u %u %u\n",
		       stream->payload_size, payload_len, stream->frame_size);
		return;
	}

	memcpy(stream->frame + stream->payload_size, payload,
	       payload_len);
	stream->payload_size += payload_len;

	if (stream->payload_size == stream->frame_size) {
		if (stream->frame_cb)
			stream->frame_cb(stream);
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
	struct uvc_probe_commit_control probe_result;
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
		printf("could claim data interface\n");
		return ret;
	}

	ret = libusb_get_device_descriptor(dev, &desc);
	if (ret < 0)
		return ret;

	switch (desc.idProduct) {
	case CV1_PID:
		control.bFrameIndex = 4;
		control.dwFrameInterval = __cpu_to_le32(192000);
		control.dwMaxVideoFrameSize = __cpu_to_le32(1280 * 960);
		control.dwMaxPayloadTransferSize = __cpu_to_le16(3072);

		stream->stride = 1280;
		stream->width = 1280;
		stream->height = 960;

		num_packets = 24;
		packet_size = 16384;
		alt_setting = 2;
		break;
	}

	ret = rift_sensor_uvc_set_cur(devh, 1, 0, VS_PROBE_CONTROL, &control,
			sizeof control);
	if (ret < 0) {
		printf("failed to set PROBE\n");
		for (int i = 0; i < sizeof control; i++) {
			printf("%02x ", ((uint8_t *)&control)[i]);
		}
		printf("\n");
		return ret;
	}

	ret = rift_sensor_uvc_get_cur(devh, 1, 0, VS_PROBE_CONTROL, &probe_result,
			sizeof probe_result);
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

	ret = libusb_set_interface_alt_setting(devh, 1, alt_setting);
	if (ret) {
		printf("failed to set interface alt setting\n");
		return ret;
	}

	stream->frame_size = stream->stride * stream->height;
	stream->frame = malloc(stream->frame_size);
	if (!stream->frame)
		return -ENOMEM;

	stream->num_transfers = 7;
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
