/*
 * OpenHMD JPEG video frame decoding
 * Copyright 2021 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */
#include <assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <setjmp.h>
#include <errno.h>

#include "ohmd-jpeg.h"

#include "jpeglib.h"

#define MAX_QUEUE_FRAMES 4

typedef struct ohmd_jpeg_decoder_err ohmd_jpeg_decoder_err;

struct ohmd_jpeg_decoder_err {
	struct jpeg_error_mgr base;

	char error_msg[JMSG_LENGTH_MAX];
	jmp_buf setjmp_buffer;
};

struct ohmd_jpeg_decoder {
	ohmd_context *ohmd_ctx;

	struct jpeg_decompress_struct cinfo;
	ohmd_jpeg_decoder_err err_mgr;

	ohmd_mutex *frames_lock;

	ohmd_video_frame **free_frames;
	int n_free_frames;

	ohmd_video_frame **alloced_frames;
	int n_alloced_frames;
};

/* Error handling */
METHODDEF (void) handle_jpeg_error(j_common_ptr cinfo)
{
	ohmd_jpeg_decoder_err *err_mgr = (ohmd_jpeg_decoder_err *) cinfo->err;

	( *(cinfo->err->format_message) ) (cinfo, err_mgr->error_msg);
	LOGI("JPEG decode error: %s", err_mgr->error_msg);

	/* Jump to the setjmp point */
	longjmp(err_mgr->setjmp_buffer, 1);
}

METHODDEF (void) handle_output_message(j_common_ptr cinfo)
{
	return;
}

METHODDEF (void) handle_emit_message (j_common_ptr cinfo, int msg_level)
{
	return;
}

static void
ohmd_jpeg_release_frame(ohmd_video_frame *frame, ohmd_jpeg_decoder *jd);

ohmd_jpeg_decoder *ohmd_jpeg_decoder_new(ohmd_context *ohmd_ctx)
{
	ohmd_jpeg_decoder *jd = ohmd_alloc(ohmd_ctx, sizeof(ohmd_jpeg_decoder));

	jd->ohmd_ctx = ohmd_ctx;
	jd->cinfo.err = jpeg_std_error(&jd->err_mgr.base);
	jd->err_mgr.base.output_message = handle_output_message;
	jd->err_mgr.base.emit_message = handle_emit_message;
	jd->err_mgr.base.error_exit = handle_jpeg_error;
	jpeg_create_decompress(&jd->cinfo);

	jd->frames_lock = ohmd_create_mutex(ohmd_ctx);

	/* Allocate frame store. They will be allocated as needed */
	jd->alloced_frames = ohmd_alloc(ohmd_ctx, MAX_QUEUE_FRAMES * sizeof (ohmd_video_frame *));
	jd->free_frames = ohmd_alloc(ohmd_ctx, MAX_QUEUE_FRAMES * sizeof (ohmd_video_frame *));

	jd->n_free_frames = jd->n_alloced_frames = 0;

	return jd;
}

void ohmd_jpeg_decoder_free(ohmd_jpeg_decoder *jd)
{
	int i;

	jpeg_destroy_decompress(&jd->cinfo);

	/* Free frames */
	for (i = 0; i < jd->n_alloced_frames; i++) {
		ohmd_video_frame *frame = jd->alloced_frames[i];
		free(frame->data);
		free(frame);
	}
	free(jd->alloced_frames);
	free(jd->free_frames);

	if (jd->frames_lock)
		ohmd_destroy_mutex(jd->frames_lock);
	free(jd);
}

static bool get_output_frame(ohmd_jpeg_decoder *jd, ohmd_video_frame *in, ohmd_video_frame **out)
{
	ohmd_lock_mutex(jd->frames_lock);
	while (jd->n_free_frames > 0) {
		jd->n_free_frames--;
		ohmd_video_frame *cur_frame = jd->free_frames[jd->n_free_frames];

		/* Frame format must match or something is very wrong */
		assert (in->width == cur_frame->width && in->height == cur_frame->height);
		*out = cur_frame;
		goto done;
	}

	/* No free frame available, allocate one. assert if there's no room
	 * because the limit is too small */
	assert(jd->n_alloced_frames < MAX_QUEUE_FRAMES);

	ohmd_video_frame *frame = ohmd_alloc(jd->ohmd_ctx, sizeof (ohmd_video_frame));
	size_t data_size = in->width * in->height;

	frame->format = OHMD_VIDEO_FRAME_FORMAT_GRAY8;
	frame->data = ohmd_alloc(jd->ohmd_ctx, data_size);
	frame->data_block_size = frame->data_size = data_size;

	frame->releasefn = (ohmd_video_frame_release_func) ohmd_jpeg_release_frame;
	frame->owner = jd;
	jd->alloced_frames[jd->n_alloced_frames++] = frame;

	*out = frame;

done:
	ohmd_unlock_mutex(jd->frames_lock);
	return true;
}

bool ohmd_jpeg_decoder_decode(ohmd_jpeg_decoder *jd, ohmd_video_frame *in, ohmd_video_frame **out)
{
	*out = NULL;

	if (setjmp(jd->err_mgr.setjmp_buffer)) {
		/* If an error occurs, it will longjmp back here, indicating failure */
		goto fail;
	}

	jpeg_mem_src(&jd->cinfo, in->data, in->data_size);

	int ret = jpeg_read_header(&jd->cinfo, TRUE);
	if (ret != JPEG_HEADER_OK) {
		LOGD("Corrupted JPEG frame");
		goto fail;
	}

	if (jd->cinfo.jpeg_color_space != JCS_YCbCr &&
	    jd->cinfo.jpeg_color_space != JCS_GRAYSCALE &&
	    jd->cinfo.jpeg_color_space != JCS_RGB) {
		LOGD("Unsupported JPEG frame colorspace %u", jd->cinfo.jpeg_color_space);
		goto fail;
	}

	jd->cinfo.out_color_space = JCS_GRAYSCALE;

	if (!jpeg_start_decompress(&jd->cinfo)) {
		LOGW("Failed to initialise JPEG decompression");
		goto fail;
	}

	/* Check the frame is of the expected dimension */
	if (in->width != jd->cinfo.output_width || in->height != jd->cinfo.output_height ||
			jd->cinfo.output_components != 1) {
		LOGW("Unexpected size / format JPEG frame (width %d vs %d, height %d vs %d, %d components",
				in->width, jd->cinfo.output_width, in->height, jd->cinfo.output_height, jd->cinfo.output_components);
		goto fail;
	}

	if (!get_output_frame(jd, in, out)) {
		goto fail;
	}

	/* Copy the details. Only the format is changing */
	(*out)->start_ts = in->start_ts;
	(*out)->pts = in->pts;
	(*out)->stride = in->stride;
	(*out)->width = in->width;
	(*out)->height = in->height;

	while (jd->cinfo.output_scanline < jd->cinfo.output_height) {
		unsigned char *buffer_array[1];
		JDIMENSION nlines = OHMD_MIN(jd->cinfo.output_height - jd->cinfo.output_scanline,
													jd->cinfo.rec_outbuf_height);

		buffer_array[0] = (*out)->data + (jd->cinfo.output_scanline * (*out)->width);

		jpeg_read_scanlines(&jd->cinfo, buffer_array, nlines);
	}

	jpeg_finish_decompress(&jd->cinfo);

#if 0
	/* Debug code to dump frames to files */
	char fname[128];
	static int file_index = 0;
	sprintf(fname, "/tmp/openhmd-jpeg/frame-%05d.jpg", file_index++);
	FILE *f = fopen(fname, "wb");
	if (f == NULL) {
		fprintf(stderr, "Decode fail: err %d %m", errno);
		return false;
	}
	fwrite(in->data, in->data_size, 1, f);
	fclose(f);
#endif

	LOGD("Success JPEG decode of %zu bytes", in->data_size);
	return true;

fail:
	LOGD("Failed JPEG decode of %zu bytes", in->data_size);
	if (*out != NULL) {
		ohmd_video_frame_release(*out);
		*out = NULL;
	}
	jpeg_abort_decompress (&jd->cinfo);
	return false;
}

static void
ohmd_jpeg_release_frame(ohmd_video_frame *frame, ohmd_jpeg_decoder *jd)
{
	assert(frame->owner == jd);

	/* Put the frame back on the free queue */
	ohmd_lock_mutex(jd->frames_lock);
	assert(jd->n_free_frames < jd->n_alloced_frames);
	jd->free_frames[jd->n_free_frames++] = frame;
	ohmd_unlock_mutex(jd->frames_lock);
}
