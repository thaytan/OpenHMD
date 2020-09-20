/*
 * Blob detection
 * Copyright 2014-2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#ifndef __BLOBWATCH_H__
#define __BLOBWATCH_H__

#include <stdbool.h>
#include <stdint.h>

#include "rift.h"

#define MAX_BLOBS_PER_FRAME  100
#define LED_INVALID_ID -1
#define LED_NOISE_ID -2
#define LED_LOCAL_ID(l) (((l) < 0) ? (l) : (l) & 0xFF)
#define LED_OBJECT_ID(l) (((l) < 0) ? (l) : (l) >> 8)
#define LED_MAKE_ID(o,n) ((o) << 8 | (n))

/* 0x24 works much better for Rift CV1, but the threshold needs
 * to be higher for DK2 which has more background bleed and bigger
 * tracking LEDs */
#define BLOB_THRESHOLD_CV1  0x24
#define BLOB_THRESHOLD_DK2  0x7f

struct blob {
	/* center of bounding box */
	float x;
	float y;
	float vx;
	float vy;
	/* bounding box */
	uint16_t width;
	uint16_t height;
	uint32_t area;
	uint32_t age;
	int16_t track_index;
	uint16_t pattern;

	uint32_t pattern_age;
	uint16_t pattern_bits[10];
	int8_t pattern_prev_phase;
	int led_id;
	int prev_led_id;
};

/*
 * Stores all blobs observed in a single frame.
 */
struct blobservation {
	int num_blobs;
	struct blob blobs[MAX_BLOBS_PER_FRAME];
	int tracked_blobs;
	uint8_t tracked[MAX_BLOBS_PER_FRAME];
};

struct blobwatch;

struct blobwatch *blobwatch_new(uint8_t threshold, int width, int height);
void blobwatch_free (struct blobwatch *bw);
void blobwatch_process(struct blobwatch *bw, uint8_t *frame,
		       int width, int height, uint8_t led_pattern_phase,
		       rift_led *leds, uint8_t num_leds,
		       struct blobservation **output);
struct blob *blobwatch_find_blob_at(struct blobwatch *bw, int x, int y);
void blobwatch_set_flicker(struct blobwatch *bw, bool enable);

#endif /* __BLOBWATCH_H__*/
