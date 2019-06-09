/*
 * LED pattern detection and identification
 * Copyright 2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "rift-sensor-flicker.h"
//#include "debug.h"

#include <stdio.h>

static int hamming_distance(uint16_t a, uint16_t b)
{
	uint16_t tmp = a ^ b;
	int distance = 0;
	int bit;

	for (bit = 1 << 9; bit; bit >>= 1)
		if (tmp & bit)
			distance++;

	return distance;
}

static int pattern_find_id(rift_led *leds, int num_patterns,
			   uint16_t pattern, int8_t *id)
{
	int i;

	for (i = 0; i < num_patterns; i++) {
		if (pattern == leds[i].pattern) {
			*id = i;
			return 2;
		}
		if (hamming_distance(pattern, leds[i].pattern) < 2) {
			*id = i;
			return 1;
		}
	}

	return -2;
}

/*
 * Records blob blinking patterns and compares against the blinking patterns
 * stored in the Rift DK2 to determine the corresponding LED IDs.
 */
void rift_sensor_flicker_process(struct blob *blobs, int num_blobs,
		     uint8_t led_pattern_phase, rift_led *leds, uint8_t num_leds)
{
	struct blob *b;
	int success = 0;
	int phase = (led_pattern_phase + 1) % 10;

	for (b = blobs; b < blobs + num_blobs; b++) {
		uint16_t pattern;

		/* Update pattern only if blob was observed previously */
		if (b->age < 1)
			continue;

		/*
		 * Interpret brightness change of more than 10% as rising
		 * or falling edge. Right shift the pattern and add the
		 * new brightness level as MSB.
		 */
		pattern = (b->pattern >> 1) & 0x1ff;
		if (b->area * 10 > b->last_area * 11)
			pattern |= (1 << 9);
		else if (b->area * 11 < b->last_area * 10)
			pattern |= (0 << 9);
		else
			pattern |= b->pattern & (1 << 9);
		b->pattern = pattern;

		/*
		 * Determine LED ID only if a full pattern was recorded and
		 * consensus about the blinking phase is established
		 */
		if (b->age < 9 || phase < 0)
			continue;

		/* Rotate the pattern bits according to the phase */
		pattern = ((pattern >> (10 - phase)) | (pattern << phase)) &
			  0x3ff;

		success += pattern_find_id(leds, num_leds, pattern,
					   &b->led_id);
	}
}
