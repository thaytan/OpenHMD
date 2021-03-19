/*
 * LED pattern detection and identification
 * Copyright 2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "rift-sensor-flicker.h"
//#include "debug.h"

#include <stdio.h>

#define HAMMING_DISTANCE 0

#if HAMMING_DISTANCE
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
#endif

static int pattern_find_id(rift_led *leds, int num_patterns,
			   uint16_t pattern, int *id)
{
	int i;

	for (i = 0; i < num_patterns; i++) {
		if (pattern == leds[i].pattern) {
			*id = i;
			return 2;
		}

#if HAMMING_DISTANCE
		if (hamming_distance(pattern, leds[i].pattern) < 2) {
			*id = i;
			return 1;
		}
#endif
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
	uint8_t phase = led_pattern_phase % 10;
	int thresh = 0;

	for (b = blobs; b < blobs + num_blobs; b++) {
		uint16_t pattern = 0;
		uint16_t min_bit, max_bit;

		/* Update pattern only if blob was observed previously */
		if (b->age < 1 || b->pattern_prev_phase == -1) {
			b->pattern_prev_phase = phase;
			continue;
		}

		if (((phase - b->pattern_prev_phase) % 10) > 2) {
			/* We can miss one frame and still guess the
			 * correct transition based on a brightness change,
			 * but if we missed 2 we might guess wrong, so don't */
			b->pattern_prev_phase = phase;
			continue;
		}


		
		b->pattern_bits[phase] = b->area;
		

		

		b->pattern_prev_phase = phase;

		/*
		 * Determine LED ID only if a full pattern was recorded and
		 * consensus about the blinking phase is established
		 */
		if (b->age < 10)
			continue;
		
		min_bit = 65535;
		max_bit = 0;
		for (int i = 0; i < 10; i++) {
			if (b->pattern_bits[i] > max_bit)
				max_bit = b->pattern_bits[i];
			if (b->pattern_bits[i] < min_bit)
				min_bit = b->pattern_bits[i];
		}
		thresh = min_bit + (max_bit - min_bit)/2;
		for (int i = 0; i < 10; i++) {
			if (b->pattern_bits[i] > thresh)
				pattern |= (1 << i);
		}

		if (min_bit < 5)
			continue;

#if 0
		printf ("blob %d age %d pattern %x phase %d bits %d %d %d %d %d %d %d %d %d %d\n",
			(int)((b-blobs)), b->age, pattern, phase,
			b->pattern_bits[0], b->pattern_bits[1], b->pattern_bits[2],
			b->pattern_bits[3], b->pattern_bits[4], b->pattern_bits[5],
			b->pattern_bits[6], b->pattern_bits[7], b->pattern_bits[8],
			b->pattern_bits[9]);
#endif
		if (b->pattern == pattern)
			b->pattern_age++;
		else
			b->pattern_age = 0;

		/* If the blob already has an LED id, require at least 3 repetitions of a new pattern
		 * before changing the ID */
		if (b->led_id == -1 || b->pattern_age > 30) {
		  int led_id = b->led_id;
		  b->pattern = pattern;
		  if (pattern == 0)
				  b->led_id = -1;
		  else {
				  success += pattern_find_id(leds, num_leds, pattern,
					     &b->led_id);
		  }
		  if (b->led_id != led_id) {
		  	printf("Changing led %d from %d to %d pattern 0x%x at age %d\n", b->track_index, led_id, b->led_id, pattern, b->age);
		  }
		}
	}
}
