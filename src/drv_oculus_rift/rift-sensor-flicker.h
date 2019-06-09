/*
 * LED pattern detection and identification
 * Copyright 2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#ifndef __FLICKER_H__
#define __FLICKER_H__

#include <stdbool.h>
#include <stdint.h>

#include "rift.h"
#include "rift-sensor-blobwatch.h"

void rift_sensor_flicker_process(struct blob *blobs, int num_blobs,
		     uint8_t led_pattern_phase, rift_led *leds, uint8_t num_leds);

#endif /* __BLOBWATCH_H__*/
