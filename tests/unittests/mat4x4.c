/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2020 Jan Schmidt
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

/* Unit Tests - Mat4x4 Tests */

#include "tests.h"

bool mat4x4f_eq(const mat4x4f m1, const mat4x4f m2, float t)
{
	for(int y = 0; y < 4; y++) {
		for(int x = 0; x < 4; x++) {
			if(!float_eq(m1.m[y][x], m2.m[y][x], t)){
				printf("\nmat4x4[%d][%d] == %f, expected %f\n", y, x, m1.m[y][x], m2.m[y][x]);
				return false;
			}
		}
	}

	return true;
}
