/*
 * Math helpers
 * Copyright 2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#include <math.h>

#include "rift-sensor-maths.h"

float f16_to_float(uint16_t f16)
{
	unsigned int sign = f16 >> 15;
	unsigned int exponent = (f16 >> 10) & 0x1f;
	unsigned int mantissa = f16 & 0x3ff;
	union {
		float f32;
		uint32_t u32;
	} u;

	if (exponent == 0) {
		if (!mantissa) {
			/* zero */
			u.u32 = sign << 31;
		} else {
			/* subnormal */
			exponent = 127 - 14;
			mantissa <<= 23 - 10;
			/*
			 * convert to normal representation:
			 * shift up mantissa and drop MSB
			 */
			while (!(mantissa & (1 << 23))) {
				mantissa <<= 1;
				exponent--;
			}
			mantissa &= 0x7fffffu;
			u.u32 = (sign << 31) | (exponent << 23) | mantissa;
		}
	} else if (exponent < 31) {
		/* normal */
		exponent += 127 - 15;
		mantissa <<= 23 - 10;
		u.u32 = (sign << 31) | (exponent << 23) | mantissa;
	} else if (mantissa == 0) {
		/* infinite */
		u.u32 = (sign << 31) | (255 << 23);
	} else {
		/* NaN */
		u.u32 = 0x7fffffffu;
	}
	return u.f32;
}

/*
 * Returns the rotation around the normalized vector axis, about the given
 * angle in quaternion q.
 */
void dquat_from_axis_angle(dquat *q, const dvec3 *axis, double angle)
{
	const double half_angle = angle * 0.5;
	const double sin_half_angle = sin(half_angle);

	q->w = cos(half_angle);
	q->x = sin_half_angle * axis->x;
	q->y = sin_half_angle * axis->y;
	q->z = sin_half_angle * axis->z;
}

/*
 * Returns the rotation along the shortest arc from normalized vector a to
 * normalized vector b in quaternion q.
 */
void dquat_from_axes(dquat *q, const vec3 *a, const vec3 *b)
{
	vec3 w;

	vec3_cross(&w, a, b);

	q->w = 1.0 + vec3_dot(a, b);
	q->x = w.x;
	q->y = w.y;
	q->z = w.z;

	dquat_normalize(q);
}

/*
 * Returns the rotation for a gyro reading after timestep dt in quaternion q.
 * This is an approximation of
 *	q->w = cos(x) * cos(y) * cos(z) + sin(x) * sin(y) * sin(z);
 *	q->x = sin(x) * cos(y) * cos(z) - cos(x) * sin(y) * sin(z);
 *	q->y = cos(x) * sin(y) * cos(z) + sin(x) * cos(y) * sin(z);
 *	q->z = cos(x) * cos(y) * sin(z) - sin(x) * sin(y) * cos(z);
 * for small time steps, where the half angles x, y, z are assumed to be small.
 */
void dquat_from_gyro(dquat *q, const vec3 *gyro, double dt)
{
	const double scale = 0.5 * dt;
	const double x = gyro->x * scale;
	const double y = gyro->y * scale;
	const double z = gyro->z * scale;

	q->w = 1.0 + x * y * z;
	q->x = x - y * z;
	q->y = y + x * z;
	q->z = z - x * y;
}
