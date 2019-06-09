/*
 * Math helpers
 * Copyright 2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#ifndef __MATHS_H__
#define __MATHS_H__

#include <math.h>
#include <float.h>
#include <stdint.h>

typedef struct {
	float x, y, z;
} vec3;

typedef struct {
	double x, y, z;
} dvec3;

typedef struct {
	double x, y, z, w;
} dquat;

typedef struct {
	double m[9];
} dmat3;

float f16_to_float(uint16_t f16);

static inline double vec3_dot(const vec3 *a, const vec3 *b)
{
	return a->x * b->x + a->y * b->y + a->z * b->z;
}

static inline double vec3_norm(const vec3 *v)
{
	return sqrt(vec3_dot(v, v));
}

static inline void vec3_normalize(vec3 *v)
{
	const float inv_norm = 1.0f / vec3_norm(v);

	v->x *= inv_norm;
	v->y *= inv_norm;
	v->z *= inv_norm;
}

static inline void vec3_cross(vec3 *c, const vec3 *a, const vec3 *b)
{
	c->x = a->y * b->z - b->y * a->z;
	c->y = a->z * b->x - b->z * a->x;
	c->z = a->x * b->y - b->x * a->y;
}

static inline double dquat_dot(const dquat *q, const dquat *p)
{
	return q->w * p->w + q->x * p->x + q->y * p->y + q->z * p->z;
}

static inline double dquat_norm(const dquat *q)
{
	return sqrt(dquat_dot(q, q));
}

static inline void dquat_normalize(dquat *q)
{
	const double inv_norm = 1.0 / dquat_norm(q);

	q->w *= inv_norm;
	q->x *= inv_norm;
	q->y *= inv_norm;
	q->z *= inv_norm;
}

static inline void dquat_mult(dquat *r, dquat *p, const dquat *q)
{
	r->w = p->w * q->w - p->x * q->x - p->y * q->y - p->z * q->z;
	r->x = p->w * q->x + p->x * q->w + p->y * q->z - p->z * q->y;
	r->y = p->w * q->y + p->y * q->w + p->z * q->x - p->x * q->z;
	r->z = p->w * q->z + p->z * q->w + p->x * q->y - p->y * q->x;
}

void dquat_from_axis_angle(dquat *quat, const dvec3 *axis, double angle);
void dquat_from_axes(dquat *q, const vec3 *a, const vec3 *b);
void dquat_from_gyro(dquat *q, const vec3 *gyro, double dt);

#endif /* __MATHS_H__ */
