/*
 * Vector subtraction and normalization
 * Copyright 2019 Philipp Zabel
 * SPDX-License-Identifier: (GPL-2.0-or-later or BSL-1.0)
 */
#ifndef VEC_H
#define VEC_H

#include <math.h>

static inline void vec3_normalise(double *v)
{
	double rnorm = 1.0 / sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	v[0] *= rnorm;
	v[1] *= rnorm;
	v[2] *= rnorm;
}

static inline void vec3_sub(double *ret, const double *a, const double *b)
{
	ret[0] = a[0] - b[0];
	ret[1] = a[1] - b[1];
	ret[2] = a[2] - b[2];
}

#endif /* VEC_H */
