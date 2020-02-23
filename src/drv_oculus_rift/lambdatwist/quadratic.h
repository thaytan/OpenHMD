/*
 * Quadratic root finder
 * Copyright 2019 Philipp Zabel
 * SPDX-License-Identifier: (GPL-2.0-or-later or BSL-1.0)
 */
#ifndef QUADRATIC_H
#define QUADRATIC_H

#include <math.h>
#include <stdbool.h>

/*
 * Store the real roots of the reduced quadratic equation x² + bx + c = 0 in
 * r1, r2. Returns true if roots are found, false otherwise.
 */
static inline bool reduced_quadratic_real_roots(double b, double c,
						double *r1, double *r2)
{
	const double half_b = 0.5 * b;
	const double quarter_v = half_b * half_b - c;

	if (quarter_v < 0)
		return false;

	const double half_y = sqrt(quarter_v);

	*r1 = -half_b + half_y;
	*r2 = -half_b - half_y;

	return true;
}

/*
 * Store the real roots of the quadratic equation ax² + bx + c = 0 in
 * r1, r2. Returns true if roots are found, false otherwise.
 */
static inline bool quadratic_real_roots(double a, double b, double c,
					double *r1, double *r2)
{
	const double a_inv = 1.0 / a;

	return reduced_quadratic_real_roots(b * a_inv, c * a_inv, r1, r2);
}

#endif /* QUADRATIC_H */
