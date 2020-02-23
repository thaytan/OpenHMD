/*
 * Quadratic root finder
 * Copyright 2019 Philipp Zabel
 * SPDX-License-Identifier: (GPL-2.0-or-later or BSL-1.0)
 */
#ifndef EIG3X3KNOWN0
#define EIG3X3KNOWN0

#include <math.h>
#include "quadratic.h"

/*
 * This implements algorithm 2 "eig3x3known0" described in the paper "Lambda
 * Twist: An Accurate Fast Robust Perspective Three Point (P3P) Solver." by
 * Mikael Persson and Klas Nordberg. See the paper for details.
 *
 * 1: function GETEIGVECTOR(m, r)
 * this is open coded below, since it is called twice, and intermediate
 * values are reused
 *
 * 7: function EIG3X3KNOWN0(M)
 * this function does not write E[2,5,8]
 */
static inline void eig3x3known0(const double *M, double *E, double *sigma)
{
	/* 8: b₃ = M(:,2) x M(:,3); */
	/* 9: b₃ = b₃ / |b₃| */
	/* we skip steps 8 and 9 */

	/*
	 * 10: m = M(:)
	 *
	 * we are passed just the upper triangular part of symmetric matrix M:
	 *
	 *       ⎛ m₁ m₂ m₃ ⎞
	 *   M = ⎜    m₅ m₆ ⎟
	 *       ⎝       m₈ ⎠
	 */
	const double m1 = M[0];
	const double m2 = M[1];
	const double m3 = M[2];
	const double m5 = M[3];
	const double m6 = M[4];
	const double m9 = M[5];

	/* reusable intermediate values */
	const double m2_sq = m2 * m2;
	const double m1_plus_m5 = m1 + m5;

	/* 11: p₁ = m₁ - m₅ - m₉ */
	/*         --            */
	const double p1 = -m1_plus_m5 - m9;

	/* 12: p₀ = -m₂² - m₃² - m₆² + m₁m₅ + m₉ + m₅m₉ */
	/*                             ---------        */
	const double p0 = -m2_sq - m3*m3 - m6*m6 + m1 * (m5 + m9) + m5 * m9;

	/* 13: [σ₁, σ₂] as the roots of σ² + p₁σ + p₀ = 0 */
	double sigma1 = 0.0;
	double sigma2 = 0.0;
	reduced_quadratic_real_roots(p1, p0, &sigma1, &sigma2);

	if (fabs(sigma1) > fabs(sigma2)) {
		sigma[0] = sigma1;
		sigma[1] = sigma2;
	} else {
		sigma[0] = sigma2;
		sigma[1] = sigma1;
	}

	const double r1 = sigma[0];
	const double r2 = sigma[1];

	/*
	 * 14: b₁ = GETEIGVECTOR(m, σ₁)
	 * 15: b₂ = GETEIGVECTOR(m, σ₂)
	 * 16: if |σ₁| > |σ₂| then
	 * 17:   Return ([b1, b2, b3], σ₁, σ₂)
	 * 18: else
	 * 19:   Return ([b2, b1, b3], σ₂, σ₁)
	 */

	/* reusable intermediate values */
	const double m1m5_minus_m2sq = m1 * m5 - m2_sq;
	const double m2m6_minus_m3m5 = m2 * m6 - m3 * m5;
	const double m2m3_minus_m1m6 = m2 * m3 - m1 * m6;

	/* 1: function GETEIGVECTOR(m, r) */
	/* 2: c = (r² + m₁m₅ - r(m₁ + m₅) - m₂²) */
	const double tmp1 = 1.0 / (r1 * (r1 - m1_plus_m5) + m1m5_minus_m2sq);
	const double tmp2 = 1.0 / (r2 * (r2 - m1_plus_m5) + m1m5_minus_m2sq);
	/* 3: a₁ = (rm₃ + m₂m₆ - m₃m₅) / c */
	/* 4: a₂ = (rm₆ + m₂m₃ - m₁m₆) / c */
	double a11 = (r1 * m3 + m2m6_minus_m3m5) * tmp1;
	double a12 = (r1 * m6 + m2m3_minus_m1m6) * tmp1;
	double a21 = (r2 * m3 + m2m6_minus_m3m5) * tmp2;
	double a22 = (r2 * m6 + m2m3_minus_m1m6) * tmp2;
	/* 5: v = (a₁ a₂ 1) */
	/* 6: Return v / |v| */
	const double rnorm1 = 1.0 / sqrt(a11 * a11 + a12 * a12 + 1.0);
	const double rnorm2 = 1.0 / sqrt(a21 * a21 + a22 * a22 + 1.0);

	E[0] = a11 * rnorm1;
	E[3] = a12 * rnorm1;
	E[6] = rnorm1;

	E[1] = a21 * rnorm2;
	E[4] = a22 * rnorm2;
	E[7] = rnorm2;
}

#endif /* EIG3X3KNOWN0 */
