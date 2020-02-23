/*
 * Gauss-Newton refinement of Λ
 * Copyright 2019 Philipp Zabel
 * SPDX-License-Identifier: (GPL-2.0-or-later or BSL-1.0)
 */
#ifndef REFINE_LAMBDA
#define REFINE_LAMBDA

#include <math.h>

/*
 * The number of iterations is chosen to just match the precision of the
 * original Lambda Twist implementation.
 * Increasing this further, we'd reach the point of diminishing returns.
 */
#define REFINE_LAMBDA_ITERATIONS 5

/*
 * Section 3.8 "Implementation Details" states:
 *
 *   "Specifically, we refine the solution using a few verified steps of
 *    Gauss-Newton optimization on the sum of squares of Eqn (4)."
 *
 * Equation 4 in the paper:
 *
 *   Λ^T M₁₂ Λ = a₁₂,   Λ^T M₁₃ Λ = a₁₃,   Λ^T M₂₃ Λ = a₂₃   (4)
 *
 * where
 *
 *         ⎛ 1  -b₁₂ 0 ⎞        ⎛ 1   0 -b₁₃⎞        ⎛ 0   0   0 ⎞
 *   M₁₂ = ⎜-b₁₂ 1   0 ⎟, M₁₃ = ⎜ 0   0   0 ⎟, M₂₃ = ⎜ 0   1 -b₂₃⎟.
 *         ⎝ 0   0   0 ⎠        ⎝-b₁₃ 0   1 ⎠        ⎝ 0 -b₂₃  1 ⎠
 *
 * thus refine Λ towards:
 *
 *   λ₁² - 2b₁₂λ₁λ₂ + λ₂² - a₁₂ = 0
 *   λ₁² - 2b₁₃λ₁λ₃ + λ₃² - a₁₃ = 0
 *   λ₂² - 2b₂₃λ₂λ₃ + λ₃² - a₂₃ = 0
 *
 */
static inline void gauss_newton_refineL(double *L,
					double a12, double a13, double a23,
					double b12, double b13, double b23)
{
	double l1 = L[0];
	double l2 = L[1];
	double l3 = L[2];

	for (int i = 0; i < REFINE_LAMBDA_ITERATIONS; i++) {
		/* λᵢ² */
		const double l1_sq = l1 * l1;
		const double l2_sq = l2 * l2;
		const double l3_sq = l3 * l3;

		/* residuals r(Λ) = λᵢ² - 2bᵢⱼλᵢλⱼ + λⱼ² - aᵢⱼ */
		const double r1 = l1_sq - 2.0 * b12 * l1 * l2 + l2_sq - a12;
		const double r2 = l1_sq - 2.0 * b13 * l1 * l3 + l3_sq - a13;
		const double r3 = l2_sq - 2.0 * b23 * l2 * l3 + l3_sq - a23;

		/* bail early if the solution is good enough */
		if (fabs(r1) + fabs(r2) + fabs(r3) < 1e-10)
			break;

		/*
		 *                   ⎛ λ₁-b₁₂λ₂  λ₂-b₁₂λ₁  0       ⎞
		 * Jᵢⱼ = ∂rᵢ/∂λⱼ = 2 ⎜ λ₁-b₁₃λ₃  0         λ₃-b₁₃λ₁⎟
		 *                   ⎝ 0         λ₂-b₂₃λ₃  λ₃-b₂₃λ₂⎠
		 */
		const double j0 = l1 - b12 * l2; /* ½∂r₁/∂λ₁ */
		const double j1 = l2 - b12 * l1; /* ½∂r₁/∂λ₂ */
		const double j3 = l1 - b13 * l3; /* ½∂r₂/∂λ₁ */
		const double j5 = l3 - b13 * l1; /* ½∂r₂/∂λ₃ */
		const double j7 = l2 - b23 * l3; /* ½∂r₃/∂λ₂ */
		const double j8 = l3 - b23 * l2; /* ½∂r₃/∂λ₃ */

		/* 4 / det(J) */
		const double det = 0.5 / (-j0 * j5 * j7 - j1 * j3 * j8);

		/* Λ = Λ - J⁻¹ * r(Λ) */
		l1 -= det * (-j5 * j7 * r1 - j1 * j8 * r2 + j1 * j5 * r3);
		l2 -= det * (-j3 * j8 * r1 + j0 * j8 * r2 - j0 * j5 * r3);
		l3 -= det * ( j3 * j7 * r1 - j0 * j7 * r2 - j1 * j3 * r3);
	}

	L[0] = l1;
	L[1] = l2;
	L[2] = l3;
}

#endif /* REFINE_LAMBDA */
