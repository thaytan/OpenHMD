/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2020 Jan Schmidt
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

/* Unit Tests - Pose Tests */

#include "tests.h"

/* Test pose 1, at X,Y = 10,10 facing "east" along the X vector
 * by rotation 90deg around the Z axis. The rotation is negative,
 * because the test is using RHS coordinates */
static const vec3f test_position1 = {{ 10.0, 10.0, 0.0 }};
static const vec3f test_orient_vec1 = {{ 0.0, 0.0, 1.0 }};
static const float test_orient_90deg = -M_PI/2;

static const vec3f test_position2 = {{ -150.984375, -371.769531, 326.843750 }};
static const quatf test_orient2 = {{ 0.833051, 0.238250, 0.341881, -0.363842 }};

const float epsilon = 0.001;

#define DEBUG_TEST 0

#if DEBUG_TEST
static void
dump_pose(const char *prefix, posef *p) {
	printf("%s: ", prefix);
	printf(" pos %f %f %f, ", p->pos.x, p->pos.y, p->pos.z);
	printf(" orient %f %f %f %f\n", p->orient.x, p->orient.y, p->orient.z, p->orient.w);
}
#else
#define dump_pose(a,b)
#endif

void test_oposef_init()
{
	posef p;

	oposef_init(&p, &test_position2, &test_orient2);

	TAssert(vec3f_eq(test_position2, p.pos, epsilon));
	TAssert(quatf_eq(test_orient2, p.orient, epsilon));

	vec3f pos1 = { 0, };
	quatf q1;

	ovec3f_set(&pos1, 10, 10, 10);
	oquatf_init_axis(&q1, &test_orient_vec1, test_orient_90deg);
	oposef_init(&p, &pos1, &q1);

  const mat4x4f expected = { {
    {  0.0, 1.0, 0.0, 10.0 },
    { -1.0, 0.0, 0.0, 10.0 },
    {  0.0, 0.0, 1.0, 10.0 },
    {  0.0, 0.0, 0.0,  1.0 },
  } };
  mat4x4f result;

  oposef_get_mat4x4(&p, result.m);
	TAssert(mat4x4f_eq(result, expected, epsilon));
}

void test_oposef_inverse()
{
	vec3f pos1 = { 0, };
	quatf q1;
	posef origin, test_pose;
	posef p1, p2;

	/* Unrotated origin pose for testing against */
	ovec3f_set(&pos1, 0, 0, 0);
	oquatf_init_axis(&q1, &test_orient_vec1, 0);
	oposef_init(&origin, &pos1, &q1);

	/* Test that pose inversion yields a new pose that gives
	 * the location and orientation of the origin
	 * point, as seen from the given pose */
	oquatf_init_axis(&q1, &test_orient_vec1, test_orient_90deg);
	oposef_init(&test_pose, &test_position1, &q1);

	p1 = test_pose;
	oposef_inverse(&p1);

	/* The original origin is now behind us at -10,-10, and
	 * then rotated clockwise by 90 deg around the (RH) Z axis,
	 * to 10,-10 */
	oquatf_init_axis(&q1, &test_orient_vec1, -test_orient_90deg);
	ovec3f_set(&pos1, 10.0, -10.0, 0.0);

	TAssert(vec3f_eq(p1.pos, pos1, epsilon));
	TAssert(quatf_eq(p1.orient, q1, epsilon));

	/* Inverting a pose should be equivalent to moving the unrotated origin point into
	 * the local frame of the test pose */
	oposef_apply_inverse(&origin, &test_pose, &p2);

	TAssert(vec3f_eq(p1.pos, p2.pos, epsilon));
	TAssert(quatf_eq(p1.orient, p2.orient, epsilon));

	/* Test that double inversion restores the original pose */
	oposef_init(&p1, &test_position2, &test_orient2);
	p2 = p1;
	oposef_inverse(&p2);
	oposef_inverse(&p2);
	TAssert(vec3f_eq(p1.pos, p2.pos, epsilon));
	TAssert(quatf_eq(p1.orient, p2.orient, epsilon));
}

void test_oposef_apply()
{
	vec3f pos1 = { 0, };
	quatf q1;
	posef test_pose;
	posef p1, p2, expected;

	/* Moving a pose into its own frame should
	 * always land at the origin - 0 delta and
	 * 0 rotation difference */
	ovec3f_set(&pos1, 10, 10, 10);
	oquatf_init_axis(&q1, &test_orient_vec1, test_orient_90deg);
	oposef_init(&test_pose, &pos1, &q1);

	oposef_apply_inverse(&test_pose, &test_pose, &p1);

	ovec3f_set(&pos1, 0, 0, 0);
	oquatf_init_axis(&q1, &test_orient_vec1, 0);
	oposef_init(&expected, &pos1, &q1);

	TAssert(vec3f_eq(p1.pos, expected.pos, epsilon));
	TAssert(quatf_eq(p1.orient, expected.orient, epsilon));

	/* Moving from an unrotated frame with an offset to
	 * the global frame just offsets the position, but
	 * not the orientation */
	ovec3f_set(&pos1, 10, 10, 10);
	oquatf_init_axis(&q1, &test_orient_vec1, 0);
	oposef_init(&test_pose, &pos1, &q1);

	ovec3f_set(&pos1, 10, 10, 10);
	oquatf_init_axis(&q1, &test_orient_vec1, test_orient_90deg);
	oposef_init(&p1, &pos1, &q1);

	oposef_apply(&p1, &test_pose, &p2);

	/* Expected result */
	ovec3f_set(&pos1, 20, 20, 20);
	oquatf_init_axis(&q1, &test_orient_vec1, test_orient_90deg);
	oposef_init(&expected, &pos1, &q1);

	TAssert(vec3f_eq(expected.pos, p2.pos, epsilon));
	TAssert(quatf_eq(expected.orient, p2.orient, epsilon));

	/* Moving a pose that's at -sqrt(200),0, rotated -45deg,
	 * from a reference that's at 10,10 rotated 45 deg
	 * should result in a pose that's on the Y axis
	 * at 0,20 and unrotated */
	ovec3f_set(&pos1, -sqrtf(200.0), 0, 0);
	oquatf_init_axis(&q1, &test_orient_vec1, -test_orient_90deg/2);
	oposef_init(&test_pose, &pos1, &q1);

	ovec3f_set(&pos1, 10, 10, 0);
	oquatf_init_axis(&q1, &test_orient_vec1, test_orient_90deg/2);
	oposef_init(&p1, &pos1, &q1);

	oposef_apply(&test_pose, &p1, &p2);

	/* Expected result */
	ovec3f_set(&pos1, 0, 20, 0);
	oquatf_init_axis(&q1, &test_orient_vec1, 0);
	oposef_init(&expected, &pos1, &q1);

	TAssert(vec3f_eq(expected.pos, p2.pos, epsilon));
	TAssert(quatf_eq(expected.orient, p2.orient, epsilon));

	/* Moving back to the reference frame should return the original
	 * relative pose */
	oposef_apply_inverse(&expected, &p1, &p2);

	TAssert(vec3f_eq(test_pose.pos, p2.pos, epsilon));
	TAssert(quatf_eq(test_pose.orient, p2.orient, epsilon));
}
