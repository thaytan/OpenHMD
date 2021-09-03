// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Math Code Implementation */

#include <assert.h>
#include <float.h>
#include <string.h>
#include <stdbool.h>
#include "omath.h"

// vector
void ovec3f_set(vec3f* me, float x, float y, float z)
{
	me->x = x;
	me->y = y;
	me->z = z;
}

static float ovec3f_get_sqlength(const vec3f* me)
{
	return POW2(me->x) + POW2(me->y) + POW2(me->z);
}

float ovec3f_get_length(const vec3f* me)
{
	return sqrtf(POW2(me->x) + POW2(me->y) + POW2(me->z));
}

void ovec3f_normalize_me(vec3f* me)
{
	if(me->x == 0 && me->y == 0 && me->z == 0)
		return;

	float len = ovec3f_get_length(me);
	me->x /= len;
	me->y /= len;
	me->z /= len;
}

void ovec3f_cross(const vec3f* a, const vec3f* b, vec3f *out)
{
	out->x = a->y * b->z - a->z * b->y;
	out->y = a->z * b->x - a->x * b->z;
	out->z = a->x * b->y - a->y * b->x;
}

void ovec3f_subtract(const vec3f* a, const vec3f* b, vec3f* out)
{
	for(int i = 0; i < 3; i++)
		out->arr[i] = a->arr[i] - b->arr[i];
}

void ovec3f_add(const vec3f* a, const vec3f* b, vec3f* out)
{
	for(int i = 0; i < 3; i++)
		out->arr[i] = a->arr[i] + b->arr[i];
}

void ovec3f_inverse(vec3f *me)
{
	for(int i = 0; i < 3; i++)
		me->arr[i] = -me->arr[i];
}

float ovec3f_get_dot(const vec3f* me, const vec3f* vec)
{
	return me->x * vec->x + me->y * vec->y + me->z * vec->z;
}

float ovec3f_get_angle(const vec3f* me, const vec3f* vec)
{
	float dot = ovec3f_get_dot(me, vec);
	float lengths = ovec3f_get_length(me) * ovec3f_get_length(vec);

	if(lengths == 0)
		return 0;

	dot /= lengths;

	if (dot >= 1.0 || dot <= -1.0)
		return 0.0;

	return acosf(dot);
}

void ovec3f_multiply_scalar (const vec3f* a, const float s, vec3f* out)
{
	out->x = a->x * s;
	out->y = a->y * s;
	out->z = a->z * s;
}

void ovec3f_multiply_mat3x3(const vec3f *a, const float m[3][3], vec3f *out)
{
	out->x = m[0][0] * a->x + m[0][1] * a->y + m[0][2] * a->z;
	out->y = m[1][0] * a->x + m[1][1] * a->y + m[1][2] * a->z;
	out->z = m[2][0] * a->x + m[2][1] * a->y + m[2][2] * a->z;
}

// quaternion

void oquatf_init_axis(quatf* me, const vec3f* vec, float angle)
{
	vec3f norm = *vec;
	ovec3f_normalize_me(&norm);

	me->x = norm.x * sinf(angle / 2.0f);
	me->y = norm.y * sinf(angle / 2.0f);
	me->z = norm.z * sinf(angle / 2.0f);
	me->w = cosf(angle / 2.0f);
}

void oquatf_set(quatf* me, float x, float y, float z, float w)
{
	me->x = x;
	me->y = y;
	me->z = z;
	me->w = w;
}

/* Calculate a quaternion that moves vec 'from' to align along 'to' */
void oquatf_from_vectors(quatf* me, const vec3f* from, const vec3f* to)
{
	vec3f axis;
	float dot;

	ovec3f_cross(from, to, &axis);
	dot = ovec3f_get_dot(from, to);

	if (ovec3f_get_sqlength(&axis) < 1e-7) {
		if (dot < 0.0) {
			/* 180 degree rotation, pick an arbirtrary rotation
			 * axis at 90 degrees to the vector */
			me->x = from->x;
			me->y = -from->z;
			me->z = from->y;
			me->w = 0.0;
			oquatf_normalize_me(me);
		}
		else {
			/* Identity rotation */
			me->x = me->y = me->z = 0.0;
			me->w = 1.0;
		}
	} else {
		me->x = axis.x;
		me->y = axis.y;
		me->z = axis.z;
		me->w = dot + sqrtf(ovec3f_get_sqlength(from) * ovec3f_get_sqlength(to));

		oquatf_normalize_me(me);
	}
}

void oquatf_get_rotated(const quatf* me, const vec3f* vec, vec3f* out_vec)
{
	quatf q = {{vec->x * me->w + vec->z * me->y - vec->y * me->z,
	            vec->y * me->w + vec->x * me->z - vec->z * me->x,
	            vec->z * me->w + vec->y * me->x - vec->x * me->y,
	            vec->x * me->x + vec->y * me->y + vec->z * me->z}};

	out_vec->x = me->w * q.x + me->x * q.w + me->y * q.z - me->z * q.y;
	out_vec->y = me->w * q.y + me->y * q.w + me->z * q.x - me->x * q.z;
	out_vec->z = me->w * q.z + me->z * q.w + me->x * q.y - me->y * q.x;
}

void oquatf_get_rotated_abs(const quatf* me, const vec3f* vec, vec3f* out_vec)
{
	quatf q = {{vec->x * me->w + vec->z * me->y - vec->y * me->z,
	            vec->y * me->w + vec->x * me->z - vec->z * me->x,
	            vec->z * me->w + vec->y * me->x - vec->x * me->y,
	            vec->x * me->x + vec->y * me->y + vec->z * me->z}};

	out_vec->x = fabs(me->w * q.x + me->x * q.w + me->y * q.z - me->z * q.y);
	out_vec->y = fabs(me->w * q.y + me->y * q.w + me->z * q.x - me->x * q.z);
	out_vec->z = fabs(me->w * q.z + me->z * q.w + me->x * q.y - me->y * q.x);
}

/* Map a rotation parameterisation to a quaternion */
void oquatf_from_rotation(quatf *me, const vec3f *rot)
{
  float theta = ovec3f_get_length(rot);
  float c_t = cosf(theta / 2.0f);
  float sinc_t;

  /* magic number for numerical stability */
  if (theta > 1e-10) {
    sinc_t = sinf(theta / 2.0f) / theta;
  }
  else {
    /* Use Taylor series expansion, as suggested by Grassia -
     * "Practical Parameterization of Rotations Using the Exponential Map"
     */
    sinc_t = 0.5 + theta * theta / 48.0f;
  }

	me->x = rot->x * sinc_t;
	me->y = rot->y * sinc_t;
	me->z = rot->z * sinc_t;
	me->w = c_t;

	oquatf_normalize_me(me);
}

/* Map a quaternion into rotation parameterisation */
void oquatf_to_rotation(const quatf *me, vec3f *rot)
{
  float theta;

  if (me->w < 0)
    theta = -2*acos(-me->w);
  else
    theta = 2*acos(me->w);

  vec3f v = {{ me->x, me->y, me->z }};
  float v_len = ovec3f_get_length(&v);

  if (v_len > 1e-10) {
    ovec3f_multiply_scalar (&v, theta / v_len, rot);
  }
  else {
    /* quaternion is approximately the identity rotation */
    ovec3f_multiply_scalar (&v, 2.0, rot);
  }

  assert(!isnan(rot->x));
  assert(!isnan(rot->y));
  assert(!isnan(rot->z));
}

void oquatf_mult(const quatf* me, const quatf* q, quatf* out_q)
{
	assert (out_q != me);
	assert (out_q != q);

	out_q->x = me->w * q->x + me->x * q->w + me->y * q->z - me->z * q->y;
	out_q->y = me->w * q->y - me->x * q->z + me->y * q->w + me->z * q->x;
	out_q->z = me->w * q->z + me->x * q->y - me->y * q->x + me->z * q->w;
	out_q->w = me->w * q->w - me->x * q->x - me->y * q->y - me->z * q->z;
}

void oquatf_mult_me(quatf* me, const quatf* q)
{
	quatf tmp = *me;
	oquatf_mult(&tmp, q, me);
}

void oquatf_normalize_me(quatf* me)
{
	float len = oquatf_get_length(me);
	me->x /= len;
	me->y /= len;
	me->z /= len;
	me->w /= len;
}

float oquatf_get_length(const quatf* me)
{
	return sqrtf(me->x * me->x + me->y * me->y + me->z * me->z + me->w * me->w);
}

float oquatf_get_dot(const quatf* me, const quatf* q)
{
	return me->x * q->x + me->y * q->y + me->z * q->z + me->w * q->w;
}

void oquatf_inverse(quatf* me)
{
	float dot = oquatf_get_dot(me, me);

	// conjugate
	for(int i = 0; i < 3; i++)
		me->arr[i] = -me->arr[i];
	
	for(int i = 0; i < 4; i++)
		me->arr[i] /= dot;
}

void oquatf_diff(const quatf* me, const quatf* q, quatf* out_q)
{
	quatf inv = *me;
	oquatf_inverse(&inv);
	oquatf_mult(&inv, q, out_q);
}

void oquatf_slerp (float fT, const quatf* rkP, const quatf* rkQ, bool shortestPath, quatf* out_q)
{
	float fCos = oquatf_get_dot(rkP, rkQ);
	quatf rkT;

	// Do we need to invert rotation?
	if (fCos < 0.0f && shortestPath)
	{
		fCos = -fCos;
		rkT = *rkQ;
		oquatf_inverse(&rkT);
	}
	else
	{
		rkT = *rkQ;
	}

	if (fabsf(fCos) < 1 - 0.001f)
	{
		// Standard case (slerp)
		float fSin = sqrtf(1 - (fCos*fCos));
		float fAngle = atan2f(fSin, fCos); 
		float fInvSin = 1.0f / fSin;
		float fCoeff0 = sin((1.0f - fT) * fAngle) * fInvSin;
		float fCoeff1 = sin(fT * fAngle) * fInvSin;
		
		out_q->x = fCoeff0 * rkP->x + fCoeff1 * rkT.x;
		out_q->y = fCoeff0 * rkP->y + fCoeff1 * rkT.y;
		out_q->z = fCoeff0 * rkP->z + fCoeff1 * rkT.z;
		out_q->w = fCoeff0 * rkP->w + fCoeff1 * rkT.w;
			
		//return fCoeff0 * rkP + fCoeff1 * rkT;
	}
	else
	{
		// There are two situations:
		// 1. "rkP" and "rkQ" are very close (fCos ~= +1), so we can do a linear
		//    interpolation safely.
		// 2. "rkP" and "rkQ" are almost inverse of each other (fCos ~= -1), there
		//    are an infinite number of possibilities interpolation. but we haven't
		//    have method to fix this case, so just use linear interpolation here.
		//Quaternion t = (1.0f - fT) * rkP + fT * rkT;
		
		out_q->x = (1.0f - fT) * rkP->x + fT * rkT.x;
		out_q->y = (1.0f - fT) * rkP->y + fT * rkT.y;
		out_q->z = (1.0f - fT) * rkP->z + fT * rkT.z;
		out_q->w = (1.0f - fT) * rkP->w + fT * rkT.w;
			
		oquatf_normalize_me(out_q);
		
		// taking the complement requires renormalisation
		//t.normalise();
		//return t;
	}
}

void oquatf_get_mat4x4(const quatf* me, const vec3f* point, float mat[4][4])
{
	mat[0][0] = 1 - 2 * me->y * me->y - 2 * me->z * me->z;
	mat[0][1] =     2 * me->x * me->y - 2 * me->w * me->z;
	mat[0][2] =     2 * me->x * me->z + 2 * me->w * me->y;
	mat[0][3] = point->x;

	mat[1][0] =     2 * me->x * me->y + 2 * me->w * me->z;
	mat[1][1] = 1 - 2 * me->x * me->x - 2 * me->z * me->z;
	mat[1][2] =     2 * me->y * me->z - 2 * me->w * me->x;
	mat[1][3] = point->y;

	mat[2][0] =     2 * me->x * me->z - 2 * me->w * me->y;
	mat[2][1] =     2 * me->y * me->z + 2 * me->w * me->x;
	mat[2][2] = 1 - 2 * me->x * me->x - 2 * me->y * me->y;
	mat[2][3] = point->z;

	mat[3][0] = 0;
	mat[3][1] = 0;
	mat[3][2] = 0;
	mat[3][3] = 1;
}

/* Take a rotation quat and decompose it into 2 rotations representing
 * the rotation (twist) around the axis, and the 'swing' between the
 * source rotation axis and the target.
 *
 * Both `me` and `twist_axis` inputs must be normalised.
 *
 * swing * twist gives back the original quat
 * (e.g. oquatf_mult(&swing, &twist, &orig_q))
 *
 * See https://arxiv.org/pdf/1506.05481.pdf
 */
void oquatf_decompose_swing_twist(const quatf *me, const vec3f *twist_axis, quatf *swing, quatf *twist)
{
       quatf twist_inv;
       vec3f orig_axis, projection;
       float dot;

       ovec3f_set(&orig_axis, me->x, me->y, me->z);

       /* Calculate projection onto the twist axis */
       dot = ovec3f_get_dot(&orig_axis, twist_axis);

       ovec3f_multiply_scalar (twist_axis, dot, &projection);
       oquatf_set(twist, projection.x, projection.y, projection.z, me->w);

       if (oquatf_get_dot(twist, twist) < FLT_EPSILON) {
               /* Singularity - 180 degree rotation and perpendicular
                * decomp axis */
               oquatf_set(twist, 0.0, 0.0, 0.0, 1.0);
       }
       else {
               oquatf_normalize_me(twist);
       }

       twist_inv = *twist;
       oquatf_inverse(&twist_inv);
       oquatf_mult(me, &twist_inv, swing);
       oquatf_normalize_me(swing);
}

void oposef_init(posef* p, const vec3f *pos, const quatf *orient)
{
	p->pos = *pos;
	p->orient = *orient;
}

/* Invert a pose - which changes it
 * to the pose of the origin point as
 * seen from the passed in pose.
 *
 * This is equivalent to oposef_apply_inverse()
 * with this pose as the transform, and
 * pos=(0,0,0) and orient=(0,0,0,1) as the
 * src pose to apply it to.
 */
void oposef_inverse(posef *me)
{
	vec3f tmp;

	/* Invert the orientation rotation to get the
	 * rotation back to the original coordinate system */
	oquatf_inverse(&me->orient);

	/* The original origin point is the inverse of the
	 * pose position, rotated by the inverse pose orientation
	 * to make it relative to this new coordinate system */
	tmp = me->pos;
	oquatf_get_rotated(&me->orient, &tmp, &me->pos);
	ovec3f_inverse(&me->pos);
}

/* Mirror the pose on the XZ axes. This is needed
 * specifically for conversion from fusion to device axes,
 * as the fusion tracks the view plane orientation, not the
 * device
 */
void oposef_mirror_XZ(posef *me)
{
	me->pos.x = -me->pos.x;
	me->pos.z = -me->pos.z;
	me->orient.x = -me->orient.x;
	me->orient.z = -me->orient.z;
}

/* Apply a transformation pose to the pose 'me'. This
 * has the effect of using xform as a reference frame, and
 * 'me' as a pose relative to that reference, and calculating
 * the overall pose in the global frame.
 */
void oposef_apply(const posef *me, const posef *xform, posef *dest)
{
	vec3f tmp;
	posef tmp_me, tmp_xform;

	/* Use a temporary, in the case dest == me or xform */
	if (dest == me) {
			tmp_me = *me;
			me = &tmp_me;
	}
	if (dest == xform) {
			tmp_xform = *xform;
			xform = &tmp_xform;
	}

	/* rotate into the global frame, then offset */
	oquatf_get_rotated(&xform->orient, &me->pos, &tmp);
	ovec3f_add(&tmp, &xform->pos, &dest->pos);

	oquatf_mult(&xform->orient, &me->orient, &dest->orient);
}

/* Apply the inverse of 'xform' to the pose 'me'.
 * This has the effect of calculating the pose that is
 * relative to the 'xform' reference frame - moving from global
 * frame to local frame.
 */
void oposef_apply_inverse(const posef *me, const posef *xform, posef *dest)
{
	vec3f tmp;
	quatf tmp_orient = me->orient;
	quatf xform_orient = xform->orient;

	oquatf_inverse(&xform_orient);

	/* Get the rotation of the new pose relative to the
	 * reference frame by rotating it backward */
	oquatf_mult(&xform_orient, &tmp_orient, &dest->orient);

	/* Get the target position relative to the
	 * new reference pose and rotate it into this
	 * pose */
	ovec3f_subtract(&me->pos, &xform->pos, &tmp);
	oquatf_get_rotated(&xform_orient, &tmp, &dest->pos);
}

/*
 * Generate the full 4x4 transformation matrix
 * equivalent to the pose */
void oposef_get_mat4x4(const posef* me, float mat[4][4])
{
	oquatf_get_mat4x4(&me->orient, &me->pos, mat);
}

// matrix

void omat4x4f_init_ident(mat4x4f* me)
{
	memset(me, 0, sizeof(*me));
	me->m[0][0] = 1.0f;
	me->m[1][1] = 1.0f;
	me->m[2][2] = 1.0f;
	me->m[3][3] = 1.0f;
}

void omat4x4f_init_perspective(mat4x4f* me, float fovy_rad, float aspect, float znear, float zfar)
{
	float sine, cotangent, delta, half_fov;

	half_fov = fovy_rad / 2.0f;
	delta = zfar - znear;
	sine = sinf(half_fov);

	if ((delta == 0.0f) || (sine == 0.0f) || (aspect == 0.0f)) {
		omat4x4f_init_ident(me);
		return;
	}

	cotangent = cosf(half_fov) / sine;

	me->m[0][0] = cotangent / aspect;
	me->m[0][1] = 0;
	me->m[0][2] = 0;
	me->m[0][3] = 0;

	me->m[1][0] = 0;
	me->m[1][1] = cotangent;
	me->m[1][2] = 0;
	me->m[1][3] = 0;

	me->m[2][0] = 0;
	me->m[2][1] = 0;
	me->m[2][2] = -(zfar + znear) / delta;
	me->m[2][3] = -2.0f * znear * zfar / delta;

	me->m[3][0] = 0;
	me->m[3][1] = 0;
	me->m[3][2] = -1.0f;
	me->m[3][3] = 0;
}

void omat4x4f_init_frustum(mat4x4f* me, float left, float right, float bottom, float top, float znear, float zfar)
{
	omat4x4f_init_ident(me);

	float delta_x = right - left;
	float delta_y = top - bottom;
	float delta_z = zfar - znear;
	if ((delta_x == 0.0f) || (delta_y == 0.0f) || (delta_z == 0.0f)) {
		/* can't divide by zero, so just give back identity */
		return;
	}

	me->m[0][0] = 2.0f * znear / delta_x;
	me->m[0][1] = 0;
	me->m[0][2] = (right + left) / delta_x;
	me->m[0][3] = 0;

	me->m[1][0] = 0;
	me->m[1][1] = 2.0f * znear / delta_y;
	me->m[1][2] = (top + bottom) / delta_y;
	me->m[1][3] = 0;

	me->m[2][0] = 0;
	me->m[2][1] = 0;
	me->m[2][2] = -(zfar + znear) / delta_z;
	me->m[2][3] = -2.0f * zfar * znear / delta_z;

	me->m[3][0] = 0;
	me->m[3][1] = 0;
	me->m[3][2] = -1.0f;
	me->m[3][3] = 0;
}

void omat4x4f_init_look_at(mat4x4f* me, const quatf* rot, const vec3f* eye)
{
	quatf q;
	vec3f p;

	q.x = -rot->x;
	q.y = -rot->y;
	q.z = -rot->z;
	q.w =  rot->w;

	p.x = -eye->x;
	p.y = -eye->y;
	p.z = -eye->z;

	me->m[0][0] = 1 - 2 * q.y * q.y - 2 * q.z * q.z;
	me->m[0][1] =     2 * q.x * q.y - 2 * q.w * q.z;
	me->m[0][2] =     2 * q.x * q.z + 2 * q.w * q.y;
	me->m[0][3] = p.x * me->m[0][0] + p.y * me->m[0][1] + p.z * me->m[0][2];

	me->m[1][0] =     2 * q.x * q.y + 2 * q.w * q.z;
	me->m[1][1] = 1 - 2 * q.x * q.x - 2 * q.z * q.z;
	me->m[1][2] =     2 * q.y * q.z - 2 * q.w * q.x;
	me->m[1][3] = p.x * me->m[1][0] + p.y * me->m[1][1] + p.z * me->m[1][2];

	me->m[2][0] =     2 * q.x * q.z - 2 * q.w * q.y;
	me->m[2][1] =     2 * q.y * q.z + 2 * q.w * q.x;
	me->m[2][2] = 1 - 2 * q.x * q.x - 2 * q.y * q.y;
	me->m[2][3] = p.x * me->m[2][0] + p.y * me->m[2][1] + p.z * me->m[2][2];

	me->m[3][0] = 0;
	me->m[3][1] = 0;
	me->m[3][2] = 0;
	me->m[3][3] = 1;
}

void omat4x4f_init_translate(mat4x4f* me, float x, float y, float z)
{
	omat4x4f_init_ident(me);
	me->m[0][3] = x;
	me->m[1][3] = y;
	me->m[2][3] = z;
}

void omat4x4f_transpose(const mat4x4f* m, mat4x4f* o)
{
	o->m[0][0] = m->m[0][0];
	o->m[1][0] = m->m[0][1];
	o->m[2][0] = m->m[0][2];
	o->m[3][0] = m->m[0][3];

	o->m[0][1] = m->m[1][0];
	o->m[1][1] = m->m[1][1];
	o->m[2][1] = m->m[1][2];
	o->m[3][1] = m->m[1][3];

	o->m[0][2] = m->m[2][0];
	o->m[1][2] = m->m[2][1];
	o->m[2][2] = m->m[2][2];
	o->m[3][2] = m->m[2][3];

	o->m[0][3] = m->m[3][0];
	o->m[1][3] = m->m[3][1];
	o->m[2][3] = m->m[3][2];
	o->m[3][3] = m->m[3][3];
}

void omat4x4f_mult(const mat4x4f* l, const mat4x4f* r, mat4x4f *o)
{
	for(int i = 0; i < 4; i++){
		float a0 = l->m[i][0], a1 = l->m[i][1], a2 = l->m[i][2], a3 = l->m[i][3];
		o->m[i][0] = a0 * r->m[0][0] + a1 * r->m[1][0] + a2 * r->m[2][0] + a3 * r->m[3][0];
		o->m[i][1] = a0 * r->m[0][1] + a1 * r->m[1][1] + a2 * r->m[2][1] + a3 * r->m[3][1];
		o->m[i][2] = a0 * r->m[0][2] + a1 * r->m[1][2] + a2 * r->m[2][2] + a3 * r->m[3][2];
		o->m[i][3] = a0 * r->m[0][3] + a1 * r->m[1][3] + a2 * r->m[2][3] + a3 * r->m[3][3];
	}
}
// High-precision vector
void ovec3d_set(vec3d* me, double x, double y, double z)
{
	me->x = x;
	me->y = y;
	me->z = z;
}

double ovec3d_get_length(const vec3d* me)
{
	return sqrt(POW2(me->x) + POW2(me->y) + POW2(me->z));
}

void ovec3d_normalize_me(vec3d* me)
{
	if(me->x == 0 && me->y == 0 && me->z == 0)
		return;

	double len = ovec3d_get_length(me);
	me->x /= len;
	me->y /= len;
	me->z /= len;
}

void ovec3d_cross(const vec3d* a, const vec3d* b, vec3d *out)
{
	out->x = a->y * b->z - a->z * b->y;
	out->y = a->z * b->x - a->x * b->z;
	out->z = a->x * b->y - a->y * b->x;
}

void ovec3d_subtract(const vec3d* a, const vec3d* b, vec3d* out)
{
	for(int i = 0; i < 3; i++)
		out->arr[i] = a->arr[i] - b->arr[i];
}

void ovec3d_add(const vec3d* a, const vec3d* b, vec3d* out)
{
	for(int i = 0; i < 3; i++)
		out->arr[i] = a->arr[i] + b->arr[i];
}

void ovec3d_inverse(vec3d *me)
{
	for(int i = 0; i < 3; i++)
		me->arr[i] = -me->arr[i];
}

double ovec3d_get_dot(const vec3d* me, const vec3d* vec)
{
	return me->x * vec->x + me->y * vec->y + me->z * vec->z;
}

double ovec3d_get_angle(const vec3d* me, const vec3d* vec)
{
	double dot = ovec3d_get_dot(me, vec);
	double lengths = ovec3d_get_length(me) * ovec3d_get_length(vec);

	if(lengths == 0)
		return 0;

	dot /= lengths;

	if (dot >= 1.0 || dot <= -1.0)
		return 0.0;

	return acos(dot);
}

void ovec3d_multiply_scalar (const vec3d* a, const double s, vec3d* out)
{
	out->x = a->x * s;
	out->y = a->y * s;
	out->z = a->z * s;
}

// High-precision quaternion
void oquatd_init_axis(quatd* me, const vec3d* vec, double angle)
{
	vec3d norm = *vec;
	ovec3d_normalize_me(&norm);

	me->x = norm.x * sin(angle / 2.0f);
	me->y = norm.y * sin(angle / 2.0f);
	me->z = norm.z * sin(angle / 2.0f);
	me->w = cos(angle / 2.0f);
}

void oquatd_get_rotated(const quatd* me, const vec3d* vec, vec3d* out_vec)
{
	quatd q = {{vec->x * me->w + vec->z * me->y - vec->y * me->z,
	            vec->y * me->w + vec->x * me->z - vec->z * me->x,
	            vec->z * me->w + vec->y * me->x - vec->x * me->y,
	            vec->x * me->x + vec->y * me->y + vec->z * me->z}};

	out_vec->x = me->w * q.x + me->x * q.w + me->y * q.z - me->z * q.y;
	out_vec->y = me->w * q.y + me->y * q.w + me->z * q.x - me->x * q.z;
	out_vec->z = me->w * q.z + me->z * q.w + me->x * q.y - me->y * q.x;
}

/* Map a rotation parameterisation to a quaternion */
void oquatd_from_rotation(quatd *me, const vec3d *rot)
{
  double theta = ovec3d_get_length(rot);
  double c_t = cos(theta / 2.0f);
  double sinc_t;

  /* magic number for numerical stability */
  if (theta > 1e-10) {
    sinc_t = sin(theta / 2.0f) / theta;
  }
  else {
    /* Use Taylor series expansion, as suggested by Grassia -
     * "Practical Parameterization of Rotations Using the Exponential Map"
     */
    sinc_t = 0.5 + theta * theta / 48.0f;
  }

	me->x = rot->x * sinc_t;
	me->y = rot->y * sinc_t;
	me->z = rot->z * sinc_t;
	me->w = c_t;

	oquatd_normalize_me(me);
}

/* Map a quaternion into rotation parameterisation */
void oquatd_to_rotation(const quatd *me, vec3d *rot)
{
  double theta;
  /* FIXME: Consider other parameterisation vectors */

  if (me->w < 0)
    theta = -2*acos(-me->w);
  else
    theta = 2*acos(me->w);

  vec3d v = {{ me->x, me->y, me->z }};
  double v_len = ovec3d_get_length(&v);

  if (v_len > 1e-10) {
    ovec3d_multiply_scalar (&v, theta / v_len, rot);
  }
  else {
    /* quaternion is approximately the identity rotation */
    ovec3d_multiply_scalar (&v, 2.0, rot);
  }

  assert(!isnan(rot->x));
  assert(!isnan(rot->y));
  assert(!isnan(rot->z));
}

void oquatd_mult(const quatd* me, const quatd* q, quatd* out_q)
{
	assert (out_q != me);
	assert (out_q != q);

	out_q->x = me->w * q->x + me->x * q->w + me->y * q->z - me->z * q->y;
	out_q->y = me->w * q->y - me->x * q->z + me->y * q->w + me->z * q->x;
	out_q->z = me->w * q->z + me->x * q->y - me->y * q->x + me->z * q->w;
	out_q->w = me->w * q->w - me->x * q->x - me->y * q->y - me->z * q->z;
}

void oquatd_mult_me(quatd* me, const quatd* q)
{
	quatd tmp = *me;
	oquatd_mult(&tmp, q, me);
}

void oquatd_normalize_me(quatd* me)
{
	double len = oquatd_get_length(me);
	me->x /= len;
	me->y /= len;
	me->z /= len;
	me->w /= len;
}

double oquatd_get_length(const quatd* me)
{
	return sqrt(me->x * me->x + me->y * me->y + me->z * me->z + me->w * me->w);
}

double oquatd_get_dot(const quatd* me, const quatd* q)
{
	return me->x * q->x + me->y * q->y + me->z * q->z + me->w * q->w;
}

void oquatd_inverse(quatd* me)
{
	double dot = oquatd_get_dot(me, me);

	// conjugate
	for(int i = 0; i < 3; i++)
		me->arr[i] = -me->arr[i];
	
	for(int i = 0; i < 4; i++)
		me->arr[i] /= dot;
}

void oquatd_diff(const quatd* me, const quatd* q, quatd* out_q)
{
	quatd inv = *me;
	oquatd_inverse(&inv);
	oquatd_mult(&inv, q, out_q);
}

// filter queue

void ofq_init(filter_queue* me, int size)
{
	memset(me, 0, sizeof(filter_queue));
	me->size = size;
}

void ofq_add(filter_queue* me, const vec3f* vec)
{
	me->elems[me->at] = *vec;
	me->at = ((me->at + 1) % me->size);
}

void ofq_get_mean(const filter_queue* me, vec3f* vec)
{
	vec->x = vec->y = vec->z = 0;
	for(int i = 0; i < me->size; i++){
		vec->x += me->elems[i].x;
		vec->y += me->elems[i].y;
		vec->z += me->elems[i].z;
	}

	vec->x /= (float)me->size;
	vec->y /= (float)me->size;
	vec->z /= (float)me->size;
}
