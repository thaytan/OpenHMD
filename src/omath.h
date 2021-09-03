// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Math */


#ifndef OMATH_H
#define OMATH_H

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define POW2(_x) ((_x) * (_x))
#define RAD_TO_DEG(_r) ((_r) * 360.0f / (2.0f * (float)M_PI))
#define DEG_TO_RAD(_d) ((_d) * (2.0f * (float)M_PI) / 360.0f)


// vector

typedef union {
	struct {
		float x, y, z;
	};
	float arr[3];
} vec3f;

void ovec3f_set(vec3f* me, float x, float y, float z);
void ovec3f_normalize_me(vec3f* me);
float ovec3f_get_length(const vec3f* me);
float ovec3f_get_angle(const vec3f* me, const vec3f* vec);
float ovec3f_get_dot(const vec3f* me, const vec3f* vec);
void ovec3f_cross(const vec3f* a, const vec3f* b, vec3f *out);
void ovec3f_subtract(const vec3f* a, const vec3f* b, vec3f* out);
void ovec3f_add(const vec3f* a, const vec3f* b, vec3f* out);
void ovec3f_inverse(vec3f *me);
void ovec3f_multiply_scalar (const vec3f* a, const float s, vec3f* out);
void ovec3f_multiply_mat3x3(const vec3f *a, const float m[3][3], vec3f *out);

// quaternion

typedef union {
	struct {
		float x, y, z, w;
	};
	float arr[4];
} quatf;

void oquatf_init_axis(quatf* me, const vec3f* vec, float angle);
void oquatf_set(quatf* me, float x, float y, float z, float w);
void oquatf_from_vectors(quatf* me, const vec3f* from, const vec3f* to);

void oquatf_get_rotated(const quatf* me, const vec3f* vec, vec3f* out_vec);
void oquatf_get_rotated_abs(const quatf* me, const vec3f* vec, vec3f* out_vec);
void oquatf_mult_me(quatf* me, const quatf* q);
void oquatf_from_rotation(quatf *me, const vec3f *rot);
void oquatf_to_rotation(const quatf *me, vec3f *rot);
void oquatf_mult(const quatf* me, const quatf* q, quatf* out_q);
void oquatf_diff(const quatf* me, const quatf* q, quatf* out_q);
void oquatf_normalize_me(quatf* me);
float oquatf_get_length(const quatf* me);
float oquatf_get_dot(const quatf* me, const quatf* q);
void oquatf_inverse(quatf* me);

void oquatf_get_mat4x4(const quatf* me, const vec3f* point, float mat[4][4]);
void oquatf_decompose_swing_twist(const quatf *me, const vec3f *twist_axis, quatf *swing, quatf *twist);

// Pose = position + orientation
typedef struct {
	vec3f pos;
	quatf orient;
} posef;

void oposef_init(posef* p, const vec3f *pos, const quatf *orient);
void oposef_inverse(posef *me);
void oposef_mirror_XZ(posef *me);
void oposef_apply(const posef *me, const posef *xform, posef *dest);
void oposef_apply_inverse(const posef *me, const posef *xform, posef *dest);
void oposef_get_mat4x4(const posef* me, float mat[4][4]);

// matrix

typedef union {
	float m[4][4];
	float arr[16];
} mat4x4f;

void omat4x4f_init_ident(mat4x4f* me);
void omat4x4f_init_perspective(mat4x4f* me, float fov_rad, float aspect, float znear, float zfar);
void omat4x4f_init_frustum(mat4x4f* me, float left, float right, float bottom, float top, float znear, float zfar);
void omat4x4f_init_look_at(mat4x4f* me, const quatf* ret, const vec3f* eye);
void omat4x4f_init_translate(mat4x4f* me, float x, float y, float z);
void omat4x4f_mult(const mat4x4f* left, const mat4x4f* right, mat4x4f* out_mat);
void omat4x4f_transpose(const mat4x4f* me, mat4x4f* out_mat);

// High-precision vec
typedef union {
	struct {
		double x, y, z;
	};
	double arr[3];
} vec3d;

void ovec3d_set(vec3d* me, double x, double y, double z);
void ovec3d_normalize_me(vec3d* me);
double ovec3d_get_length(const vec3d* me);
double ovec3d_get_angle(const vec3d* me, const vec3d* vec);
double ovec3d_get_dot(const vec3d* me, const vec3d* vec);
void ovec3d_cross(const vec3d* a, const vec3d* b, vec3d *out);
void ovec3d_subtract(const vec3d* a, const vec3d* b, vec3d* out);
void ovec3d_add(const vec3d* a, const vec3d* b, vec3d* out);
void ovec3d_inverse(vec3d *me);
void ovec3d_multiply_scalar (const vec3d* a, const double s, vec3d* out);

// High precision quat
typedef union {
	struct {
		double x, y, z, w;
	};
	double arr[4];
} quatd;

void oquatd_init_axis(quatd* me, const vec3d* vec, double angle);

void oquatd_get_rotated(const quatd* me, const vec3d* vec, vec3d* out_vec);
void oquatd_from_rotation(quatd *me, const vec3d *rot);
void oquatd_to_rotation(const quatd *me, vec3d *rot);
void oquatd_mult_me(quatd* me, const quatd* q);
void oquatd_mult(const quatd* me, const quatd* q, quatd* out_q);
void oquatd_diff(const quatd* me, const quatd* q, quatd* out_q);
void oquatd_normalize_me(quatd* me);
double oquatd_get_length(const quatd* me);
double oquatd_get_dot(const quatd* me, const quatd* q);
void oquatd_inverse(quatd* me);

// filter queue
#define FILTER_QUEUE_MAX_SIZE 256

typedef struct {
	int at, size;
	vec3f elems[FILTER_QUEUE_MAX_SIZE];
} filter_queue;

void ofq_init(filter_queue* me, int size);
void ofq_add(filter_queue* me, const vec3f* vec);
void ofq_get_mean(const filter_queue* me, vec3f* vec);

#endif
