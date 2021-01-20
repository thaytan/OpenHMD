/*
 * GStreamer
 * Copyright (C) 2006 Stefan Kost <ensonic@users.sf.net>
 * Copyright (C) 2019 Jan Schmidt <<user@hostname.org>>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * SECTION:element-ohmdriftsensor
 *
 * Filter for doing processing experiments on OpenHMD sensor input
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch -v -m filesrc location=test-file.mp4 ! decodebin ! ohmdriftsensor ! glimagesink
 * ]|
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <gst/gst.h>
#include <gst/base/base.h>
#include <gst/controller/controller.h>

#include "gstohmdriftsensor.h"

#define KALMAN_FILTER 1

#define DEFAULT_DRAW_SIDE_BY_SIDE FALSE
#define DEFAULT_DRAW_LED_MARKERS TRUE
#define DEFAULT_DRAW_BLOB_SQUARES FALSE
#define DEFAULT_DUMP_BLOBS FALSE

const guint32 OHMD_MARKER = ('O' << 24 | 'H' << 16 | 'M' << 8 | 'D');

GST_DEBUG_CATEGORY_STATIC (gst_ohmd_rift_sensor_debug);
#define GST_CAT_DEFAULT gst_ohmd_rift_sensor_debug

/* Filter signals and args */
enum
{
  /* FILL ME */
  LAST_SIGNAL
};

enum
{
  PROP_0,
  PROP_DRAW_SIDE_BY_SIDE,
  PROP_DRAW_LED_MARKERS,
  PROP_DRAW_BLOB_SQUARES,
  PROP_DUMP_BLOBS
};

/* the capabilities of the inputs and outputs.
 *
 * FIXME:describe the real formats here.
 */
static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-raw,format=(string)GRAY8")
    );

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-raw,format=(string)RGB")
    );

#define gst_ohmd_rift_sensor_parent_class parent_class
G_DEFINE_TYPE (GstOhmdRiftSensor, gst_ohmd_rift_sensor, GST_TYPE_VIDEO_FILTER);

static void gst_ohmd_rift_sensor_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_ohmd_rift_sensor_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

static gboolean gst_ohmd_rift_sensor_set_info (GstVideoFilter * filter,
    GstCaps * incaps, GstVideoInfo * in_info, GstCaps * outcaps,
    GstVideoInfo * out_info);

static GstFlowReturn gst_ohmd_rift_sensor_transform_frame (GstVideoFilter *
    filter, GstVideoFrame * in_frame, GstVideoFrame * out_frame);

/* Function that can halve the value
 * of ints, fractions, int/fraction ranges and lists of ints/fractions */
static gboolean
_halve_value (GValue * out, const GValue * in_value)
{
  /* Fundamental fixed types first */
  if (G_VALUE_HOLDS_INT (in_value)) {
    g_value_init (out, G_TYPE_INT);
    g_value_set_int (out, MAX (g_value_get_int (in_value) / 2, 1));
  } else if (GST_VALUE_HOLDS_INT_RANGE (in_value)) {
    gint range_min = gst_value_get_int_range_min (in_value);
    gint range_max = gst_value_get_int_range_max (in_value);
    gint range_step = gst_value_get_int_range_step (in_value);
    g_value_init (out, GST_TYPE_INT_RANGE);
    if (range_min != 1)
      range_min = MAX (1, range_min / 2);
    if (range_max != G_MAXINT)
      range_max = MAX (1, range_max / 2);
    gst_value_set_int_range_step (out, range_min,
        range_max, MAX (1, range_step / 2));
  } else if (GST_VALUE_HOLDS_LIST (in_value)) {
    gint i;
    g_value_init (out, GST_TYPE_LIST);
    for (i = 0; i < gst_value_list_get_size (in_value); i++) {
      const GValue *entry;
      GValue tmp = G_VALUE_INIT;

      entry = gst_value_list_get_value (in_value, i);
      /* Random list values might not be the right type */
      if (!_halve_value (&tmp, entry))
        goto fail;
      gst_value_list_append_and_take_value (out, &tmp);
    }
  } else {
    return FALSE;
  }

  return TRUE;
fail:
  g_value_unset (out);
  return FALSE;
}

static GstStructure *
_halve_structure_field (const GstStructure * in, const gchar * field_name)
{
  GstStructure *out;
  const GValue *in_value = gst_structure_get_value (in, field_name);
  GValue tmp = G_VALUE_INIT;

  if (G_UNLIKELY (in_value == NULL))
    return gst_structure_copy (in);     /* Field doesn't exist, leave it as is */

  if (!_halve_value (&tmp, in_value))
    return NULL;

  out = gst_structure_copy (in);
  gst_structure_set_value (out, field_name, &tmp);
  g_value_unset (&tmp);

  return out;
}

/* Function that can double the value
 * of ints, fractions, int/fraction ranges and lists of ints/fractions */
static gboolean
_double_value (GValue * out, const GValue * in_value)
{
  /* Fundamental fixed types first */
  if (G_VALUE_HOLDS_INT (in_value)) {
    gint n = g_value_get_int (in_value);
    g_value_init (out, G_TYPE_INT);
    if (n <= G_MAXINT / 2)
      g_value_set_int (out, n * 2);
    else
      g_value_set_int (out, G_MAXINT);
  } else if (GST_VALUE_HOLDS_INT_RANGE (in_value)) {
    gint range_min = gst_value_get_int_range_min (in_value);
    gint range_max = gst_value_get_int_range_max (in_value);
    gint range_step = gst_value_get_int_range_step (in_value);
    if (range_min != 1) {
      range_min = MIN (G_MAXINT / 2, range_min);
      range_min *= 2;
    }
    if (range_max != G_MAXINT) {
      range_max = MIN (G_MAXINT / 2, range_max);
      range_max *= 2;
    }
    range_step = MIN (G_MAXINT / 2, range_step);
    g_value_init (out, GST_TYPE_INT_RANGE);
    gst_value_set_int_range_step (out, range_min, range_max, range_step);
  } else if (GST_VALUE_HOLDS_LIST (in_value)) {
    gint i;
    g_value_init (out, GST_TYPE_LIST);
    for (i = 0; i < gst_value_list_get_size (in_value); i++) {
      const GValue *entry;
      GValue tmp = G_VALUE_INIT;

      entry = gst_value_list_get_value (in_value, i);
      /* Random list values might not be the right type */
      if (!_double_value (&tmp, entry))
        goto fail;
      gst_value_list_append_and_take_value (out, &tmp);
    }
  } else {
    return FALSE;
  }

  return TRUE;
fail:
  g_value_unset (out);
  return FALSE;
}

static GstStructure *
_double_structure_field (const GstStructure * in, const gchar * field_name)
{
  GstStructure *out;
  const GValue *in_value = gst_structure_get_value (in, field_name);
  GValue tmp = G_VALUE_INIT;

  if (G_UNLIKELY (in_value == NULL))
    return gst_structure_copy (in);     /* Field doesn't exist, leave it as is */

  if (!_double_value (&tmp, in_value))
    return NULL;

  out = gst_structure_copy (in);
  gst_structure_set_value (out, field_name, &tmp);
  g_value_unset (&tmp);

  return out;
}

static GstCaps *
gst_ohmd_rift_sensor_caps_remove_format_info (GstCaps * caps)
{
  GstStructure *st;
  GstCapsFeatures *f;
  gint i, n;
  GstCaps *res;

  res = gst_caps_new_empty ();

  n = gst_caps_get_size (caps);
  for (i = 0; i < n; i++) {
    st = gst_caps_get_structure (caps, i);
    f = gst_caps_get_features (caps, i);

    /* If this is already expressed by the existing caps
     * skip this structure */
    if (i > 0 && gst_caps_is_subset_structure_full (res, st, f))
      continue;

    st = gst_structure_copy (st);
    /* Only remove format info for the cases when we can actually convert */
    if (!gst_caps_features_is_any (f)
        && gst_caps_features_is_equal (f,
            GST_CAPS_FEATURES_MEMORY_SYSTEM_MEMORY))
      gst_structure_remove_fields (st, "format", NULL);

    gst_caps_append_structure_full (res, st, gst_caps_features_copy (f));
  }

  return res;
}

/* Return a copy of the caps with the requested field halved in value/range */
static GstCaps *
_halve_caps_field (const GstCaps * in, const gchar * field_name)
{
  gint i;
  GstCaps *out = gst_caps_new_empty ();

  for (i = 0; i < gst_caps_get_size (in); i++) {
    const GstStructure *cur = gst_caps_get_structure (in, i);
    GstCapsFeatures *f = gst_caps_get_features (in, i);

    GstStructure *res = _halve_structure_field (cur, field_name);
    out =
        gst_caps_merge_structure_full (out, res,
        f ? gst_caps_features_copy (f) : NULL);
  }

  return out;
}

/* Return a copy of the caps with the requested field doubled in value/range */
static GstCaps *
_double_caps_field (const GstCaps * in, const gchar * field_name)
{
  gint i;
  GstCaps *out = gst_caps_new_empty ();

  for (i = 0; i < gst_caps_get_size (in); i++) {
    const GstStructure *cur = gst_caps_get_structure (in, i);
    GstCapsFeatures *f = gst_caps_get_features (in, i);

    GstStructure *res = _double_structure_field (cur, field_name);
    out =
        gst_caps_merge_structure_full (out, res,
        f ? gst_caps_features_copy (f) : NULL);
  }

  return out;
}

static GstCaps *
gst_ohmd_rift_sensor_transform_caps (GstBaseTransform * btrans,
    GstPadDirection direction, GstCaps * caps, GstCaps * filter_caps)
{
  GstOhmdRiftSensor *filter = GST_OHMDRIFTSENSOR (btrans);
  GstCaps *tmp, *tmp2;
  GstCaps *result;

  /* Get all possible caps that we can transform to */
  tmp = gst_ohmd_rift_sensor_caps_remove_format_info (caps);

  if (filter->draw_side_by_side) {
    /* We need to double or halve the width, depending on the direction */
    if (direction == GST_PAD_SINK) {
      tmp2 = _double_caps_field (tmp, "width");
    } else {
      tmp2 = _halve_caps_field (tmp, "width");
    }

    gst_caps_unref (tmp);
    tmp = tmp2;
  }

  if (filter_caps) {
    tmp2 = gst_caps_intersect_full (filter_caps, tmp, GST_CAPS_INTERSECT_FIRST);
    gst_caps_unref (tmp);
    tmp = tmp2;
  }

  result = tmp;

  GST_DEBUG_OBJECT (btrans, "transformed %" GST_PTR_FORMAT " into %"
      GST_PTR_FORMAT, caps, result);

  return result;
}

static void
gst_ohmd_rift_sensor_finalize (GObject * obj)
{
  GstOhmdRiftSensor *filter = GST_OHMDRIFTSENSOR (obj);

  if (filter->bw)
    blobwatch_free (filter->bw);
  if (filter->cs)
    correspondence_search_free (filter->cs);

  g_free (filter->led_out_points);
  G_OBJECT_CLASS (parent_class)->finalize (obj);
}

/* initialize the ohmdriftsensor's class */
static void
gst_ohmd_rift_sensor_class_init (GstOhmdRiftSensorClass * klass)
{
  GObjectClass *gobject_class = (GObjectClass *) klass;
  GstElementClass *gstelement_class = (GstElementClass *) klass;
  GstBaseTransformClass *gstbasetransform_class =
      (GstBaseTransformClass *) klass;
  GstVideoFilterClass *gstvideofilter_class = (GstVideoFilterClass *) klass;

  gobject_class->finalize = gst_ohmd_rift_sensor_finalize;
  gobject_class->set_property = gst_ohmd_rift_sensor_set_property;
  gobject_class->get_property = gst_ohmd_rift_sensor_get_property;

  gst_element_class_set_details_simple (gstelement_class,
      "OpenHMD Rift Sensor filter",
      "Video/Filter",
      "OpenHMD Oculus Rift experiements filter",
      "Jan Schmidt <jan@centricular.com>");

  gst_element_class_add_pad_template (gstelement_class,
      gst_static_pad_template_get (&src_template));
  gst_element_class_add_pad_template (gstelement_class,
      gst_static_pad_template_get (&sink_template));

  gstvideofilter_class->set_info =
      GST_DEBUG_FUNCPTR (gst_ohmd_rift_sensor_set_info);

  gstbasetransform_class->transform_caps =
      GST_DEBUG_FUNCPTR (gst_ohmd_rift_sensor_transform_caps);
  gstvideofilter_class->transform_frame =
      GST_DEBUG_FUNCPTR (gst_ohmd_rift_sensor_transform_frame);

  GST_DEBUG_CATEGORY_INIT (gst_ohmd_rift_sensor_debug, "ohmdriftsensor", 0,
      "OpenHMD Rift Sensor Filter");

  g_object_class_install_property (gobject_class, PROP_DRAW_SIDE_BY_SIDE,
      g_param_spec_boolean ("draw-side-by-side", "Draw Side-by-Side",
          "Draw the processed frame with the original frame side-by-side",
          DEFAULT_DRAW_SIDE_BY_SIDE,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (gobject_class, PROP_DRAW_LED_MARKERS,
      g_param_spec_boolean ("draw-led-markers", "Draw LED markers",
          "Draw the processed frame markers for extracted LED positions",
          DEFAULT_DRAW_LED_MARKERS,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (gobject_class, PROP_DRAW_BLOB_SQUARES,
      g_param_spec_boolean ("draw-blob-squares", "Draw BLOB squares",
          "Draw the squares around detected blobs", DEFAULT_DRAW_BLOB_SQUARES,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (gobject_class, PROP_DUMP_BLOBS,
      g_param_spec_boolean ("dump-blobs", "Dump BLOBs", "Dump BLOB info",
          DEFAULT_DUMP_BLOBS, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
}

static void
gst_ohmd_rift_sensor_setup (GstOhmdRiftSensor * filter)
{
  GST_INFO_OBJECT (filter, "Configuring for %s processing",
      filter->is_cv1 ? "CV1" : "DK2");
  if (filter->is_cv1) {
    rift_leds_init (&filter->leds[0], 0);
    rift_leds_init (&filter->leds[1], 1);
    rift_leds_init (&filter->leds[2], 2);
  } else
    rift_leds_init_dk2 (&filter->leds[0], 0);

  filter->last_pattern_time = GST_CLOCK_TIME_NONE;

  memset (&filter->pose, 0, sizeof (filter->pose));

  double *const A = filter->camera_matrix.m;
  double *const k = filter->dist_coeffs;

  if (filter->is_cv1) {
#if 0
    /* Camera matrix taken from one of my sensors (WMTD304A501EB2):
     * f = [ 715.185 715.185 ], c = [ 658.333 469.870 ]
     * k = [  0.069530 -0.019189  0.001986  0.000214 ]
     */
    A[0] = 715.185;
    A[2] = 658.333;
    A[4] = 715.185;
    A[5] = 469.870;
    A[8] = 1.0;

    k[0] = 0.069530;
    k[1] = -0.019189;
    k[2] = 0.001986;
    k[3] = 0.000214;
#else
    /* Camera matrix taken from one of my sensors (WMTD30333009TJ):
     * f = [ 715.258 715.258 ], c = [ 636.018, 488.010 ]
     * k = [  0.069530 -0.019189  0.001986  0.000214 ]
     */
    A[0] = 715.258;
    A[2] = 636.018;
    A[4] = 715.258;
    A[5] = 488.010;
    A[8] = 1.0;

    k[0] = 0.074007;
    k[1] = -0.025007;
    k[2] = 0.005786;
    k[3] = -0.000620;
#endif
  } else {
    A[0] = 684.600;
    A[2] = 385.521;
    A[4] = 684.671;
    A[5] = 238.878;

    k[0] = 0.069530;
    k[1] = -0.019189;
    k[2] = 0.001986;
    k[3] = 0.000214;
    k[0] = -0.491153;
    k[1] = 0.295728;
    k[2] = -0.000887;
    k[3] = -0.000456;
    k[4] = -0.102742;
  }

  if (filter->cs)
    correspondence_search_free (filter->cs);
  filter->cs =
      correspondence_search_new (&filter->camera_matrix, filter->dist_coeffs,
      filter->is_cv1);

  filter->led_models[0] = led_search_model_new (&filter->leds[0]);
  correspondence_search_set_model (filter->cs, 0, filter->led_models[0]);

  if (filter->is_cv1) {
    filter->led_models[1] = led_search_model_new (&filter->leds[1]);
    correspondence_search_set_model (filter->cs, 1, filter->led_models[1]);
    filter->led_models[2] = led_search_model_new (&filter->leds[2]);
    correspondence_search_set_model (filter->cs, 2, filter->led_models[2]);
  }
}


static void
gst_ohmd_rift_sensor_init (GstOhmdRiftSensor * filter)
{
  filter->led_out_points = g_new0 (vec3f, 50);  /* At least as big as the HMD LED array */

  filter->draw_side_by_side = DEFAULT_DRAW_SIDE_BY_SIDE;
  filter->draw_led_markers = DEFAULT_DRAW_LED_MARKERS;
  filter->draw_blob_squares = DEFAULT_DRAW_BLOB_SQUARES;
  filter->dump_blobs = DEFAULT_DUMP_BLOBS;
}

static void
gst_ohmd_rift_sensor_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstOhmdRiftSensor *filter = GST_OHMDRIFTSENSOR (object);

  switch (prop_id) {
    case PROP_DRAW_SIDE_BY_SIDE:
      filter->draw_side_by_side = g_value_get_boolean (value);
      break;
    case PROP_DRAW_LED_MARKERS:
      filter->draw_led_markers = g_value_get_boolean (value);
      break;
    case PROP_DRAW_BLOB_SQUARES:
      filter->draw_blob_squares = g_value_get_boolean (value);
      break;
    case PROP_DUMP_BLOBS:
      filter->dump_blobs = g_value_get_boolean (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_ohmd_rift_sensor_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstOhmdRiftSensor *filter = GST_OHMDRIFTSENSOR (object);

  switch (prop_id) {
    case PROP_DRAW_SIDE_BY_SIDE:
      g_value_set_boolean (value, filter->draw_side_by_side);
      break;
    case PROP_DRAW_LED_MARKERS:
      g_value_set_boolean (value, filter->draw_led_markers);
      break;
    case PROP_DRAW_BLOB_SQUARES:
      g_value_set_boolean (value, filter->draw_blob_squares);
      break;
    case PROP_DUMP_BLOBS:
      g_value_set_boolean (value, filter->dump_blobs);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static gboolean
gst_ohmd_rift_sensor_set_info (GstVideoFilter * vf,
    GstCaps * incaps, GstVideoInfo * in_info, GstCaps * outcaps,
    GstVideoInfo * out_info)
{
  GstOhmdRiftSensor *filter = GST_OHMDRIFTSENSOR (vf);

  filter->fps_n = in_info->fps_n;
  filter->fps_d = in_info->fps_d;
  if (filter->fps_n == 0 || filter->fps_d == 0) {
    filter->fps_n = 625;
    filter->fps_n = 12;
  }

  if (in_info->width == 1280)
    filter->is_cv1 = true;
  else
    filter->is_cv1 = false;

  gst_ohmd_rift_sensor_setup (filter);

  if (filter->bw)
    blobwatch_free (filter->bw);
  filter->bw =
      blobwatch_new (filter->is_cv1 ? BLOB_THRESHOLD_CV1 : BLOB_THRESHOLD_DK2,
      in_info->width, in_info->height);
  blobwatch_set_flicker (filter->bw, false);
  return TRUE;
}

static void
draw_rgb_marker (guint8 * pixels, gint width, gint stride, gint height,
    gint x_pos, gint y_pos, gint mark_width, gint mark_height, guint32 colour)
{
  gint x, y;
  gint min_x = MAX (0, x_pos - mark_width / 2);
  gint max_x = MIN (width, x_pos + mark_width / 2 + 1.5);
  gint min_y = MAX (0, y_pos - mark_height / 2);
  gint max_y = MIN (height, y_pos + mark_height / 2 + 1.5);

  if (y_pos < 0 || y_pos >= height)
    return;
  if (x_pos < 0 || x_pos >= width)
    return;

  /* Horizontal line */
  guint8 *dest = pixels + stride * y_pos + 3 * min_x;
  for (x = 0; x < max_x - min_x; x++) {
    GST_WRITE_UINT24_BE (dest, colour);
    dest += 3;
  }

  /* Vertical line */
  dest = pixels + stride * min_y + 3 * x_pos;
  for (y = 0; y < max_y - min_y; y++) {
    GST_WRITE_UINT24_BE (dest, colour);
    dest += stride;
  }
}

static void
clamp (gint * val, gint max)
{
  if (*val < 0)
    *val = 0;
  if (*val >= max)
    *val = max - 1;
}

static void
clamp_rect (gint * x, gint * y, gint * rw, gint * rh, gint width, gint height)
{
  clamp (x, width);
  clamp (y, height);
  clamp (rw, width - *x);
  clamp (rh, height - *y);
}


static void
draw_rgb_rect (guint8 * pixels, gint width, gint stride, gint height,
    gint start_x, gint start_y, gint box_width, gint box_height, guint32 colour)
{
  clamp_rect (&start_x, &start_y, &box_width, &box_height, width, height);

  gint x, y;
  guint8 *dest = pixels + stride * start_y + 3 * start_x;
  for (x = 0; x < box_width; x++) {
    GST_WRITE_UINT24_BE (dest, colour);
    dest += 3;
  }

  for (y = 1; y < box_height - 1; y++) {
    dest = pixels + stride * (start_y + y) + 3 * start_x;

    GST_WRITE_UINT24_BE (dest, colour);
    dest += 3 * (box_width - 1);
    GST_WRITE_UINT24_BE (dest, colour);
  }

  dest = pixels + stride * (start_y + box_height - 1) + 3 * start_x;
  for (x = 0; x < box_width; x++) {
    GST_WRITE_UINT24_BE (dest, colour);
    dest += 3;
  }
}

static void
paint_rgb_rect (guint8 * in_pixels, gint in_stride, guint8 * pixels, gint width,
    gint stride, gint height, gint start_x, gint start_y, gint box_width,
    gint box_height, guint32 mask_colour)
{
  clamp_rect (&start_x, &start_y, &box_width, &box_height, width, height);

  gint x, y;
  guint8 *src;
  guint8 *dest;
  gint r = mask_colour >> 16;
  gint g = (mask_colour >> 8) & 0xFF;
  gint b = mask_colour & 0xFF;

  for (y = 0; y < box_height; y++) {
    src = in_pixels + in_stride * (start_y + y) + start_x;
    dest = pixels + stride * (start_y + y) + 3 * start_x;
    for (x = 0; x < box_width; x++) {
      guint8 s = src[0];
      *dest++ = (r * s) >> 8;
      *dest++ = (g * s) >> 8;
      *dest++ = (b * s) >> 8;
      src++;
    }
  }
}

static void
draw_rgb_filled_rect (guint8 * pixels, gint width, gint stride, gint height,
    gint start_x, gint start_y, gint box_width, gint box_height, guint32 colour)
{
  clamp_rect (&start_x, &start_y, &box_width, &box_height, width, height);

  gint x, y;
  guint8 *dest;
  for (y = 0; y < box_height; y++) {
    dest = pixels + stride * (start_y + y) + 3 * start_x;
    for (x = 0; x < box_width; x++) {
      GST_WRITE_UINT24_BE (dest, colour);
      dest += 3;
    }
  }
}


static gboolean
tracker_process_blobs (GstOhmdRiftSensor * filter, GstClockTime ts)
{
  struct blobservation *bwobs = filter->bwobs;
  gboolean ret = FALSE;
  int d;
  rift_pose_metrics score;

#if 1
  correspondence_search_set_blobs (filter->cs, bwobs->blobs, bwobs->num_blobs);

  for (d = 0; d < 3; d++) {
    bool match_all_blobs = (d == 0);    /* Let the HMD match whatever it can */

    if (correspondence_search_find_one_pose (filter->cs, d, match_all_blobs,
            &filter->pose, &score)) {
      ret = TRUE;

      if (score.good_pose_match) {
        /* Clear existing blob IDs for this device, then
         * back project LED ids into blobs if we find them and the dot product
         * shows them pointing strongly to the camera */
        for (int index = 0; index < filter->bwobs->num_blobs; index++) {
          struct blob *b = filter->bwobs->blobs + index;
          if (LED_OBJECT_ID (b->led_id) == d) {
            b->led_id = LED_INVALID_ID;
          }
        }

        rift_mark_matching_blobs (&filter->pose,
            bwobs->blobs, bwobs->num_blobs, d,
            filter->leds[d].points, filter->leds[d].num_points,
            &filter->camera_matrix, filter->dist_coeffs, filter->is_cv1);

        /* Refine the pose with PnP now that we've labelled the blobs */
        estimate_initial_pose (filter->bwobs->blobs, filter->bwobs->num_blobs,
            d, filter->leds[d].points, filter->leds[d].num_points,
            &filter->camera_matrix, filter->dist_coeffs, filter->is_cv1,
            &filter->pose, NULL, NULL, true);
      }
    }
  }
#else
  dmat3 *camera_matrix = &filter->camera_matrix;
  double *dist_coeffs = filter->dist_coeffs;
  //double dist_coeffs[4] = { 0, };
  quatf rot = filter->pose_orient;
  vec3f trans = filter->pose_pos;
  int num_leds = 0;

  /*
   * Estimate initial pose without previously known [rot|trans].
   */
  if (estimate_initial_pose (bwobs->blobs, bwobs->num_blobs,
          filter->leds[0].points, filter->leds[0].num_points, camera_matrix,
          dist_coeffs, filter->is_cv1, &rot, &trans, &num_leds, true)) {

    filter->pose_pos = trans;
    filter->pose_orient = rot;

#if 0
    /* Test code that just spins the HMD around 1 axis for visualisation testing */
    quatf pose_orient;
    pose_orient.x = cos (filter->angle);
    pose_orient.y = 0;
    pose_orient.z = sin (filter->angle);
    pose_orient.w = 0;
    filter->angle += M_PI / 60;
    filter->pose_orient = pose_orient;
#endif

    g_print
        ("sensor %u Got PnP pose quat %f %f %f %f  pos %f %f %f from %d LEDs %d blobs\n",
        0, filter->pose_orient.x, filter->pose_orient.y, filter->pose_orient.z,
        filter->pose_orient.w, filter->pose_pos.x, filter->pose_pos.y,
        filter->pose_pos.z, num_leds, bwobs->num_blobs);
    ret = TRUE;
  }
#endif

  return ret;
}

int
compare_blobs (const void *elem1, const void *elem2)
{
  const struct blob *b1 = *(struct blob **) elem1;
  const struct blob *b2 = *(struct blob **) elem2;

  if (b1->y > b2->y)
    return 1;
  if (b1->y < b2->y)
    return -1;

  if (b1->x > b2->x)
    return 1;
  if (b1->x < b2->x)
    return -1;

  return 0;
}

static GstFlowReturn
gst_ohmd_rift_sensor_transform_frame (GstVideoFilter * base,
    GstVideoFrame * in_frame, GstVideoFrame * out_frame)
{
  GstOhmdRiftSensor *filter = GST_OHMDRIFTSENSOR (base);
  gint x, y;
  guint8 *src = in_frame->data[0];
  guint8 *dest = out_frame->data[0];
  gint width = in_frame->info.width;
  gint height = in_frame->info.height;
  gint in_stride = in_frame->info.stride[0];
  gint out_stride = out_frame->info.stride[0];
  guint8 led_pattern_phase = filter->led_pattern_phase;
  GstClockTime in_ts = GST_BUFFER_PTS (in_frame->buffer);

  /* If there's an OHMD marker in the frame, we can read the pattern phase directly */
  if (GST_READ_UINT32_BE (src) == OHMD_MARKER) {
    led_pattern_phase = src[4];
    GST_LOG_OBJECT (filter, "Have LED pattern phase %u in frame",
        led_pattern_phase);
  } else {
    gint n_frames = 1;

    /* Guess based on how much time passed */
    if (GST_CLOCK_TIME_IS_VALID (in_ts)) {
      if (GST_CLOCK_TIME_IS_VALID (filter->last_pattern_time)) {
        GstClockTimeDiff ts_diff = in_ts - filter->last_pattern_time;
        /* Calc frames elapsed, rounded to the nearest */
        n_frames =
            gst_util_uint64_scale_round (ts_diff, filter->fps_n,
            filter->fps_d * GST_SECOND);
        GST_LOG_OBJECT (filter, "TS diff %" G_GUINT64_FORMAT " frames = %d",
            ts_diff, n_frames);
      }
      filter->last_pattern_time = in_ts;
    }
    led_pattern_phase = (led_pattern_phase + n_frames) % 10;
  }

  filter->led_pattern_phase = led_pattern_phase;

  if (GST_CLOCK_TIME_IS_VALID (in_ts))
    gst_object_sync_values (GST_OBJECT (filter), in_ts);

  blobwatch_process (filter->bw, src, width, height,
      led_pattern_phase, NULL, 0, &filter->bwobs);

  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      guint8 *d = dest + x * 3;
      /* Expand GRAY8 to yellow */
      d[0] = d[1] = src[x];
      d[2] = 0;
    }
    if (filter->draw_side_by_side) {
      /* Expand to RGB and copy the source to the RHS */
      guint8 *d = dest + width * 3;
      for (x = 0; x < width; x++) {
        /* Expand GRAY8 to yellow */
        d[0] = d[1] = d[2] = src[x];
        d += 3;
      }
    }

    dest += out_stride;
    src += in_stride;
  }

  if (filter->bwobs) {
    if (filter->bwobs->num_blobs > 0) {
      struct blob *sorted_blobs[MAX_BLOBS_PER_FRAME];

      GST_INFO_OBJECT (filter, "Sensor %d phase %d Blobs: %d", filter->id,
          led_pattern_phase, filter->bwobs->num_blobs);

      /* Copy pointers into the sorted blobs array */
      for (int index = 0; index < filter->bwobs->num_blobs; index++)
        sorted_blobs[index] = filter->bwobs->blobs + index;

      if (filter->dump_blobs) {
        g_print ("Sensor %d phase %d Blobs: %d\n", filter->id,
            led_pattern_phase, filter->bwobs->num_blobs);

        qsort (sorted_blobs, filter->bwobs->num_blobs, sizeof (struct blob *),
            compare_blobs);

        for (int index = 0; index < filter->bwobs->num_blobs; index++) {
          struct blob *b = sorted_blobs[index];
          g_print
              ("Sensor %d Blob)[%d]: %f,%f %dx%d (age %d) id %d pattern %x (unchanged %u)\n",
              filter->id, index, b->x, b->y, b->width, b->height, b->age,
              b->led_id, b->pattern, b->pattern_age);
        }
        g_print ("\n");
      }

      /* Process the blobs, which also label them if we find good
       * poses */
      bool found_poses = tracker_process_blobs (filter, in_ts);

      /* Draw the blobs in the video */
      for (int index = 0; index < filter->bwobs->num_blobs; index++) {
        struct blob *b = sorted_blobs[index];
        gint start_x, start_y, w, h;

        start_x = b->left;
        start_y = b->top;
        w = b->width + 0.5;
        h = b->height + 0.5;
        clamp_rect (&start_x, &start_y, &w, &h, width, height);

        /* Paint unknown blobs purple, known blobs green */
        if (filter->draw_blob_squares) {
          draw_rgb_rect (out_frame->data[0], width, out_stride, height, start_x,
              start_y, b->width, b->height,
              b->led_id == LED_INVALID_ID ? 0xFF00FF : 0x00FFFF);
        } else {
          paint_rgb_rect (in_frame->data[0], in_stride, out_frame->data[0],
              width, out_stride, height, start_x, start_y, b->width, b->height,
              b->led_id == LED_INVALID_ID ? 0xFF00FF : 0x00FFFF);
        }

        /* Draw a dot at the weighted center */
        draw_rgb_marker (out_frame->data[0], width, out_stride, height,
            round(b->x), round(b->y), 0, 0,
            b->led_id == LED_INVALID_ID ? 0xFF00FF : 0x00FFFF);
      }

      if (found_poses) {
        int d;

        for (d = 0; d < 3; d++) {
          const int colours[] = { 0xFF0000, 0x00FF00, 0x0000FF };
          int i;
          rift_pose_metrics score;

          if (!correspondence_search_have_pose (filter->cs, d,
                  &filter->pose, &score))
            continue;

          /* Draw a marker that says we think we saw this device */
          draw_rgb_filled_rect (out_frame->data[0], width, out_stride, height,
              16 * d, 0, 16, 16, colours[d]);

          if (filter->draw_led_markers) {
            /* Loop over the LED points a final time and draw markers
             * into the video over the top */

            /* Project HMD LEDs into the image again doh */
            rift_project_points (filter->leds[d].points,
                filter->leds[d].num_points, &filter->camera_matrix,
                filter->dist_coeffs, filter->is_cv1, &filter->pose,
                filter->led_out_points);

            for (i = 0; i < filter->leds[d].num_points; i++) {
              vec3f *p = filter->led_out_points + i;
              vec3f normal, position;
				      double facing_dot;
              int x = round (p->x);
              int y = round (p->y);

              oquatf_get_rotated (&filter->pose.orient,
                  &filter->leds[d].points[i].dir, &normal);

              oquatf_get_rotated (&filter->pose.orient,
                  &filter->leds[d].points[i].pos, &position);
				      ovec3f_add (&filter->pose.pos, &position, &position);
				      ovec3f_normalize_me (&position);

				      facing_dot = ovec3f_get_dot (&position, &normal);

              if (facing_dot < -0.25) {
                /* Camera facing */
                draw_rgb_marker (out_frame->data[0], width, out_stride, height,
                    x, y, 6, 6, colours[d]);
              } else {
                draw_rgb_marker (out_frame->data[0], width, out_stride, height,
                    x, y, 4, 4, 0x202000);
              }
            }
          }
        }
      }
    }

    blobwatch_release_observation (filter->bw, filter->bwobs, FALSE);
    filter->bwobs = NULL;
  }

  return GST_FLOW_OK;
}


static gboolean
ohmdriftsensor_init (GstPlugin * ohmdriftsensor)
{
  return gst_element_register (ohmdriftsensor, "ohmdriftsensor", GST_RANK_NONE,
      GST_TYPE_OHMDRIFTSENSOR);
}

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    ohmdplugin,
    "OpenHMD Rift Sensor filter",
    ohmdriftsensor_init,
    PACKAGE_VERSION, GST_LICENSE, GST_PACKAGE_NAME, GST_PACKAGE_ORIGIN)
