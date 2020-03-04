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

#define DUMP_BLOBS 0
#define KALMAN_FILTER 1

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

static GstCaps *
gst_ohmd_rift_sensor_transform_caps (GstBaseTransform * btrans,
    GstPadDirection direction, GstCaps * caps, GstCaps * filter)
{
  GstCaps *tmp, *tmp2;
  GstCaps *result;

  /* Get all possible caps that we can transform to */
  tmp = gst_ohmd_rift_sensor_caps_remove_format_info (caps);

  if (filter) {
    tmp2 = gst_caps_intersect_full (filter, tmp, GST_CAPS_INTERSECT_FIRST);
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
  if (filter->pose_filter)
    kalman_pose_free (filter->pose_filter);

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

  if (filter->pose_filter)
    kalman_pose_free (filter->pose_filter);
  filter->pose_filter = kalman_pose_new (NULL);
  memset (&filter->pose_pos, 0, sizeof (filter->pose_pos));
  memset (&filter->pose_orient, 0, sizeof (filter->pose_orient));

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
}

static void
gst_ohmd_rift_sensor_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  //GstOhmdRiftSensor *filter = GST_OHMDRIFTSENSOR (object);

  switch (prop_id) {
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_ohmd_rift_sensor_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  //GstOhmdRiftSensor *filter = GST_OHMDRIFTSENSOR (object);

  switch (prop_id) {
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
  filter->bw = blobwatch_new (in_info->width, in_info->height);
  blobwatch_set_flicker (filter->bw, false);
  return TRUE;
}

static void
draw_rgb_marker (guint8 * pixels, gint width, gint stride, gint height,
    gint x_pos, gint y_pos, gint mark_width, gint mark_height, guint32 colour)
{
  gint x, y;
  gint min_x = MAX (0, x_pos - mark_width / 2);
  gint max_x = MIN (width, x_pos + mark_width / 2);
  gint min_y = MAX (0, y_pos - mark_height / 2);
  gint max_y = MIN (height, y_pos + mark_height / 2);

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
paint_rgb_rect (guint8 *in_pixels, gint in_stride, guint8 * pixels, gint width, gint stride, gint height,
    gint start_x, gint start_y, gint box_width, gint box_height, guint32 mask_colour)
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
      *dest++ = (r * src[0]) >> 8;
      *dest++ = (g * src[0]) >> 8;
      *dest++ = (b * src[0]) >> 8;
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
#if 1
  correspondence_search_set_blobs (filter->cs, bwobs->blobs, bwobs->num_blobs);
  if (correspondence_search_find_pose (filter->cs) > 0)
    ret = TRUE;

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

#if KALMAN_FILTER
    kalman_pose_update (filter->pose_filter, ts, &trans, &rot);
    kalman_pose_get_estimated (filter->pose_filter, &filter->pose_pos,
        &filter->pose_orient);
#else
    filter->pose_pos = trans;
    filter->pose_orient = rot;
#endif

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

#if DUMP_BLOBS
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
#endif

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

    dest += out_stride;
    src += in_stride;
  }

  if (filter->bwobs && filter->bwobs->num_blobs > 0) {
    struct blob *sorted_blobs[MAX_BLOBS_PER_FRAME];

    GST_INFO_OBJECT (filter, "Sensor %d phase %d Blobs: %d", filter->id,
        led_pattern_phase, filter->bwobs->num_blobs);
#if DUMP_BLOBS
    g_print ("Sensor %d phase %d Blobs: %d\n", filter->id, led_pattern_phase,
        filter->bwobs->num_blobs);
#endif

    /* Copy pointers into the sorted blobs array */
    for (int index = 0; index < filter->bwobs->num_blobs; index++)
      sorted_blobs[index] = filter->bwobs->blobs + index;

#if DUMP_BLOBS
    qsort (sorted_blobs, filter->bwobs->num_blobs, sizeof (struct blob *),
        compare_blobs);
#endif

    bool found_poses = tracker_process_blobs (filter, in_ts);

    if (found_poses) {
      int d;

      for (d = 0; d < 3; d++) {
        const int colours[] = { 0xFF0000, 0x00FF00, 0x0000FF };
        int i;
        rift_pose_metrics score;

        if (!correspondence_search_have_pose (filter->cs, d,
                &filter->pose_orient, &filter->pose_pos, &score))
          continue;

        /* Draw a marker that says we think we saw this device */
        draw_rgb_filled_rect (out_frame->data[0], width, out_stride, height,
            16 * d, 0, 16, 16, colours[d]);

        /* Project HMD LEDs into the image */
        rift_project_points (filter->leds[d].points, filter->leds[d].num_points,
            &filter->camera_matrix, filter->dist_coeffs, filter->is_cv1,
            &filter->pose_orient, &filter->pose_pos, filter->led_out_points);

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

          for (i = 0; i < filter->leds[d].num_points; i++) {
            vec3f *p = filter->led_out_points + i;
            vec3f facing;
            int x = round (p->x);
            int y = round (p->y);
  
            oquatf_get_rotated (&filter->pose_orient,
                &filter->leds[d].points[i].dir, &facing);
  
            if (facing.z < -0.25) {
              /* Camera facing */
              struct blob *b = blobwatch_find_blob_at (filter->bw, x, y);
              if (b != NULL && b->led_id != i) {
                /* Found a blob! */
                b->led_id = LED_MAKE_ID (d, i);
                g_print ("Marking LED %d/%d at %d,%d now %u\n", d, i,
                    x, y, b->led_id);
              }
            }
          }
        }
      }

      /* Draw the blobs in the video */
      for (int index = 0; index < filter->bwobs->num_blobs; index++) {
        struct blob *b = sorted_blobs[index];
        gint start_x, start_y, w, h;

#if DUMP_BLOBS
        g_print
            ("Sensor %d Blob)[%d]: %d,%d %dx%d (age %d) id %d pattern %x (unchanged %u)\n",
            filter->id, index, b->x, b->y, b->width, b->height, b->age,
            b->led_id, b->pattern, b->pattern_age);
#endif

        start_x = b->x - b->width / 2;
        start_y = b->y - b->height / 2;
        w = b->width;
        h = b->height;
        clamp_rect (&start_x, &start_y, &w, &h, width, height);

        /* Paint unknown blobs purple, known blobs green */
        paint_rgb_rect (in_frame->data[0], in_stride, out_frame->data[0], width, out_stride, height, start_x,
            start_y, b->width, b->height,
            b->led_id == LED_INVALID_ID ? 0xFF00FF : 0x00FF00);
      }
#if DUMP_BLOBS
      g_print ("\n");
#endif

      /* Loop over the LED points a final time and draw markers
       * into the video over the top */
      for (d = 0; d < 3; d++) {
        const int colours[] = { 0xFF0000, 0x00FF00, 0x0000FF };
        int i;
        rift_pose_metrics score;

        if (!correspondence_search_have_pose (filter->cs, d,
                &filter->pose_orient, &filter->pose_pos, &score))
          continue;

        /* Refine the pose with PnP */
        estimate_initial_pose (filter->bwobs->blobs, filter->bwobs->num_blobs,
          filter->leds[d].points, filter->leds[d].num_points, &filter->camera_matrix,
          filter->dist_coeffs, filter->is_cv1,
          &filter->pose_orient, &filter->pose_pos, NULL, NULL, true);

        /* Project HMD LEDs into the image again doh */
        rift_project_points (filter->leds[d].points, filter->leds[d].num_points,
            &filter->camera_matrix, filter->dist_coeffs, filter->is_cv1,
            &filter->pose_orient, &filter->pose_pos, filter->led_out_points);

        for (i = 0; i < filter->leds[d].num_points; i++) {
          vec3f *p = filter->led_out_points + i;
          vec3f facing;
          int x = round (p->x);
          int y = round (p->y);
    
          oquatf_get_rotated (&filter->pose_orient,
              &filter->leds[d].points[i].dir, &facing);
    
          if (facing.z < -0.25) {
            /* Camera facing */
            draw_rgb_marker (out_frame->data[0], width, out_stride, height, x,
                y, 8, 8, colours[d]);
          } else {
            draw_rgb_marker (out_frame->data[0], width, out_stride, height, x,
                y, 8, 8, 0x202000);
          }
        }
      }
    }
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
