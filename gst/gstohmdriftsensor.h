/* 
 * GStreamer
 * Copyright (C) 2006 Stefan Kost <ensonic@users.sf.net>
 * Copyright (C) 2019 Jan Schmidt <jan@centricular.com>
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
 
#ifndef __GST_OHMDRIFTSENSOR_H__
#define __GST_OHMDRIFTSENSOR_H__

#include <gst/gst.h>
#include <gst/video/gstvideofilter.h>

#include "rift.h"
#include "rift-sensor-blobwatch.h"
#include "rift-sensor-maths.h"
#include "rift-sensor-opencv.h"
#include "correspondence_search.h"
#include "led_search.h"

G_BEGIN_DECLS

#define GST_TYPE_OHMDRIFTSENSOR \
  (gst_ohmd_rift_sensor_get_type())
#define GST_OHMDRIFTSENSOR(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_OHMDRIFTSENSOR,GstOhmdRiftSensor))
#define GST_OHMDRIFTSENSOR_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_OHMDRIFTSENSOR,GstOhmdRiftSensorClass))
#define GST_IS_OHMDRIFTSENSOR(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_OHMDRIFTSENSOR))
#define GST_IS_OHMDRIFTSENSOR_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_OHMDRIFTSENSOR))

typedef struct _GstOhmdRiftSensor      GstOhmdRiftSensor;
typedef struct _GstOhmdRiftSensorClass GstOhmdRiftSensorClass;

struct _GstOhmdRiftSensor {
  GstVideoFilter element;
  gint fps_n, fps_d;

  int id;
  rift_leds leds[3];
  vec3f *led_out_points;

  GstClockTime last_pattern_time;
  uint8_t led_pattern_phase;
  struct blobwatch* bw;
  struct blobservation* bwobs;

  gboolean draw_side_by_side;
  gboolean draw_led_markers;
  gboolean draw_blob_squares;
  gboolean dump_blobs;

  dmat3 camera_matrix;
  double dist_coeffs[5];

  gboolean is_cv1;

  double angle;

  posef pose;

  led_search_model_t *led_models[3];
  correspondence_search_t *cs;
};

struct _GstOhmdRiftSensorClass {
  GstVideoFilterClass parent_class;
};

GType gst_ohmd_rift_sensor_get_type (void);

void rift_leds_init_dk2 (rift_leds *leds, uint8_t num_points);

G_END_DECLS

#endif /* __GST_OHMDRIFTSENSOR_H__ */
