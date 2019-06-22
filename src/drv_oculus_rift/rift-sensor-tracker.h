#include "rift.h"

typedef struct rift_sensor_ctx_s rift_sensor_ctx;

int rift_sensor_tracker_init (rift_sensor_ctx **ctx, const uint8_t radio_id[5],
    rift_leds *leds);
void rift_sensor_tracker_new_exposure (rift_sensor_ctx *ctx, uint8_t led_pattern_phase);
void rift_sensor_tracker_free (rift_sensor_ctx *ctx);
