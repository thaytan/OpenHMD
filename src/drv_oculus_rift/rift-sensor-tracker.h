#include "rift.h"

typedef struct rift_tracker_ctx_s rift_tracker_ctx;

rift_tracker_ctx *rift_sensor_tracker_new (ohmd_context* ohmd_ctx,
		const uint8_t radio_id[5], rift_leds *leds);

void rift_sensor_tracker_new_exposure (rift_tracker_ctx *ctx, uint8_t led_pattern_phase);
void rift_sensor_tracker_free (rift_tracker_ctx *ctx);
