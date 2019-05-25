
typedef struct rift_sensor_ctx_s rift_sensor_ctx;

int rift_sensor_tracker_init (rift_sensor_ctx **ctx, const uint8_t radio_id[5]);
void rift_sensor_tracker_free (rift_sensor_ctx *ctx);
