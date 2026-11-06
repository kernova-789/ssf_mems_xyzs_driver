/* Metadata-only harness; no bus implementation is linked. */
#include "sensor.h"
#include <stdlib.h>
int ssf_mems_sensor_read_features(struct ssf_mems_data *data, unsigned int timeout_ms) {
  (void)data; (void)timeout_ms; abort();
}
int ssf_mems_raw_collect_batch(struct ssf_mems_data *data, u32 length, bool reset_stats) {
  (void)data; (void)length; (void)reset_stats; abort();
}

int ssf_mems_sound_collect_finite(struct ssf_mems_data *data) {
  (void)data; abort();
}
