/* Metadata-only harness; no bus implementation is linked. */
#include "sensor.h"
#include <stdlib.h>
int ssf_mems_sensor_read_features(struct ssf_mems_data *data, unsigned int timeout_ms) {
  (void)data; (void)timeout_ms; abort();
}
int ssf_mems_raw_collect_finite(struct ssf_mems_data *data, unsigned int duration_ms, u32 length) {
  (void)data; (void)duration_ms; (void)length; abort();
}

int ssf_mems_sound_collect_finite(struct ssf_mems_data *data) {
  (void)data; abort();
}
