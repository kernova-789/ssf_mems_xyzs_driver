/* Validate production model metadata without a serial or upper-layer stub. */
#include "model/modbus_table.h"
#include "sensor.h"
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv) {
  const char *bad = "not_checked";
  int ret = ssf_mems_sensor_validate(&ssf_mems_xyzs, &bad);

  assert(argc == 2);
  if (strcmp(argv[1], "valid") == 0) {
    assert(ret == 0 && bad == NULL);
    assert(ssf_mems_sensor_validate(&ssf_mems_xyzs, NULL) == 0);
    assert(ssf_mems_sensor_field_desc(&ssf_mems_xyzs, SSF_MEMS_FIELD_NONE) == NULL);
    assert(ssf_mems_sensor_field_desc(&ssf_mems_xyzs, SSF_MEMS_FIELD_MAX) == NULL);
    assert(ssf_mems_sensor_field_desc(&ssf_mems_xyzs, SSF_MEMS_FIELD_TEMPERATURE)->width == 2);
  } else {
    assert(ret == -EINVAL && bad != NULL && strcmp(bad, argv[1]) == 0);
    assert(ssf_mems_sensor_validate(&ssf_mems_xyzs, NULL) == -EINVAL);
  }
  puts("PASS model metadata validation");
  return 0;
}
