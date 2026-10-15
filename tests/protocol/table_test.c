/* Validate production model metadata without a serial or upper-layer stub. */
#include "model/modbus_table.h"
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv) {
  const char *bad = "not_checked";
  int ret = ssf_mems_modbus_validate_tables(&bad);

  assert(argc == 2);
  if (strcmp(argv[1], "valid") == 0) {
    assert(ret == 0 && bad == NULL);
    assert(ssf_mems_modbus_validate_tables(NULL) == 0);
    assert(ssf_mems_sensor_field_desc(SSF_MEMS_FIELD_NONE) == NULL);
    assert(ssf_mems_sensor_field_desc(SSF_MEMS_FIELD_MAX) == NULL);
    assert(ssf_mems_sensor_field_desc(SSF_MEMS_FIELD_TEMPERATURE)->width == 2);
  } else {
    assert(ret == -EINVAL && bad != NULL && strcmp(bad, argv[1]) == 0);
    assert(ssf_mems_modbus_validate_tables(NULL) == -EINVAL);
  }
  puts("PASS model metadata validation");
  return 0;
}
