/* A replacement model table must determine the acquisition work target. */
#include "core.h"
#include "ssf_mems_acquisition.h"

#include <assert.h>

unsigned long jiffies;

int main(void) {
  struct serdev_device serial = {0};
  struct ssf_mems_xyzs_data sensor = {.serdev = &serial};

  assert(ssf_mems_modbus_validate_tables(NULL) == 0);
  assert(ssf_mems_sensor_profile.default_work_baudrate == SSF_MEMS_BAUDRATE_115200);
  assert(ssf_mems_acquisition_init(&sensor) == 0);
  assert(sensor.acquisition.target_baudrate == ssf_mems_sensor_profile.default_work_baudrate);
  assert(ssf_mems_baudrate_to_value(sensor.acquisition.target_baudrate) == 115200);
  assert(sensor.acquisition.target_sampling_rate_index ==
         ssf_mems_sensor_profile.default_sampling_rate_index);
  puts("PASS alternate model table sets acquisition work baudrate");
  return 0;
}
