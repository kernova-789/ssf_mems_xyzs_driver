/* A replacement model table must determine the acquisition work target. */
#include "core.h"
#include "ssf_mems_acquisition.h"

#include <assert.h>

unsigned long jiffies;

int main(void) {
  struct serdev_device serial = {0};
  struct ssf_mems_data sensor = {.serdev = &serial, .sensor = &ssf_mems_xyzs, .sensor_ops = ssf_mems_xyzs.ops};

  assert(ssf_mems_sensor_validate(&ssf_mems_xyzs, NULL) == 0);
  assert(ssf_mems_xyzs.profile.default_work_baudrate == SSF_MEMS_BAUDRATE_115200);
  assert(ssf_mems_acquisition_init(&sensor) == 0);
  assert(sensor.acquisition.target_baudrate == ssf_mems_xyzs.profile.default_work_baudrate);
  assert(ssf_mems_baudrate_to_value(&ssf_mems_xyzs, sensor.acquisition.target_baudrate) == 115200);
  assert(sensor.acquisition.target_sampling_rate_index ==
         ssf_mems_xyzs.profile.default_sampling_rate_index);
  struct ssf_mems_data xyz = {.serdev = &serial, .sensor = &ssf_mems_xyz,
                              .sensor_ops = ssf_mems_xyz.ops};
  struct ssf_mems_sensor_data first = {0}, second = {0};
  struct ssf_mems_acquisition_policy policy;
  assert(ssf_mems_sensor_validate(&ssf_mems_xyz, NULL) == 0);
  assert(ssf_mems_acquisition_init(&xyz) == 0);
  assert(xyz.acquisition.target_baudrate == SSF_MEMS_BAUDRATE_128000);
  assert(sensor.acquisition.target_baudrate == SSF_MEMS_BAUDRATE_115200);
  ssf_mems_acquisition_policy_init(&policy, &ssf_mems_xyz, 0);
  policy.config.mode = SSF_MEMS_POLL_ADAPTIVE;
  ssf_mems_acquisition_policy_sample(&policy, &first, 5, 0);
  memset((u8 *)&second + sizeof(second.xyz), 0xa5,
         sizeof(second) - sizeof(second.xyz));
  ssf_mems_acquisition_policy_sample(&policy, &second, 5, 1);
  assert(policy.comparisons == 1 && policy.repeats == 1);
  second.xyz.z.displacement_peak_to_peak_um = 1;
  ssf_mems_acquisition_policy_sample(&policy, &second, 5, 2);
  assert(policy.comparisons == 2 && policy.repeats == 1);
  puts("PASS per-instance model targets and XYZ adaptive field comparison");
  return 0;
}
