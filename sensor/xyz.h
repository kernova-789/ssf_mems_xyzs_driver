/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once
#include <linux/types.h>

/* XYZ V2.4 每轴六个特征；温度统一到摄氏度 ×100。 */
struct ssf_mems_xyz_axis_features {
  u16 acc_peak_x100;
  u16 acc_peak_to_peak_x100;
  u16 acc_rms_x100;
  u16 kurtosis_x100;
  u16 velocity_rms_x100;
  u16 displacement_peak_to_peak_um;
};
struct ssf_mems_xyz_features {
  struct ssf_mems_xyz_axis_features x, y, z;
  s16 temperature_x100;
};
