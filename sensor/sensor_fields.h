/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include "sensor_data.h"

#include <linux/kernel.h>

/* 描述规范化缓存值到公开物理单位的比例，不依赖任何 IIO 类型。 */
enum ssf_mems_feature_scale {
  SSF_MEMS_FEATURE_SCALE_ACCEL, /* g ×100；公开值为 m/s²。 */
  SSF_MEMS_FEATURE_SCALE_VELOCITY, /* mm/s ×100；公开值为 m/s。 */
  SSF_MEMS_FEATURE_SCALE_TEMPERATURE, /* ℃ ×100；公开值为毫摄氏度。 */
  SSF_MEMS_FEATURE_SCALE_CENTI, /* 规范量 ×100；乘 0.01。 */
  SSF_MEMS_FEATURE_SCALE_DECI, /* 规范量 ×10；乘 0.1。 */
  SSF_MEMS_FEATURE_SCALE_DISPLACEMENT, /* µm；公开值为 m。 */
  SSF_MEMS_FEATURE_SCALE_UNITY, /* 百分数或标志等原值；乘 1。 */
};

/* 通用特征语义编号；前 29 项保持既有 XYZS IIO ABI 顺序。 */
#define SSF_MEMS_FEATURE_FIELDS(X) \
  X(X_HIGH_FREQ_ACC_RMS) \
  X(X_LOW_FREQ_VELOCITY_RMS) \
  X(Y_HIGH_FREQ_ACC_RMS) \
  X(Y_LOW_FREQ_VELOCITY_RMS) \
  X(Z_HIGH_FREQ_ACC_RMS) \
  X(Z_LOW_FREQ_VELOCITY_RMS) \
  X(TEMPERATURE) \
  X(X_ACC_PEAK_TO_PEAK) \
  X(Y_ACC_PEAK_TO_PEAK) \
  X(Z_ACC_PEAK_TO_PEAK) \
  X(X_ACC_PEAK) \
  X(Y_ACC_PEAK) \
  X(Z_ACC_PEAK) \
  X(X_ACC_RMS) \
  X(Y_ACC_RMS) \
  X(Z_ACC_RMS) \
  X(X_KURTOSIS) \
  X(Y_KURTOSIS) \
  X(Z_KURTOSIS) \
  X(X_VELOCITY_RMS) \
  X(Y_VELOCITY_RMS) \
  X(Z_VELOCITY_RMS) \
  X(SOUND_RMS) \
  X(SOUND_PEAK) \
  X(SOUND_PEAK_TO_PEAK) \
  X(ZERO_CROSSING_RATE) \
  X(SPECTRAL_CENTROID) \
  X(SPECTRAL_FLUX) \
  X(STARTUP_FLAGS) \
  X(X_DISPLACEMENT_PEAK_TO_PEAK) \
  X(Y_DISPLACEMENT_PEAK_TO_PEAK) \
  X(Z_DISPLACEMENT_PEAK_TO_PEAK)

enum ssf_mems_sensor_field {
  SSF_MEMS_FIELD_NONE,
#define SSF_MEMS_FIELD_ID(_name) SSF_MEMS_FIELD_##_name,
  SSF_MEMS_FEATURE_FIELDS(SSF_MEMS_FIELD_ID)
#undef SSF_MEMS_FIELD_ID
  SSF_MEMS_FIELD_MAX,
};

struct ssf_mems_sensor_field_desc {
  size_t offset;
  u8 width;
  enum ssf_mems_feature_scale scale;
};
struct ssf_mems_sensor;
const struct ssf_mems_sensor_field_desc *ssf_mems_sensor_field_desc(
    const struct ssf_mems_sensor *sensor, enum ssf_mems_sensor_field field);

/* 直接嵌入寄存器表的偏移映射，成员类型或布局改变时自动同步。 */
#define SSF_MEMS_FEATURE_MEMBER(_member, _scale) \
  .field = { .offset = offsetof(struct ssf_mems_sensor_data, _member), \
             .width = sizeof(((struct ssf_mems_sensor_data *)0)->_member), \
             .scale = SSF_MEMS_FEATURE_SCALE_##_scale }
