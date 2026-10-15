/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include "model/sensor_data.h"

#include <linux/kernel.h>

/* 描述规范化缓存值到公开物理单位的比例，不依赖任何 IIO 类型。 */
enum ssf_mems_feature_scale {
  SSF_MEMS_FEATURE_SCALE_ACCEL, /* g ×100；公开值为 m/s²。 */
  SSF_MEMS_FEATURE_SCALE_VELOCITY, /* mm/s ×100；公开值为 m/s。 */
  SSF_MEMS_FEATURE_SCALE_TEMPERATURE, /* ℃ ×100；公开值为毫摄氏度。 */
  SSF_MEMS_FEATURE_SCALE_CENTI, /* 规范量 ×100；乘 0.01。 */
  SSF_MEMS_FEATURE_SCALE_DECI, /* 规范量 ×10；乘 0.1。 */
  SSF_MEMS_FEATURE_SCALE_UNITY, /* 百分数或标志等原值；乘 1。 */
};

/* 唯一的特征字段清单：名称、缓存成员、规范单位比例。顺序同时保持 IIO ABI。 */
#define SSF_MEMS_SENSOR_FIELDS(X)                                            \
  X(X_HIGH_FREQ_ACC_RMS, x.high_freq_acc_rms_x100, ACCEL)                    \
  X(X_LOW_FREQ_VELOCITY_RMS, x.low_freq_velocity_rms_x100, VELOCITY)          \
  X(Y_HIGH_FREQ_ACC_RMS, y.high_freq_acc_rms_x100, ACCEL)                    \
  X(Y_LOW_FREQ_VELOCITY_RMS, y.low_freq_velocity_rms_x100, VELOCITY)          \
  X(Z_HIGH_FREQ_ACC_RMS, z.high_freq_acc_rms_x100, ACCEL)                    \
  X(Z_LOW_FREQ_VELOCITY_RMS, z.low_freq_velocity_rms_x100, VELOCITY)          \
  X(TEMPERATURE, temperature_x100, TEMPERATURE)                              \
  X(X_ACC_PEAK_TO_PEAK, x.acc_peak_to_peak_x100, ACCEL)                      \
  X(Y_ACC_PEAK_TO_PEAK, y.acc_peak_to_peak_x100, ACCEL)                      \
  X(Z_ACC_PEAK_TO_PEAK, z.acc_peak_to_peak_x100, ACCEL)                      \
  X(X_ACC_PEAK, x.acc_peak_x100, ACCEL)                                     \
  X(Y_ACC_PEAK, y.acc_peak_x100, ACCEL)                                     \
  X(Z_ACC_PEAK, z.acc_peak_x100, ACCEL)                                     \
  X(X_ACC_RMS, x.acc_rms_x100, ACCEL)                                       \
  X(Y_ACC_RMS, y.acc_rms_x100, ACCEL)                                       \
  X(Z_ACC_RMS, z.acc_rms_x100, ACCEL)                                       \
  X(X_KURTOSIS, x.kurtosis_x100, CENTI)                                    \
  X(Y_KURTOSIS, y.kurtosis_x100, CENTI)                                    \
  X(Z_KURTOSIS, z.kurtosis_x100, CENTI)                                    \
  X(X_VELOCITY_RMS, x.velocity_rms_x100, VELOCITY)                           \
  X(Y_VELOCITY_RMS, y.velocity_rms_x100, VELOCITY)                           \
  X(Z_VELOCITY_RMS, z.velocity_rms_x100, VELOCITY)                           \
  X(SOUND_RMS, sound.rms_db_x100, CENTI)                                   \
  X(SOUND_PEAK, sound.peak_db_x100, CENTI)                                 \
  X(SOUND_PEAK_TO_PEAK, sound.peak_to_peak_db_x100, CENTI)                   \
  X(ZERO_CROSSING_RATE, zero_crossing_rate_percent, UNITY)                  \
  X(SPECTRAL_CENTROID, spectral_centroid_hz_x10, DECI)                      \
  X(SPECTRAL_FLUX, spectral_flux_x100, CENTI)                              \
  X(STARTUP_FLAGS, startup_flags, UNITY)

enum ssf_mems_sensor_field {
  SSF_MEMS_FIELD_NONE, /* 寄存器不映射到特征缓存。 */
#define SSF_MEMS_FIELD_ID(_name, _member, _scale) SSF_MEMS_FIELD_##_name,
  SSF_MEMS_SENSOR_FIELDS(SSF_MEMS_FIELD_ID)
#undef SSF_MEMS_FIELD_ID
  SSF_MEMS_FIELD_MAX,
};

/* 缓存字段布局由成员类型生成，协议解码和上层读取共用同一份描述。 */
struct ssf_mems_sensor_field_desc {
  size_t offset;
  u8 width;
  enum ssf_mems_feature_scale scale;
};

extern const struct ssf_mems_sensor_field_desc
    ssf_mems_sensor_fields[SSF_MEMS_FIELD_MAX];

/* 查找一个规范特征字段的布局和单位比例。
 * 返回：字段描述，NONE 或无效编号返回 NULL。
 */
static inline const struct ssf_mems_sensor_field_desc *
ssf_mems_sensor_field_desc(enum ssf_mems_sensor_field field) {
  if (field <= SSF_MEMS_FIELD_NONE || field >= SSF_MEMS_FIELD_MAX)
    return NULL;
  return &ssf_mems_sensor_fields[field];
}
