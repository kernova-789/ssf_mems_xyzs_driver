#pragma once

#include <linux/types.h>

struct serdev_device;
struct ssf_mems_xyzs_data;

#define SSF_MEMS_FEATURE_FIRST_REG 40001U
#define SSF_MEMS_FEATURE_REG_COUNT 29U
#define SSF_MEMS_FEATURE_SCALE 100U

/*
 * 所有特征值均以定点整数形式保存。以 _x100 结尾的字段所存储的数值，
 * 是传感器手册中所述物理量的 100 倍。保留原始定点表示既可避免在
 * 内核中进行浮点运算，也能完整保留寄存器中的精确数值。
 */
struct ssf_mems_axis_features {
  u16 high_freq_acc_rms_x100; /* mm/s，沿用寄存器表中的表述 */
  u16 low_freq_velocity_rms_x100; /* mm/s */
  u16 acc_peak_to_peak_x100; /* g */
  u16 acc_peak_x100;         /* g */
  u16 acc_rms_x100;          /* g */
  u16 kurtosis_x100;
  u16 velocity_rms_x100; /* mm/s */
};

struct ssf_mems_sound_features {
  u16 rms_x100;          /* Pa */
  u16 peak_x100;         /* Pa */
  u16 peak_to_peak_x100; /* Pa */
};

struct ssf_mems_sensor_data {
  struct ssf_mems_axis_features x;
  struct ssf_mems_axis_features y;
  struct ssf_mems_axis_features z;

  u16 temperature_x100; /* 摄氏度 */
  struct ssf_mems_sound_features sound;
  u16 zero_crossing_rate_x100; /* 百分比 */
  u16 spectral_centroid_x100;  /* Hz */
  u16 spectral_flux_x100;      /* Pa^2 */

  /* 位 0、位 1 和位 2 分别表示 X、Y 和 Z 轴的启动状态。 */
  u8 startup_flags;
};

/* 将寄存器 40001～40029 的值转换为传感器特征字段。 */
int ssf_mems_modbus_decode_features(const u16 *registers, size_t count,
                                    struct ssf_mems_sensor_data *result);

/* 读取全部特征寄存器，并以原子方式替换 data->sensor_data。 */
int ssf_mems_modbus_read_features(struct serdev_device *serdev,
                                  unsigned int timeout_ms);

/* 复制最近一次解析的值；若尚未读取过数据，则返回 -ENODATA。 */
int ssf_mems_modbus_get_features(struct serdev_device *serdev,
                                 struct ssf_mems_sensor_data *result);
