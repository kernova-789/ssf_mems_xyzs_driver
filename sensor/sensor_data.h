/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once
#include <linux/types.h>
#include "xyz.h"
#include "xyzs.h"
#define SSF_MEMS_CENTI_SCALE 100U
#define SSF_MEMS_DECI_SCALE 10U

/* 定长批次输出的一个同步 XYZ 样本，保留有符号 ADC 计数。 */
struct ssf_mems_raw_sample {
  s16 xyz[3]; /* 手册定义的 X、Y、Z，满量程 ±16 g。 */
  u32 packet_sequence; /* 传感器包号；每包包含 64 个样本，可用于识别丢包。 */
  u16 sample_index; /* 样本在包内的位置，范围 0..63。 */
};

/* 型号在 probe 时确定，此后只访问对应的 union 成员。 */
struct ssf_mems_sensor_data {
  union {
    struct ssf_mems_xyzs_features xyzs;
    struct ssf_mems_xyz_features xyz;
  };
};
