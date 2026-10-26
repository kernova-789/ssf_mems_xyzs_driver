/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once
#include "modbus/modbus_types.h"
#include "sensor_data.h"

struct ssf_mems_data;
struct ssf_mems_sensor_ops {
  int (*collect_sound)(struct ssf_mems_data *data); /* 可选：单批 4096 点。 */
  int (*read_features)(struct ssf_mems_data *data, unsigned int timeout_ms);
  int (*collect_raw)(struct ssf_mems_data *data, unsigned int duration_ms,
                     u32 samples);
};

/* 当前型号的配置含义；上层只使用规范化后的特征单位和配置编号。 */
struct ssf_mems_sensor_profile {
  const char *const *sampling_frequencies; /* Hz 文本，按采样率编码索引。 */
  size_t sampling_frequency_count;
  size_t sampling_length_count; /* 普通特征计算采样长度的编码数量。 */
  const unsigned int *baudrates; /* bit/s，按传感器波特率编码索引。 */
  size_t baudrate_count;
  enum ssf_mems_baudrate default_work_baudrate; /* 驱动首次连接的目标编码。 */
  u16 default_sampling_rate_index;
  u16 raw_max_sampling_rate_index; /* 型号支持的定长采样率上限。 */
  u32 raw_batch_max_samples; /* 驱动开放的批次长度上限，按 1024 的二次幂选择。 */
  const unsigned int *raw_min_baudrates; /* 可选：各采样率所需最低线速；NULL 不增加线速约束。 */
  size_t raw_min_baudrate_count;
  u32 raw_accel_scale_nano; /* 原始计数换算为 m/s² 的纳米比例。 */
  u8 raw_axis_order[3]; /* 手册XYZ对应线上三轴的位置，取值0..2。 */
  u16 feature_enable_mask;
  u8 feature_enable_shift;
};

/* 只读型号描述；每个设备独立持有指针，允许不同型号同时绑定。 */
struct ssf_mems_sensor {
  const char *name;
  const char *raw_name;
  const char *sound_name;
  u32 sound_sampling_frequency; /* Hz，独立于振动采样率；无能力时为 0。 */
  size_t feature_size; /* 对应 union 成员的有效大小。 */
  const struct ssf_reg_desc *regs;
  size_t reg_count;
  const struct ssf_block_cmd_desc *blocks;
  size_t block_count;
  struct ssf_mems_sensor_profile profile;
  const struct ssf_mems_sensor_ops *ops;
  bool supports_ota;
};
extern const struct ssf_mems_sensor ssf_mems_xyzs;
extern const struct ssf_mems_sensor ssf_mems_xyz;

bool ssf_mems_sensor_raw_samples_valid(const struct ssf_mems_sensor *sensor,
                                      u32 samples);

const struct ssf_reg_desc *ssf_mems_sensor_find_reg(
    const struct ssf_mems_sensor *sensor, u16 display_reg);
const struct ssf_reg_desc *ssf_mems_sensor_find_reg_by_name(
    const struct ssf_mems_sensor *sensor, const char *name);
const struct ssf_block_cmd_desc *ssf_mems_sensor_find_block(
    const struct ssf_mems_sensor *sensor, enum ssf_block_cmd_id id);
const struct ssf_reg_desc *ssf_mems_sensor_next_feature(
    const struct ssf_mems_sensor *sensor, u32 cursor, u32 end);
int ssf_mems_sensor_validate(const struct ssf_mems_sensor *sensor,
                             const char **bad_name);
/* 复用读取/转换机制；所有段成功后才原子替换缓存。 */
int ssf_mems_sensor_read_features(struct ssf_mems_data *data,
                                  unsigned int timeout_ms);
int ssf_mems_sensor_decode_features(const struct ssf_mems_sensor *sensor,
    const u16 *registers, size_t count, struct ssf_mems_sensor_data *result);
