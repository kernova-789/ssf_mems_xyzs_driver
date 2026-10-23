// SPDX-License-Identifier: GPL-2.0-only
#include "sensor.h"
#include "model/modbus_table.h"
#include <linux/errno.h>
#include <linux/string.h>

/* 按手册地址查找寄存器。
 * 返回：表项指针，未找到时为 NULL。
 */
const struct ssf_reg_desc *
ssf_mems_sensor_find_reg(const struct ssf_mems_sensor *sensor, u16 display_reg) {
  size_t i;

  if (sensor == NULL)
    return NULL;
  for (i = 0; i < sensor->reg_count; i++) {
    if (sensor->regs[i].display_reg == display_reg)
      return &sensor->regs[i];
  }
  return NULL;
}

/* 按名称查找寄存器。
 * 返回：表项指针，名称无效或未找到时为 NULL。
 * 名称区分大小写。
 */
const struct ssf_reg_desc *
ssf_mems_sensor_find_reg_by_name(const struct ssf_mems_sensor *sensor, const char *name) {
  size_t i;

  if (name == NULL || name[0] == '\0')
    return NULL;
  if (sensor == NULL)
    return NULL;
  for (i = 0; i < sensor->reg_count; i++) {
    if (sensor->regs[i].name != NULL &&
        strcmp(sensor->regs[i].name, name) == 0)
      return &sensor->regs[i];
  }
  return NULL;
}

/* 按命令编号查找寄存器块。
 * 返回：块描述指针，未找到时为 NULL。
 */
const struct ssf_block_cmd_desc *
ssf_mems_sensor_find_block(const struct ssf_mems_sensor *sensor, enum ssf_block_cmd_id id) {
  size_t i;

  if (sensor == NULL)
    return NULL;
  for (i = 0; i < sensor->block_count; i++) {
    if (sensor->blocks[i].id == id)
      return &sensor->blocks[i];
  }
  return NULL;
}

/* 查找指定范围内的下一个可读特征。
 * 返回：寄存器指针，没有剩余特征时为 NULL。
 */
const struct ssf_reg_desc *ssf_mems_sensor_next_feature(
    const struct ssf_mems_sensor *sensor, u32 cursor, u32 end) {
  const struct ssf_reg_desc *next = NULL;
  size_t i;

  if (sensor == NULL)
    return NULL;
  for (i = 0; i < sensor->reg_count; i++) {
    const struct ssf_reg_desc *reg = &sensor->regs[i];

    if (reg->feature_field == SSF_MEMS_FIELD_NONE || (reg->access & SSF_REG_READ) == 0 ||
        reg->read_format != SSF_RX_HOLDING_REGS || reg->display_reg < cursor ||
        reg->display_reg >= end)
      continue;
    if (next == NULL || reg->display_reg < next->display_reg)
      next = reg;
  }
  return next;
}

/* 检查型号配置的编码范围和原始数据布局。
 * 返回：true 有效，false 存在空表、越界或非法布局。
 */
static bool ssf_mems_modbus_profile_valid(const struct ssf_mems_sensor *sensor) {
  const struct ssf_mems_sensor_profile *p = &sensor->profile;
  unsigned int mask;
  size_t i;

  if (p->sampling_frequencies == NULL || p->sampling_frequency_count == 0 ||
      p->sampling_frequency_count > 65536U || p->baudrates == NULL ||
      p->sampling_length_count == 0 || p->sampling_length_count > 65536U ||
      p->baudrate_count == 0 || p->baudrate_count > SSF_MEMS_BAUDRATE_MAX ||
      (unsigned int)p->default_work_baudrate >= p->baudrate_count ||
      p->default_sampling_rate_index >= p->sampling_frequency_count ||
      p->raw_max_sampling_rate_index >= p->sampling_frequency_count ||
      p->raw_accel_scale_nano == 0 || p->raw_accel_scale_nano >= 1000000000U ||
      p->feature_enable_shift >= 16)
    return false;
  if ((p->raw_min_baudrates == NULL && p->raw_min_baudrate_count != 0) ||
      (p->raw_min_baudrates != NULL &&
       p->raw_min_baudrate_count != p->sampling_frequency_count))
    return false;
  for (i = 0; i < p->sampling_frequency_count; i++)
    if (p->sampling_frequencies[i] == NULL || p->sampling_frequencies[i][0] == '\0')
      return false;
  for (i = 0; i < p->raw_min_baudrate_count; i++)
    if (p->raw_min_baudrates[i] == 0)
      return false;
  for (i = 0; i < p->baudrate_count; i++)
    if (p->baudrates[i] == 0 || p->baudrates[i] > 0x7fffffffU)
      return false;
  for (i = 0; i < ARRAY_SIZE(p->raw_axis_order); i++)
    if (p->raw_axis_order[i] >= ARRAY_SIZE(p->raw_axis_order))
      return false;
  if (p->raw_axis_order[0] == p->raw_axis_order[1] ||
      p->raw_axis_order[0] == p->raw_axis_order[2] ||
      p->raw_axis_order[1] == p->raw_axis_order[2])
    return false;
  mask = p->feature_enable_mask >> p->feature_enable_shift;
  return (mask << p->feature_enable_shift) == p->feature_enable_mask &&
         (mask & (mask + 1U)) == 0;
}

/* 检查一个寄存器的权限和特征映射。
 * 返回：true 有效，false 表项字段无效。
 */
static bool ssf_mems_modbus_reg_valid(const struct ssf_reg_desc *reg) {
  const struct ssf_mems_sensor_field_desc *field;

  if (reg->name == NULL || reg->name[0] == '\0' || reg->access == 0 ||
      (reg->access & ~(SSF_REG_READ | SSF_REG_WRITE)) != 0 ||
      (unsigned int)reg->type > SSF_REG_U32_LOW ||
      (unsigned int)reg->read_format > SSF_RX_RAW_XYZ ||
      (reg->write_single && (reg->access & SSF_REG_WRITE) == 0))
    return false;
  if (reg->feature_field == SSF_MEMS_FIELD_NONE)
    return true;
  field = &reg->field;
  return reg->feature_field > SSF_MEMS_FIELD_NONE &&
         reg->feature_field < SSF_MEMS_FIELD_MAX &&
         (unsigned int)field->scale <= SSF_MEMS_FEATURE_SCALE_UNITY &&
         (reg->type == SSF_REG_U16 || reg->type == SSF_REG_S16) &&
         (reg->access & SSF_REG_READ) != 0 &&
         reg->read_format == SSF_RX_HOLDING_REGS && reg->value_mask != 0 &&
         field->offset < sizeof(struct ssf_mems_sensor_data) &&
         field->width <= sizeof(struct ssf_mems_sensor_data) - field->offset &&
         (field->width == sizeof(u16) ||
          (field->width == sizeof(u8) && reg->value_mask <= 0xffU));
}

/* 检查长度是否在当前型号开放的二次幂批次范围内。 */
bool ssf_mems_sensor_raw_samples_valid(const struct ssf_mems_sensor *sensor,
                                      u32 samples) {
  return sensor != NULL && samples >= 1024 && samples <= 32768 &&
         samples <= sensor->profile.raw_batch_max_samples &&
         (samples & (samples - 1U)) == 0;
}

/* 检查型号表的编码、布局和名称；bad_name 返回出错项。允许稀疏表。 */
int ssf_mems_sensor_validate(const struct ssf_mems_sensor *sensor, const char **bad_name) {
  const char *bad = "sensor_profile";
  size_t i, j;

  if (sensor == NULL || sensor->regs == NULL || sensor->blocks == NULL ||
      sensor->feature_size == 0 ||
      sensor->feature_size > sizeof(struct ssf_mems_sensor_data) ||
      sensor->ops == NULL || sensor->ops->read_features == NULL ||
      sensor->ops->collect_raw == NULL || !ssf_mems_modbus_profile_valid(sensor) ||
      !ssf_mems_sensor_raw_samples_valid(sensor, sensor->profile.raw_batch_max_samples))
    goto invalid;
  for (i = 0; i < sensor->reg_count; i++) {
    const struct ssf_reg_desc *reg = &sensor->regs[i];

    bad = reg->name && reg->name[0] ? reg->name : "unnamed_register";
    if (!ssf_mems_modbus_reg_valid(reg) ||
        (reg->feature_field != SSF_MEMS_FIELD_NONE &&
         (reg->field.offset >= sensor->feature_size ||
          reg->field.width > sensor->feature_size - reg->field.offset)))
      goto invalid;
    for (j = 0; j < i; j++) {
      const struct ssf_reg_desc *other = &sensor->regs[j];

      if (strcmp(reg->name, other->name) == 0 ||
          reg->display_reg == other->display_reg ||
          reg->protocol_addr == other->protocol_addr ||
          (reg->feature_field != SSF_MEMS_FIELD_NONE &&
           (reg->feature_field == other->feature_field ||
            (other->feature_field != SSF_MEMS_FIELD_NONE &&
             reg->field.offset < other->field.offset + other->field.width &&
             other->field.offset < reg->field.offset + reg->field.width))))
        goto invalid;
    }
  }
  for (i = 0; i < sensor->block_count; i++) {
    const struct ssf_block_cmd_desc *block = &sensor->blocks[i];
    const struct ssf_modbus_frame_desc *frame =
        ssf_mems_modbus_find_frame(block->function);

    bad = block->name && block->name[0] ? block->name : "unnamed_block";
    if (block->name == NULL || block->name[0] == '\0' || frame == NULL ||
        block->reg_count == 0 ||
        (u32)block->start_display_reg + block->reg_count > 65536U ||
        block->rx_format != frame->rx_format ||
        (unsigned int)block->id > SSF_BLOCK_WRITE_WORK_PARAMETERS ||
        (unsigned int)block->decode_kind > SSF_DECODE_FEATURES ||
        block->direction != (frame->tx_format == SSF_TX_READ_REGS ?
                                 SSF_CMD_READ : SSF_CMD_WRITE) ||
        (block->sparse_read && frame->tx_format != SSF_TX_READ_REGS))
      goto invalid;
    for (j = 0; j < i; j++)
      if (block->id == sensor->blocks[j].id ||
          strcmp(block->name, sensor->blocks[j].name) == 0)
        goto invalid;
  }
  if (bad_name != NULL)
    *bad_name = NULL;
  return 0;
invalid:
  if (bad_name != NULL)
    *bad_name = bad;
  return -EINVAL;
}


const struct ssf_mems_sensor_field_desc *ssf_mems_sensor_field_desc(
    const struct ssf_mems_sensor *sensor, enum ssf_mems_sensor_field field) {
  size_t i;

  if (sensor == NULL || field <= SSF_MEMS_FIELD_NONE || field >= SSF_MEMS_FIELD_MAX)
    return NULL;
  for (i = 0; i < sensor->reg_count; i++)
    if (sensor->regs[i].feature_field == field)
      return &sensor->regs[i].field;
  return NULL;
}
