// SPDX-License-Identifier: GPL-2.0-only
#define SSF_MEMS_DEFINE_MODBUS_TABLE
#include "model/modbus_table.h"

#include <linux/errno.h>

/* 所有模块共享的字段布局，由规范字段清单生成。 */
#define SSF_MEMS_FIELD_DESC(_name, _member, _scale) \
  [SSF_MEMS_FIELD_##_name] = { \
    .offset = offsetof(struct ssf_mems_sensor_data, _member), \
    .width = sizeof(((struct ssf_mems_sensor_data *)0)->_member), \
    .scale = SSF_MEMS_FEATURE_SCALE_##_scale, \
  },
const struct ssf_mems_sensor_field_desc ssf_mems_sensor_fields[SSF_MEMS_FIELD_MAX] = {
    SSF_MEMS_SENSOR_FIELDS(SSF_MEMS_FIELD_DESC)
};
#undef SSF_MEMS_FIELD_DESC

/* 按功能码查找请求帧描述。
 * 返回：描述指针，未找到时为 NULL。
 */
const struct ssf_modbus_frame_desc *
ssf_mems_modbus_find_frame(u8 function) {
  size_t i;

  for (i = 0; i < SSF_FRAME_TABLE_SIZE; i++) {
    if (ssf_frame_table[i].tx_format != SSF_TX_NONE &&
        ssf_frame_table[i].function == function)
      return &ssf_frame_table[i];
  }
  return NULL;
}

/* 按功能码查找响应帧描述。
 * 返回：描述指针，未找到时为 NULL。
 * 支持普通响应和异常响应。
 */
const struct ssf_modbus_frame_desc *
ssf_mems_modbus_find_rx_frame(u8 function) {
  const struct ssf_modbus_frame_desc *normal;
  size_t i;

  normal = ssf_mems_modbus_find_frame(function & ~SSF_MEMS_MODBUS_EXCEPTION_FLAG);
  if (normal == NULL || (function & SSF_MEMS_MODBUS_EXCEPTION_FLAG) == 0)
    return normal;

  for (i = 0; i < SSF_FRAME_TABLE_SIZE; i++) {
    if (ssf_frame_table[i].rx_format == SSF_RX_EXCEPTION)
      return &ssf_frame_table[i];
  }
  return NULL;
}

/* 按手册地址查找寄存器。
 * 返回：表项指针，未找到时为 NULL。
 */
const struct ssf_reg_desc *
ssf_mems_modbus_find_reg(u16 display_reg) {
  size_t i;

  for (i = 0; i < SSF_REG_TABLE_SIZE; i++) {
    if (ssf_reg_table[i].display_reg == display_reg)
      return &ssf_reg_table[i];
  }
  return NULL;
}

/* 按名称查找寄存器。
 * 返回：表项指针，名称无效或未找到时为 NULL。
 * 名称区分大小写。
 */
const struct ssf_reg_desc *
ssf_mems_modbus_find_reg_by_name(const char *name) {
  size_t i;

  if (name == NULL || name[0] == '\0')
    return NULL;
  for (i = 0; i < SSF_REG_TABLE_SIZE; i++) {
    if (ssf_reg_table[i].name != NULL &&
        strcmp(ssf_reg_table[i].name, name) == 0)
      return &ssf_reg_table[i];
  }
  return NULL;
}

/* 按命令编号查找寄存器块。
 * 返回：块描述指针，未找到时为 NULL。
 */
const struct ssf_block_cmd_desc *
ssf_mems_modbus_find_block_cmd(enum ssf_block_cmd_id id) {
  size_t i;

  for (i = 0; i < SSF_BLOCK_CMD_TABLE_SIZE; i++) {
    if (ssf_block_cmd_table[i].id == id)
      return &ssf_block_cmd_table[i];
  }
  return NULL;
}

/* 查找指定范围内的下一个可读特征。
 * 返回：寄存器指针，没有剩余特征时为 NULL。
 */
const struct ssf_reg_desc *ssf_mems_modbus_next_feature(u32 cursor, u32 end) {
  const struct ssf_reg_desc *next = NULL;
  size_t i;

  for (i = 0; i < SSF_REG_TABLE_SIZE; i++) {
    const struct ssf_reg_desc *reg = &ssf_reg_table[i];

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
static bool ssf_mems_modbus_profile_valid(void) {
  const struct ssf_mems_sensor_profile *p = &ssf_mems_sensor_profile;
  unsigned int mask;
  size_t i;

  if (p->sampling_frequencies == NULL || p->sampling_frequency_count == 0 ||
      p->sampling_frequency_count > 65536U || p->baudrates == NULL ||
      p->baudrate_count == 0 || p->baudrate_count > SSF_MEMS_BAUDRATE_MAX ||
      (unsigned int)p->default_work_baudrate >= p->baudrate_count ||
      p->default_sampling_rate_index >= p->sampling_frequency_count ||
      p->raw_max_sampling_rate_index >= p->sampling_frequency_count ||
      p->raw_accel_scale_nano == 0 || p->raw_accel_scale_nano >= 1000000000U ||
      p->feature_enable_shift >= 16)
    return false;
  for (i = 0; i < p->sampling_frequency_count; i++)
    if (p->sampling_frequencies[i] == NULL || p->sampling_frequencies[i][0] == '\0')
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
  field = ssf_mems_sensor_field_desc(reg->feature_field);
  return field != NULL && (reg->type == SSF_REG_U16 || reg->type == SSF_REG_S16) &&
         (reg->access & SSF_REG_READ) != 0 &&
         reg->read_format == SSF_RX_HOLDING_REGS && reg->value_mask != 0 &&
         field->offset < sizeof(struct ssf_mems_sensor_data) &&
         field->width <= sizeof(struct ssf_mems_sensor_data) - field->offset &&
         (field->width == sizeof(u16) ||
          (field->width == sizeof(u8) && reg->value_mask <= 0xffU));
}

/* 检查型号表的编码、布局和名称是否有效。
 * 返回：0 有效，-EINVAL 无效；bad_name 指向出错配置或表项名称。
 * 允许稀疏寄存器，缺少业务表项仍由调用该业务时报告。
 */
int ssf_mems_modbus_validate_tables(const char **bad_name) {
  const char *bad = "sensor_profile";
  size_t i, j;

  if (!ssf_mems_modbus_profile_valid())
    goto invalid;
  for (i = 0; i < SSF_REG_TABLE_SIZE; i++) {
    const struct ssf_reg_desc *reg = &ssf_reg_table[i];

    bad = reg->name && reg->name[0] ? reg->name : "unnamed_register";
    if (!ssf_mems_modbus_reg_valid(reg))
      goto invalid;
    for (j = 0; j < i; j++) {
      const struct ssf_reg_desc *other = &ssf_reg_table[j];

      if (strcmp(reg->name, other->name) == 0 ||
          reg->display_reg == other->display_reg ||
          reg->protocol_addr == other->protocol_addr ||
          (reg->feature_field != SSF_MEMS_FIELD_NONE &&
           reg->feature_field == other->feature_field))
        goto invalid;
    }
  }
  for (i = 0; i < SSF_BLOCK_CMD_TABLE_SIZE; i++) {
    const struct ssf_block_cmd_desc *block = &ssf_block_cmd_table[i];
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
      if (block->id == ssf_block_cmd_table[j].id ||
          strcmp(block->name, ssf_block_cmd_table[j].name) == 0)
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

