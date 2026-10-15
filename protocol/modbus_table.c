// SPDX-License-Identifier: GPL-2.0-only
#define SSF_MEMS_DEFINE_MODBUS_TABLE
#include "modbus_table.h"

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

    if (reg->feature_width == 0 || (reg->access & SSF_REG_READ) == 0 ||
        reg->read_format != SSF_RX_HOLDING_REGS || reg->display_reg < cursor ||
        reg->display_reg >= end)
      continue;
    if (next == NULL || reg->display_reg < next->display_reg)
      next = reg;
  }
  return next;
}

