// SPDX-License-Identifier: GPL-2.0-only
#include "model/modbus_table.h"

const struct ssf_modbus_frame_desc ssf_frame_table[] = {
    {
        .function = SSF_MEMS_MODBUS_FUNC_READ,
        .tx_format = SSF_TX_READ_REGS,
        .rx_format = SSF_RX_HOLDING_REGS,
        .max_reg_count = SSF_MEMS_MODBUS_READ_MAX_REGS,
        .tx_base_len = 8,
        .address_offset = 2,
        .quantity_offset = 4,
        .rx_base_len = 5,
        .rx_bytes_per_reg = 2,
        .rx_byte_count_offset = 2,
        .rx_data_offset = 3,
    },
    {
        .function = SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE,
        .tx_format = SSF_TX_WRITE_SINGLE,
        .rx_format = SSF_RX_WRITE_SINGLE_ECHO,
        .max_reg_count = 1,
        .tx_base_len = 8,
        .address_offset = 2,
        .quantity_offset = 4,
        .rx_base_len = 8,
    },
    {
        .function = SSF_MEMS_MODBUS_FUNC_WRITE_MULTI,
        .tx_format = SSF_TX_WRITE_MULTI,
        .rx_format = SSF_RX_WRITE_MULTI_ACK,
        .max_reg_count = SSF_MEMS_MODBUS_WRITE_MAX_REGS,
        .tx_base_len = 9,
        .tx_bytes_per_reg = 2,
        .tx_data_offset = 7,
        .address_offset = 2,
        .quantity_offset = 4,
        .tx_byte_count_offset = 6,
        .rx_base_len = 8,
    },
    {
        .tx_format = SSF_TX_NONE,
        .rx_format = SSF_RX_EXCEPTION,
        .rx_base_len = 5,
        .rx_data_offset = 2,
    },
};

const size_t ssf_frame_table_size = ARRAY_SIZE(ssf_frame_table);

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

