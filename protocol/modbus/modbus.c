#include "modbus/modbus.h"
#include "model/modbus_table.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

/* 计算一段数据的 Modbus CRC16。
 * 返回：CRC16 校验值。
 */
u16 ssf_mems_modbus_crc16(const u8 *buf, size_t len) {
  u16 crc = SSF_MEMS_MODBUS_CRC_INIT;

  while (len-- != 0) {
    int i;

    crc ^= *buf++;
    for (i = 0; i < 8; i++)
      crc = (crc & 1) != 0 ? (crc >> 1) ^ SSF_MEMS_MODBUS_CRC_POLY : crc >> 1;
  }
  return crc;
}

/* 计算请求帧长度。
 * 返回：帧字节数，负数为参数、格式或长度错误。
 */
int ssf_mems_modbus_tx_frame_len(const struct ssf_modbus_frame_desc *frame,
                               u16 reg_count) {
  size_t len;

  if (frame == NULL)
    return -EINVAL;
  if (frame->tx_format == SSF_TX_NONE)
    return -EOPNOTSUPP;
  if (reg_count == 0 || reg_count > frame->max_reg_count)
    return -EINVAL;

  len = frame->tx_base_len + (size_t)frame->tx_bytes_per_reg * reg_count;
  if (len > SSF_MEMS_MODBUS_MAX_FRAME_LEN)
    return -EMSGSIZE;
  return len;
}

/* 计算响应帧长度。
 * 返回：帧字节数，负数为描述或长度错误。
 */
int ssf_mems_modbus_rx_frame_len(const struct ssf_modbus_frame_desc *frame,
                               u8 byte_count) {
  size_t len;

  if (frame == NULL)
    return -EINVAL;
  if (frame->rx_bytes_per_reg != 0 &&
      (byte_count == 0 || byte_count % frame->rx_bytes_per_reg != 0 ||
       byte_count / frame->rx_bytes_per_reg > frame->max_reg_count))
    return -EMSGSIZE;

  len = frame->rx_base_len + (frame->rx_bytes_per_reg != 0 ? byte_count : 0);
  if (len > SSF_MEMS_MODBUS_MAX_FRAME_LEN)
    return -EMSGSIZE;
  return len;
}

/* 把一个 16 位值按大端写入缓冲区。
 * 返回：无。
 * 调用者保证写入位置有效。
 */
static void ssf_mems_modbus_put_u16(u8 *buf, size_t offset, u16 value) {
  buf[offset] = value >> 8;
  buf[offset + 1] = value & 0xff;
}

/* 从缓冲区读取一个大端 16 位值。
 * 返回：读出的数值。
 * 调用者保证读取位置有效。
 */
static u16 ssf_mems_modbus_get_u16(const u8 *buf, size_t offset) {
  return ((u16)buf[offset] << 8) | buf[offset + 1];
}

/* 组装一个 Modbus 请求帧。
 * 返回：帧字节数，负数为参数、格式或空间错误。
 */
int ssf_mems_modbus_build_request(const struct ssf_modbus_transfer *transfer,
                                 u8 slave_id, const u16 *values,
                                 size_t values_count, u8 *buf, size_t capacity) {
  const struct ssf_modbus_frame_desc *frame;
  int len;
  u16 crc;
  u16 i;

  if (transfer == NULL || transfer->frame == NULL || buf == NULL)
    return -EINVAL;
  frame = transfer->frame;
  len = ssf_mems_modbus_tx_frame_len(frame, transfer->reg_count);
  if (len < 0)
    return len;
  if (capacity < len)
    return -ENOSPC;
  if (len < 2 + SSF_MEMS_MODBUS_CRC_LEN ||
      frame->address_offset + 2 > len - SSF_MEMS_MODBUS_CRC_LEN ||
      frame->quantity_offset + 2 > len - SSF_MEMS_MODBUS_CRC_LEN)
    return -EINVAL;
  if (frame->tx_format != SSF_TX_READ_REGS &&
      (values == NULL || values_count < transfer->reg_count))
    return -EINVAL;

  memset(buf, 0, len);
  buf[0] = slave_id;
  buf[1] = frame->function;
  ssf_mems_modbus_put_u16(buf, frame->address_offset, transfer->protocol_addr);
  ssf_mems_modbus_put_u16(buf, frame->quantity_offset,
                         frame->tx_format == SSF_TX_WRITE_SINGLE ? values[0] : transfer->reg_count);

  if (frame->tx_format == SSF_TX_WRITE_MULTI) {
    if (frame->tx_bytes_per_reg != sizeof(u16) ||
        frame->tx_byte_count_offset >= len - SSF_MEMS_MODBUS_CRC_LEN ||
        frame->tx_data_offset + transfer->reg_count * sizeof(u16) != len - SSF_MEMS_MODBUS_CRC_LEN)
      return -EINVAL;
    buf[frame->tx_byte_count_offset] = transfer->reg_count * sizeof(u16);
    for (i = 0; i < transfer->reg_count; i++)
      ssf_mems_modbus_put_u16(buf, frame->tx_data_offset + i * sizeof(u16), values[i]);
  }

  /* RTU 线上 CRC 低字节在前，与寄存器的大端数据顺序不同。 */
  crc = ssf_mems_modbus_crc16(buf, len - SSF_MEMS_MODBUS_CRC_LEN);
  buf[len - 2] = crc & 0xff;
  buf[len - 1] = crc >> 8;
  return len;
}

/* 检查 Modbus 帧的 CRC。
 * 返回：0 正确，-EINVAL 空帧，-EMSGSIZE 帧过短，-EBADMSG 校验失败。
 */
int ssf_mems_modbus_check_crc(const u8 *buf, size_t len) {
  u16 received;

  if (buf == NULL)
    return -EINVAL;
  if (len < 2 + SSF_MEMS_MODBUS_CRC_LEN)
    return -EMSGSIZE;
  received = buf[len - 2] | ((u16)buf[len - 1] << 8);
  return received == ssf_mems_modbus_crc16(buf, len - SSF_MEMS_MODBUS_CRC_LEN) ? 0 : -EBADMSG;
}

/* 解析当前请求对应的 Modbus 应答。
 * 返回：0 成功，-ENOMSG 不匹配，-EREMOTEIO 从机异常，其他负数为帧或参数错误。
 */
int ssf_mems_modbus_parse_response(const struct ssf_modbus_transfer *transfer,
                                   u8 slave_id, u16 write_value,
                                   const u8 *buf, size_t len,
                                   u16 *values, size_t values_count) {
  const struct ssf_modbus_frame_desc *frame;
  const struct ssf_modbus_frame_desc *rx;
  u8 byte_count = 0;
  int expected_len;
  int ret;
  u16 i;

  if (transfer == NULL || transfer->frame == NULL || buf == NULL)
    return -EINVAL;
  if (len < 2)
    return -EMSGSIZE;
  frame = transfer->frame;
  if (buf[0] != slave_id ||
      (buf[1] != frame->function && buf[1] != (frame->function | SSF_MEMS_MODBUS_EXCEPTION_FLAG)))
    return -ENOMSG;

  rx = ssf_mems_modbus_find_rx_frame(buf[1]);
  if (rx == NULL)
    return -ENOMSG;
  if (rx->rx_bytes_per_reg != 0) {
    if (len <= rx->rx_byte_count_offset)
      return -EMSGSIZE;
    byte_count = buf[rx->rx_byte_count_offset];
  }
  expected_len = ssf_mems_modbus_rx_frame_len(rx, byte_count);
  if (expected_len < 0 || len != expected_len)
    return -EMSGSIZE;
  ret = ssf_mems_modbus_check_crc(buf, len);
  if (ret != 0)
    return ret;
  if (rx->rx_format == SSF_RX_EXCEPTION)
    return -EREMOTEIO;

  if (rx->rx_format == SSF_RX_HOLDING_REGS) {
    if (byte_count != transfer->reg_count * rx->rx_bytes_per_reg)
      return -ENOMSG;
    if (values == NULL || values_count < transfer->reg_count)
      return -EFAULT;
    if (rx->rx_bytes_per_reg != sizeof(u16) ||
        rx->rx_data_offset + byte_count != len - SSF_MEMS_MODBUS_CRC_LEN)
      return -EMSGSIZE;
    for (i = 0; i < transfer->reg_count; i++)
      values[i] = ssf_mems_modbus_get_u16(buf, rx->rx_data_offset + i * sizeof(u16));
    return 0;
  }

  if (rx->address_offset + 2 > len - SSF_MEMS_MODBUS_CRC_LEN ||
      rx->quantity_offset + 2 > len - SSF_MEMS_MODBUS_CRC_LEN)
    return -EMSGSIZE;
  if (ssf_mems_modbus_get_u16(buf, rx->address_offset) != transfer->protocol_addr)
    return -ENOMSG;
  if (ssf_mems_modbus_get_u16(buf, rx->quantity_offset) !=
      (rx->rx_format == SSF_RX_WRITE_SINGLE_ECHO ? write_value : transfer->reg_count))
    return -ENOMSG;
  return 0;
}
