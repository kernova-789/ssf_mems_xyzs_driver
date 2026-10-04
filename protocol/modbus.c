#include "modbus.h"

#include "core.h"
#include "modbus_request.h"
#include "modbus_table.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

/* 返回输入字节序列的 Modbus CRC16；初值、多项式和 CRC 长度统一由表头定义。 */
u16 ssf_mems_modbus_crc16(const u8 *buf, size_t len) {
  u16 crc = SSF_MEMS_MODBUS_CRC_INIT;

  while (len--) {
    int i;

    crc ^= *buf++;
    for (i = 0; i < 8; i++)
      crc = (crc & 1) ? (crc >> 1) ^ SSF_MEMS_MODBUS_CRC_POLY : crc >> 1;
  }
  return crc;
}

/* 返回请求帧长度；-EINVAL 数量/描述错误，-EOPNOTSUPP 无发送格式，-EMSGSIZE 超出 RTU 帧上限。 */
int ssf_mems_modbus_tx_frame_len(const struct ssf_modbus_frame_desc *frame,
                               u16 reg_count) {
  size_t len;

  if (!frame)
    return -EINVAL;
  if (frame->tx_format == SSF_TX_NONE)
    return -EOPNOTSUPP;
  if (!reg_count || reg_count > frame->max_reg_count)
    return -EINVAL;

  len = frame->tx_base_len + (size_t)frame->tx_bytes_per_reg * reg_count;
  if (len > SSF_MEMS_MODBUS_MAX_FRAME_LEN)
    return -EMSGSIZE;
  return len;
}

/* 返回响应帧长度；-EINVAL 无描述，-EMSGSIZE 普通读取字节数为零、不是整寄存器或超限。 */
int ssf_mems_modbus_rx_frame_len(const struct ssf_modbus_frame_desc *frame,
                               u8 byte_count) {
  size_t len;

  if (!frame)
    return -EINVAL;
  if (frame->rx_bytes_per_reg &&
      (!byte_count || byte_count % frame->rx_bytes_per_reg ||
       byte_count / frame->rx_bytes_per_reg > frame->max_reg_count))
    return -EMSGSIZE;

  len = frame->rx_base_len + (frame->rx_bytes_per_reg ? byte_count : 0);
  if (len > SSF_MEMS_MODBUS_MAX_FRAME_LEN)
    return -EMSGSIZE;
  return len;
}

/* 返回 0 请求描述已生成；-EINVAL 参数/范围错误，-ENOENT 地址未入表，-EACCES 权限不足，-EOPNOTSUPP 帧或块命令不支持。 */
int ssf_mems_modbus_plan_request(u8 function, u16 display_reg, u16 reg_count,
                                const struct ssf_block_cmd_desc *block,
                                struct ssf_modbus_transfer *transfer) {
  const struct ssf_modbus_frame_desc *frame;
  const struct ssf_reg_desc *first;
  unsigned int access;
  u16 i;

  if (!transfer || !reg_count ||
      (u32)display_reg + reg_count > 0x10000U)
    return -EINVAL;

  frame = ssf_mems_modbus_find_frame(function);
  if (!frame)
    return -EOPNOTSUPP;
  if (reg_count > frame->max_reg_count)
    return -EINVAL;

  access = frame->tx_format == SSF_TX_READ_REGS ? SSF_REG_READ : SSF_REG_WRITE;
  if (block) {
    if (block->function != function || block->rx_format != frame->rx_format ||
        block->direction != (access == SSF_REG_READ ? SSF_CMD_READ : SSF_CMD_WRITE))
      return -EOPNOTSUPP;
    if (!block->reg_count || (u32)block->start_display_reg + block->reg_count > 0x10000U ||
        display_reg < block->start_display_reg ||
        (u32)display_reg + reg_count > (u32)block->start_display_reg + block->reg_count)
      return -EINVAL;
    if (!block->sparse_read &&
        (display_reg != block->start_display_reg || reg_count != block->reg_count))
      return -EINVAL;
  }

  first = ssf_mems_modbus_find_reg(display_reg);
  if (!first)
    return -ENOENT;
  if ((u32)first->protocol_addr + reg_count > 0x10000U)
    return -EINVAL;

  for (i = 0; i < reg_count; i++) {
    const struct ssf_reg_desc *reg = ssf_mems_modbus_find_reg(display_reg + i);

    if (!reg)
      return -ENOENT;
    if (!(reg->access & access))
      return -EACCES;
    if (reg->protocol_addr != (u32)first->protocol_addr + i)
      return -EINVAL;
    if (access == SSF_REG_READ && reg->read_format != frame->rx_format)
      return -EOPNOTSUPP;
    if (frame->tx_format == SSF_TX_WRITE_SINGLE && !reg->write_single)
      return -EOPNOTSUPP;
  }

  transfer->frame = frame;
  transfer->block = block;
  transfer->display_reg = display_reg;
  transfer->protocol_addr = first->protocol_addr;
  transfer->reg_count = reg_count;
  return 0;
}

/* 写入大端寄存器值；无返回值，调用前由帧描述和长度校验保证位置有效。 */
static void ssf_mems_modbus_put_u16(u8 *buf, size_t offset, u16 value) {
  buf[offset] = value >> 8;
  buf[offset + 1] = value & 0xff;
}

/* 返回指定位置的大端寄存器值，调用前必须完成帧长度校验。 */
static u16 ssf_mems_modbus_get_u16(const u8 *buf, size_t offset) {
  return ((u16)buf[offset] << 8) | buf[offset + 1];
}

/* 返回组装后的帧长度；-EINVAL 参数/布局错误，-ENOSPC 帧缓冲区不足，其他负值来自帧长度检查。 */
int ssf_mems_modbus_build_request(const struct ssf_modbus_transfer *transfer,
                                 u8 slave_id, const u16 *values,
                                 size_t values_count, u8 *buf, size_t capacity) {
  const struct ssf_modbus_frame_desc *frame;
  int len;
  u16 crc;
  u16 i;

  if (!transfer || !transfer->frame || !buf)
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
      (!values || values_count < transfer->reg_count))
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

  crc = ssf_mems_modbus_crc16(buf, len - SSF_MEMS_MODBUS_CRC_LEN);
  buf[len - 2] = crc & 0xff;
  buf[len - 1] = crc >> 8;
  return len;
}

/* 返回 0 CRC 正确，-EINVAL 空帧，-EMSGSIZE 帧过短，-EBADMSG CRC 不匹配。 */
int ssf_mems_modbus_check_crc(const u8 *buf, size_t len) {
  u16 received;

  if (!buf)
    return -EINVAL;
  if (len < 2 + SSF_MEMS_MODBUS_CRC_LEN)
    return -EMSGSIZE;
  received = buf[len - 2] | ((u16)buf[len - 1] << 8);
  return received == ssf_mems_modbus_crc16(buf, len - SSF_MEMS_MODBUS_CRC_LEN) ? 0 : -EBADMSG;
}

/* 返回 0 响应成功，-ENOMSG 不匹配当前请求，-EREMOTEIO 从机异常，-EFAULT 结果缓冲区异常，-EINVAL 参数错误，-EMSGSIZE 帧长错误，-EBADMSG CRC 错误。 */
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

  if (!transfer || !transfer->frame || !buf)
    return -EINVAL;
  if (len < 2)
    return -EMSGSIZE;
  frame = transfer->frame;
  if (buf[0] != slave_id ||
      (buf[1] != frame->function && buf[1] != (frame->function | SSF_MEMS_MODBUS_EXCEPTION_FLAG)))
    return -ENOMSG;

  rx = ssf_mems_modbus_find_rx_frame(buf[1]);
  if (!rx)
    return -ENOMSG;
  if (rx->rx_bytes_per_reg) {
    if (len <= rx->rx_byte_count_offset)
      return -EMSGSIZE;
    byte_count = buf[rx->rx_byte_count_offset];
  }
  expected_len = ssf_mems_modbus_rx_frame_len(rx, byte_count);
  if (expected_len < 0 || len != expected_len)
    return -EMSGSIZE;
  ret = ssf_mems_modbus_check_crc(buf, len);
  if (ret)
    return ret;
  if (rx->rx_format == SSF_RX_EXCEPTION)
    return -EREMOTEIO;

  if (rx->rx_format == SSF_RX_HOLDING_REGS) {
    if (byte_count != transfer->reg_count * rx->rx_bytes_per_reg)
      return -ENOMSG;
    if (!values || values_count < transfer->reg_count)
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

/* 写入当前地址段的已保留特征字段；返回 0 成功，-EINVAL 参数或字段映射越界；删除的表项跳过。 */
static int ssf_mems_modbus_decode_feature_range(u16 start_reg,
                                               const u16 *registers,
                                               size_t count,
                                               struct ssf_mems_sensor_data *result) {
  size_t i;

  if (!registers || !result || count > 0x10000U - start_reg)
    return -EINVAL;

  for (i = 0; i < count; i++) {
    const struct ssf_reg_desc *reg = ssf_mems_modbus_find_reg(start_reg + i);
    u16 value;
    u8 *dest;

    if (!reg || !reg->feature_width || !(reg->access & SSF_REG_READ))
      continue;
    if (reg->feature_offset > sizeof(*result) ||
        reg->feature_width > sizeof(*result) - reg->feature_offset)
      return -EINVAL;

    value = registers[i] & reg->value_mask;
    dest = (u8 *)result + reg->feature_offset;
    if (reg->feature_width == sizeof(u16)) {
      memcpy(dest, &value, sizeof(value));
    } else if (reg->feature_width == sizeof(u8)) {
      *dest = value;
    } else {
      return -EINVAL;
    }
  }
  return 0;
}

/* 按块表解码完整连续特征块；返回 0 成功，-ENOENT 块命令缺失，-EINVAL 参数/数量错误；删除表项对应字段置零。 */
int ssf_mems_modbus_decode_features(const u16 *registers, size_t count,
                                    struct ssf_mems_sensor_data *result) {
  const struct ssf_block_cmd_desc *block;

  if (!registers || !result)
    return -EINVAL;
  block = ssf_mems_modbus_find_block_cmd(SSF_BLOCK_READ_ALL_FEATURES);
  if (!block)
    return -ENOENT;
  if (count != block->reg_count || block->decode_kind != SSF_DECODE_FEATURES)
    return -EINVAL;

  memset(result, 0, sizeof(*result));
  return ssf_mems_modbus_decode_feature_range(block->start_display_reg, registers, count, result);
}

/* 返回范围内下一个仍可读的特征寄存器；没有剩余特征返回 NULL，表项顺序不影响选择。 */
static const struct ssf_reg_desc *
ssf_mems_modbus_next_feature(u32 cursor, u32 end) {
  const struct ssf_reg_desc *next = NULL;
  size_t i;

  for (i = 0; i < SSF_REG_TABLE_SIZE; i++) {
    const struct ssf_reg_desc *reg = &ssf_reg_table[i];

    if (!reg->feature_width || !(reg->access & SSF_REG_READ) ||
        reg->read_format != SSF_RX_HOLDING_REGS ||
        reg->display_reg < cursor || reg->display_reg >= end)
      continue;
    if (!next || reg->display_reg < next->display_reg)
      next = reg;
  }
  return next;
}

/* 原子替换特征值缓存；无返回值，未读取或删除的字段保持本轮初始化的零值。 */
static void ssf_mems_modbus_store_features(struct ssf_mems_xyzs_data *data,
                                          const struct ssf_mems_sensor_data *features) {
  mutex_lock(&data->sensor_data_lock);
  data->sensor_data = *features;
  data->sensor_data_valid = true;
  mutex_unlock(&data->sensor_data_lock);
}

/* 按保留表项分段读取并缓存特征；返回 0 成功（无特征时缓存全零），-EINVAL 参数/块范围错误，-ENODEV 无驱动数据，-ENOENT 块命令缺失，-EOPNOTSUPP 块格式不支持；其他负值来自读请求或字段解码。 */
int ssf_mems_modbus_read_features(struct serdev_device *serdev,
                                  unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_sensor_data features = {0};
  const struct ssf_block_cmd_desc *block;
  u16 registers[SSF_MEMS_MODBUS_READ_MAX_REGS];
  u32 cursor;
  u32 end;
  int ret;

  if (!serdev)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;
  block = ssf_mems_modbus_find_block_cmd(SSF_BLOCK_READ_ALL_FEATURES);
  if (!block)
    return -ENOENT;
  if (!block->sparse_read || block->decode_kind != SSF_DECODE_FEATURES ||
      block->rx_format != SSF_RX_HOLDING_REGS || block->function != SSF_MEMS_MODBUS_FUNC_READ)
    return -EOPNOTSUPP;

  cursor = block->start_display_reg;
  end = cursor + block->reg_count;
  if (!block->reg_count || end > 0x10000U)
    return -EINVAL;

  while (cursor < end) {
    const struct ssf_reg_desc *first = ssf_mems_modbus_next_feature(cursor, end);
    u16 count = 1;

    if (!first)
      break;
    while (count < ARRAY_SIZE(registers) && (u32)first->display_reg + count < end) {
      const struct ssf_reg_desc *reg = ssf_mems_modbus_find_reg(first->display_reg + count);

      if (!reg || !reg->feature_width || !(reg->access & SSF_REG_READ) ||
          reg->read_format != block->rx_format ||
          reg->protocol_addr != (u32)first->protocol_addr + count)
        break;
      count++;
    }

    ret = ssf_mems_modbus_read_block_range(serdev, block, first->display_reg,
                                          count, registers, ARRAY_SIZE(registers),
                                          timeout_ms);
    if (ret)
      return ret;
    ret = ssf_mems_modbus_decode_feature_range(first->display_reg, registers, count, &features);
    if (ret)
      return ret;
    cursor = (u32)first->display_reg + count;
  }

  ssf_mems_modbus_store_features(data, &features);
  return 0;
}

/* 返回 0 缓存复制成功，-EINVAL 空设备/输出指针，-ENODEV 无驱动数据，-ENODATA 尚无有效缓存。 */
int ssf_mems_modbus_get_features(struct serdev_device *serdev,
                                 struct ssf_mems_sensor_data *result) {
  struct ssf_mems_xyzs_data *data;

  if (!serdev || !result)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;

  mutex_lock(&data->sensor_data_lock);
  if (!data->sensor_data_valid) {
    mutex_unlock(&data->sensor_data_lock);
    return -ENODATA;
  }
  *result = data->sensor_data;
  mutex_unlock(&data->sensor_data_lock);
  return 0;
}
