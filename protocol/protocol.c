#include "protocol.h"
#include "core.h"
#include "modbus_request.h"
#include "modbus_table.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

/* 初始化传感器协议缓存及其锁；返回 0 成功，-EINVAL 驱动数据为空。 */
int ssf_mems_protocol_init(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_protocol_state *state;

  if (!data)
    return -EINVAL;
  state = &data->protocol;
  mutex_init(&state->lock);
  memset(&state->features, 0, sizeof(state->features));
  state->valid = false;
  return 0;
}

/* 同步处理完整帧；返回 0 已认领或已丢弃，-EINVAL 空上下文/帧；未认领帧不额外解析、不缓存，由接收层释放。 */
int ssf_mems_protocol_handle_frame(void *context, const u8 *buf, size_t len) {
  struct ssf_mems_xyzs_data *data = context;

  if (!data || !buf)
    return -EINVAL;
  ssf_mems_modbus_claim_frame(data, buf, len);
  return 0;
}

/* 写入当前地址段的已保留特征字段；返回 0 成功，-EINVAL 参数或字段映射越界；删除的表项跳过。 */
static int ssf_mems_protocol_decode_feature_range(u16 start_reg,
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
int ssf_mems_protocol_decode_features(const u16 *registers, size_t count,
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
  return ssf_mems_protocol_decode_feature_range(block->start_display_reg, registers, count, result);
}

/* 返回范围内下一个仍可读的特征寄存器；没有剩余特征返回 NULL，表项顺序不影响选择。 */
static const struct ssf_reg_desc *
ssf_mems_protocol_next_feature(u32 cursor, u32 end) {
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
static void ssf_mems_protocol_store_features(struct ssf_mems_xyzs_data *data,
                                          const struct ssf_mems_sensor_data *features) {
  struct ssf_mems_protocol_state *state = &data->protocol;

  mutex_lock(&state->lock);
  state->features = *features;
  state->valid = true;
  mutex_unlock(&state->lock);
}

/* 按保留表项分段读取并缓存特征；返回 0 成功（无特征时缓存全零），-EINVAL 参数/块范围错误，-ENODEV 无驱动数据，-ENOENT 块命令缺失，-EOPNOTSUPP 块格式不支持；其他负值来自读请求或字段解码。 */
int ssf_mems_protocol_read_features(struct serdev_device *serdev,
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
    const struct ssf_reg_desc *first = ssf_mems_protocol_next_feature(cursor, end);
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
    ret = ssf_mems_protocol_decode_feature_range(first->display_reg, registers, count, &features);
    if (ret)
      return ret;
    cursor = (u32)first->display_reg + count;
  }

  ssf_mems_protocol_store_features(data, &features);
  return 0;
}

/* 返回 0 缓存复制成功，-EINVAL 空设备/输出指针，-ENODEV 无驱动数据，-ENODATA 尚无有效缓存。 */
int ssf_mems_protocol_get_features(struct serdev_device *serdev,
                                 struct ssf_mems_sensor_data *result) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_protocol_state *state;

  if (!serdev || !result)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;

  state = &data->protocol;
  mutex_lock(&state->lock);
  if (!state->valid) {
    mutex_unlock(&state->lock);
    return -ENODATA;
  }
  *result = state->features;
  mutex_unlock(&state->lock);
  return 0;
}

/* 固定写入块表规定的十个工作参数；返回 0 成功，-EINVAL 参数错误，-ENOENT 块/地址缺失；其他负值来自块请求校验、发送或等待。 */
int ssf_mems_protocol_write_work_parameters(struct serdev_device *serdev,
                                           const u16 *values, size_t values_count,
                                           unsigned int timeout_ms) {
  const struct ssf_block_cmd_desc *block;

  if (!serdev || !values)
    return -EINVAL;
  block = ssf_mems_modbus_find_block_cmd(SSF_BLOCK_WRITE_WORK_PARAMETERS);
  if (!block)
    return -ENOENT;
  return ssf_mems_modbus_write_block(serdev, block, values, values_count, timeout_ms);
}
