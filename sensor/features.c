// SPDX-License-Identifier: GPL-2.0-only
#include "sensor.h"
#include "core.h"
#include "modbus/modbus_request.h"
#include <linux/errno.h>
#include <linux/string.h>

/* 把一段寄存器值填入特征结构体。
 * 返回：0 成功，-EINVAL 参数或字段范围无效，-ERANGE 换算越界。
 * 跳过未入表的特征。
 */
static int
ssf_mems_sensor_decode_feature_range(
    const struct ssf_mems_sensor *sensor, u16 start_reg, const u16 *registers,
    size_t count, struct ssf_mems_sensor_data *result) {
  size_t i;

  if (registers == NULL || result == NULL || count > 0x10000U - start_reg)
    return -EINVAL;

  for (i = 0; i < count; i++) {
    const struct ssf_reg_desc *reg = ssf_mems_sensor_find_reg(sensor, start_reg + i);
    const struct ssf_mems_sensor_field_desc *field;
    u16 stored;
    s64 value;
    u8 *dest;

    if (reg == NULL || reg->feature_field == SSF_MEMS_FIELD_NONE ||
        (reg->access & SSF_REG_READ) == 0)
      continue;
    field = &reg->field;
    if (field == NULL || field->offset > sizeof(*result) ||
        field->width > sizeof(*result) - field->offset)
      return -EINVAL;

    value = registers[i] & reg->value_mask;
    if (reg->type == SSF_REG_S16)
      value = (s16)value;
    /* 按型号表比例统一到 sensor_data.h 的单位。 */
    value = (s64)value * (reg->feature_multiplier ? reg->feature_multiplier : 1U) /
            (reg->feature_divisor ? reg->feature_divisor : 1U);
    dest = (u8 *)result + field->offset;
    if (field->width == sizeof(u16)) {
      if (reg->type == SSF_REG_S16 ? value < -32768 || value > 32767 :
                                    value < 0 || value > 65535)
        return -ERANGE;
      stored = (u16)value;
      memcpy(dest, &stored, sizeof(stored));
    } else if (field->width == sizeof(u8)) {
      if (value < 0 || value > 255)
        return -ERANGE;
      *dest = value;
    } else {
      return -EINVAL;
    }
  }
  return 0;
}

/* 把完整特征块转换为特征结构体。
 * 返回：0 成功，-EINVAL 参数无效，-ENOENT 块描述缺失，-ERANGE 换算越界。
 * 删除的表项对应字段置零。
 */
int ssf_mems_sensor_decode_features(
    const struct ssf_mems_sensor *sensor, const u16 *registers, size_t count,
    struct ssf_mems_sensor_data *result) {
  const struct ssf_block_cmd_desc *block;

  if (registers == NULL || result == NULL)
    return -EINVAL;
  block = ssf_mems_sensor_find_block(sensor, SSF_BLOCK_READ_ALL_FEATURES);
  if (block == NULL)
    return -ENOENT;
  if (count != block->reg_count || block->decode_kind != SSF_DECODE_FEATURES)
    return -EINVAL;

  memset(result, 0, sizeof(*result));
  return ssf_mems_sensor_decode_feature_range(sensor, block->start_display_reg,
                                                registers, count, result);
}

/* 分段读取全部特征并更新缓存。
 * 返回：0 成功，负数为参数、表项、通信或解码错误。
 * 所有分段成功后才更新缓存。
 */
int ssf_mems_sensor_read_features(struct ssf_mems_data *data,
                                    unsigned int timeout_ms) {
  struct ssf_mems_protocol_state *state;
  struct ssf_mems_sensor_data features = {0};
  const struct ssf_block_cmd_desc *block;
  u16 registers[SSF_MEMS_MODBUS_READ_MAX_REGS];
  u32 cursor;
  u32 end;
  int ret;

  if (data == NULL || data->serdev == NULL || data->sensor == NULL)
    return -EINVAL;
  state = &data->protocol;
  mutex_lock(&state->bus_lock);

  block = ssf_mems_sensor_find_block(data->sensor, SSF_BLOCK_READ_ALL_FEATURES);
  if (block == NULL) {
    ret = -ENOENT;
    goto out_unlock;
  }
  if (block->sparse_read == false || block->decode_kind != SSF_DECODE_FEATURES ||
      block->rx_format != SSF_RX_HOLDING_REGS ||
      block->function != SSF_MEMS_MODBUS_FUNC_READ) {
    ret = -EOPNOTSUPP;
    goto out_unlock;
  }

  cursor = block->start_display_reg;
  end = cursor + block->reg_count;
  if (block->reg_count == 0 || end > 0x10000U) {
    ret = -EINVAL;
    goto out_unlock;
  }

  while (cursor < end) {
    const struct ssf_reg_desc *first =
        ssf_mems_sensor_next_feature(data->sensor, cursor, end);
    u16 count = 1;

    if (first == NULL)
      break;
    while (count < ARRAY_SIZE(registers) &&
           (u32)first->display_reg + count < end) {
      const struct ssf_reg_desc *reg =
          ssf_mems_sensor_find_reg(data->sensor, first->display_reg + count);

      if (reg == NULL || reg->feature_field == SSF_MEMS_FIELD_NONE ||
          (reg->access & SSF_REG_READ) == 0 ||
          reg->read_format != block->rx_format ||
          reg->protocol_addr != (u32)first->protocol_addr + count)
        break;
      count++;
    }

    ret = ssf_mems_modbus_read_block_range_locked(
        data->serdev, block, first->display_reg, count, registers,
        ARRAY_SIZE(registers), timeout_ms);
    if (ret != 0)
      goto out_unlock;
    ret = ssf_mems_sensor_decode_feature_range(data->sensor, first->display_reg, registers,
                                                 count, &features);
    if (ret != 0)
      goto out_unlock;
    cursor = (u32)first->display_reg + count;
  }

  /* 所有分段均读取/解码成功才发布，避免新旧混合的主机快照。 */
  ret = ssf_mems_protocol_store_features(data, &features);

out_unlock:
  mutex_unlock(&state->bus_lock);
  return ret;
}

