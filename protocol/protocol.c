#include "protocol.h"
#include "core.h"
#include "modbus_receive.h"
#include "modbus_request.h"
#include "modbus_table.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

static const int ssf_mems_baudrates_table[SSF_MEMS_BAUDRATE_MAX] = {
    [SSF_MEMS_BAUDRATE_DEFAULT] = 9600,  [SSF_MEMS_BAUDRATE_2400] = 2400,
    [SSF_MEMS_BAUDRATE_4800] = 4800,     [SSF_MEMS_BAUDRATE_9600] = 9600,
    [SSF_MEMS_BAUDRATE_19200] = 19200,   [SSF_MEMS_BAUDRATE_38400] = 38400,
    [SSF_MEMS_BAUDRATE_57600] = 57600,   [SSF_MEMS_BAUDRATE_115200] = 115200,
    [SSF_MEMS_BAUDRATE_128000] = 128000, [SSF_MEMS_BAUDRATE_230400] = 230400,
    [SSF_MEMS_BAUDRATE_256000] = 256000, [SSF_MEMS_BAUDRATE_460800] = 460800,
    [SSF_MEMS_BAUDRATE_500000] = 500000, [SSF_MEMS_BAUDRATE_512000] = 512000,
    [SSF_MEMS_BAUDRATE_600000] = 600000, [SSF_MEMS_BAUDRATE_750000] = 750000,
    [SSF_MEMS_BAUDRATE_921600] = 921600, [SSF_MEMS_BAUDRATE_1000000] = 1000000,
};

/* 初始化传感器协议缓存及其锁；返回 0 成功，-EINVAL 驱动数据为空。 */
int ssf_mems_protocol_init(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_protocol_state *state;

  if (!data)
    return -EINVAL;
  state = &data->protocol;
  state->baudrate = SSF_MEMS_BAUDRATE_DEFAULT;
  mutex_init(&state->lock);
  mutex_init(&state->io_lock);
  memset(&state->features, 0, sizeof(state->features));
  state->valid = false;
  return 0;
}

/* 同步处理完整帧；返回 0 已认领或已丢弃，-EINVAL
 * 空上下文/帧；未认领帧不额外解析、不缓存，由接收层释放。 */
int ssf_mems_protocol_handle_frame(void *context, const u8 *buf, size_t len) {
  struct ssf_mems_xyzs_data *data = context;

  if (!data || !buf)
    return -EINVAL;
  ssf_mems_modbus_claim_frame(data, buf, len);
  return 0;
}

/* 写入当前地址段的已保留特征字段；返回 0 成功，-EINVAL
 * 参数或字段映射越界；删除的表项跳过。 */
static int
ssf_mems_protocol_decode_feature_range(u16 start_reg, const u16 *registers,
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

/* 按块表解码完整连续特征块；返回 0 成功，-ENOENT 块命令缺失，-EINVAL
 * 参数/数量错误；删除表项对应字段置零。 */
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
  return ssf_mems_protocol_decode_feature_range(block->start_display_reg,
                                                registers, count, result);
}

/* 返回范围内下一个仍可读的特征寄存器；没有剩余特征返回
 * NULL，表项顺序不影响选择。 */
static const struct ssf_reg_desc *ssf_mems_protocol_next_feature(u32 cursor,
                                                                 u32 end) {
  const struct ssf_reg_desc *next = NULL;
  size_t i;

  for (i = 0; i < SSF_REG_TABLE_SIZE; i++) {
    const struct ssf_reg_desc *reg = &ssf_reg_table[i];

    if (!reg->feature_width || !(reg->access & SSF_REG_READ) ||
        reg->read_format != SSF_RX_HOLDING_REGS || reg->display_reg < cursor ||
        reg->display_reg >= end)
      continue;
    if (!next || reg->display_reg < next->display_reg)
      next = reg;
  }
  return next;
}

/* 原子替换特征值缓存；未读取或删除的字段保持本轮初始化的零值。 */
int
ssf_mems_protocol_store_features(struct ssf_mems_xyzs_data *data,
                                 const struct ssf_mems_sensor_data *features) {
  struct ssf_mems_protocol_state *state;

  if (!data || !features)
    return -EINVAL;
  state = &data->protocol;

  mutex_lock(&state->lock);
  state->features = *features;
  state->valid = true;
  mutex_unlock(&state->lock);
  return 0;
}

/* 按保留表项分段读取并缓存特征；返回 0 成功（无特征时缓存全零），-EINVAL
 * 参数/块范围错误，-ENODEV 无驱动数据，-ENOENT 块命令缺失，-EOPNOTSUPP
 * 块格式不支持；其他负值来自读请求或字段解码。 */
int ssf_mems_protocol_read_features(struct serdev_device *serdev,
                                    unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_protocol_state *state;
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
  state = &data->protocol;
  mutex_lock(&state->io_lock);

  block = ssf_mems_modbus_find_block_cmd(SSF_BLOCK_READ_ALL_FEATURES);
  if (!block) {
    ret = -ENOENT;
    goto out_unlock;
  }
  if (!block->sparse_read || block->decode_kind != SSF_DECODE_FEATURES ||
      block->rx_format != SSF_RX_HOLDING_REGS ||
      block->function != SSF_MEMS_MODBUS_FUNC_READ) {
    ret = -EOPNOTSUPP;
    goto out_unlock;
  }

  cursor = block->start_display_reg;
  end = cursor + block->reg_count;
  if (!block->reg_count || end > 0x10000U) {
    ret = -EINVAL;
    goto out_unlock;
  }

  while (cursor < end) {
    const struct ssf_reg_desc *first =
        ssf_mems_protocol_next_feature(cursor, end);
    u16 count = 1;

    if (!first)
      break;
    while (count < ARRAY_SIZE(registers) &&
           (u32)first->display_reg + count < end) {
      const struct ssf_reg_desc *reg =
          ssf_mems_modbus_find_reg(first->display_reg + count);

      if (!reg || !reg->feature_width || !(reg->access & SSF_REG_READ) ||
          reg->read_format != block->rx_format ||
          reg->protocol_addr != (u32)first->protocol_addr + count)
        break;
      count++;
    }

    ret = ssf_mems_modbus_read_block_range(serdev, block, first->display_reg,
                                           count, registers,
                                           ARRAY_SIZE(registers), timeout_ms);
    if (ret)
      goto out_unlock;
    ret = ssf_mems_protocol_decode_feature_range(first->display_reg, registers,
                                                 count, &features);
    if (ret)
      goto out_unlock;
    cursor = (u32)first->display_reg + count;
  }

  ret = ssf_mems_protocol_store_features(data, &features);

out_unlock:
  mutex_unlock(&state->io_lock);
  return ret;
}

/* 返回 0 缓存复制成功，-EINVAL 空设备/输出指针，-ENODEV 无驱动数据，-ENODATA
 * 尚无有效缓存。 */
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

/* 固定写入块表规定的十个工作参数；返回 0 成功，-EINVAL 参数错误，-ENOENT
 * 块/地址缺失；其他负值来自块请求校验、发送或等待。 */
int ssf_mems_protocol_write_work_parameters(struct serdev_device *serdev,
                                            const u16 *values,
                                            size_t values_count,
                                            unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data;
  const struct ssf_block_cmd_desc *block;
  int ret;

  if (!serdev || !values)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;
  block = ssf_mems_modbus_find_block_cmd(SSF_BLOCK_WRITE_WORK_PARAMETERS);
  if (!block)
    return -ENOENT;

  mutex_lock(&data->protocol.io_lock);
  ret = ssf_mems_modbus_write_block(serdev, block, values, values_count,
                                    timeout_ms);
  mutex_unlock(&data->protocol.io_lock);
  return ret;
}

/* 将寄存器枚举转换为串口波特率；非法枚举返回 -EINVAL。 */
int ssf_mems_baudrate_to_value(enum ssf_mems_baudrate baudrate) {
  if ((unsigned int)baudrate >= SSF_MEMS_BAUDRATE_MAX)
    return -EINVAL;

  return ssf_mems_baudrates_table[baudrate];
}

/* 设置主机串口波特率；返回 0，控制器不支持设置时返回 -EIO。 */
static int
ssf_mems_protocol_set_host_baudrate(struct ssf_mems_xyzs_data *data,
                                    enum ssf_mems_baudrate baudrate) {
  unsigned int actual;
  int requested;

  requested = ssf_mems_baudrate_to_value(baudrate);
  if (requested < 0)
    return requested;

  actual = serdev_device_set_baudrate(data->serdev, requested);
  if (!actual)
    return -EIO;
  if (actual != requested)
    dev_dbg(&data->serdev->dev,
            "requested baudrate %d, controller selected %u\n", requested,
            actual);
  return 0;
}

/* 只尝试一次读取 40102，并校验寄存器值是受支持的枚举。 */
static int ssf_mems_protocol_read_baudrate_once(
    struct serdev_device *serdev, enum ssf_mems_baudrate *baudrate,
    unsigned int timeout_ms) {
  u16 value;
  int ret;

  ret = ssf_mems_modbus_read_reg(serdev, 40102, &value, timeout_ms);
  if (ret)
    return ret;
  if (value >= SSF_MEMS_BAUDRATE_MAX)
    return -EPROTO;

  *baudrate = value;
  return 0;
}

/* 判断某个候选波特率是否已经尝试过，避免重复扫描 */
/* 返回 true 表示该实际速率已由入口速率或更早枚举覆盖。 */
static bool ssf_mems_protocol_baudrate_tried(
    enum ssf_mems_baudrate candidate, enum ssf_mems_baudrate initial) {
  unsigned int i;
  int value = ssf_mems_baudrates_table[candidate];

  if (value == ssf_mems_baudrates_table[initial])
    return true;
  for (i = 0; i < (unsigned int)candidate; i++) {
    if (ssf_mems_baudrates_table[i] == value &&
        ssf_mems_baudrates_table[i] != ssf_mems_baudrates_table[initial])
      return true;
  }
  return false;
}

/* 写入传感器波特率寄存器，在写回显应答完整匹配后切换主机串口。 */
int ssf_mems_protocol_set_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate baudrate,
                                   unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data;
  int ret;

  if (!serdev || (unsigned int)baudrate >= SSF_MEMS_BAUDRATE_MAX)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;

  mutex_lock(&data->protocol.io_lock);
  ret = ssf_mems_modbus_write_reg(serdev, 40102, baudrate, timeout_ms);
  if (ret)
    goto out_unlock;

  ssf_mems_modbus_receive_flush(data);
  ret = ssf_mems_protocol_set_host_baudrate(data, baudrate);
  if (!ret)
    data->protocol.baudrate = baudrate;

out_unlock:
  mutex_unlock(&data->protocol.io_lock);
  return ret;
}

/* 先以当前速率读取；仅当没有可匹配应答而超时时扫描其他枚举速率。 */
int ssf_mems_protocol_get_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate *baudrate,
                                   unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_protocol_state *state;
  enum ssf_mems_baudrate initial;
  enum ssf_mems_baudrate found;
  unsigned int i;
  int restore_ret;
  int ret;

  if (!serdev || !baudrate)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;
  state = &data->protocol;

  mutex_lock(&state->io_lock);
  initial = state->baudrate;
  if ((unsigned int)initial >= SSF_MEMS_BAUDRATE_MAX) {
    ret = -EINVAL;
    goto out_unlock;
  }

  ret = ssf_mems_protocol_read_baudrate_once(serdev, &found, timeout_ms);
  if (!ret) {
    state->baudrate = found;
    *baudrate = found;
    goto out_unlock;
  }
  if (ret != -ETIMEDOUT)
    goto out_unlock;

  for (i = 0; i < SSF_MEMS_BAUDRATE_MAX; i++) {
    enum ssf_mems_baudrate candidate = i;

    if (ssf_mems_protocol_baudrate_tried(candidate, initial))
      continue;

    ssf_mems_modbus_receive_flush(data);
    ret = ssf_mems_protocol_set_host_baudrate(data, candidate);
    if (ret)
      goto restore_initial;

    ret = ssf_mems_protocol_read_baudrate_once(serdev, &found, timeout_ms);
    if (!ret) {
      if (ssf_mems_baudrates_table[found] !=
          ssf_mems_baudrates_table[candidate]) {
        ret = -EPROTO;
        goto restore_initial;
      }
      state->baudrate = found;
      *baudrate = found;
      ssf_mems_modbus_receive_flush(data);
      goto out_unlock;
    }
    if (ret != -ETIMEDOUT)
      goto restore_initial;
  }

  ret = -ETIMEDOUT;

restore_initial:
  ssf_mems_modbus_receive_flush(data);
  restore_ret = ssf_mems_protocol_set_host_baudrate(data, initial);
  if (restore_ret)
    ret = restore_ret;

out_unlock:
  mutex_unlock(&state->io_lock);
  return ret;
}
