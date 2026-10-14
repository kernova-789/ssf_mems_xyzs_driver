#include "protocol.h"
#include "core.h"
#include "modbus.h"
#include "modbus_receive.h"
#include "modbus_request.h"
#include "modbus_table.h"

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/property.h>
#include <linux/string.h>

/* 40102 的编码转换为实际 bit/s；编码 0 和 3 都表示 9600。 */
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

/* 解析启动属性；返回 0 成功或已使用默认值，-EINVAL 从站地址/校验位非法。 */
static int ssf_mems_protocol_parse_properties(struct ssf_mems_xyzs_data *data);

/* 高层配置项对应的手册显示地址；特征掩码和完整开关字共用 40070。 */
static const u16 ssf_mems_setting_registers[SSF_MEMS_SETTING_MAX] = {
    [SSF_MEMS_SETTING_SAMPLING_RATE] = 40052,
    [SSF_MEMS_SETTING_SAMPLING_LENGTH] = 40053,
    [SSF_MEMS_SETTING_PARAMETER_SWITCH] = 40070,
    [SSF_MEMS_SETTING_FEATURE_ENABLE] = 40070,
    [SSF_MEMS_SETTING_FIRMWARE_VERSION] = 40121,
};

/* 读取指定配置；返回 0 成功，-EINVAL 参数/配置项非法，-ENODEV 无驱动数据；
 * 其余错误及含义见 modbus_request.h 的请求返回值约定。 */
int ssf_mems_protocol_read_setting(struct serdev_device *serdev,
                                  enum ssf_mems_sensor_setting setting,
                                  u16 *value, unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data;
  u16 reg_value;
  int ret;

  if (serdev == NULL || value == NULL ||
      (unsigned int)setting >= SSF_MEMS_SETTING_MAX)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;
  mutex_lock(&data->protocol.bus_lock);
  ret = ssf_mems_modbus_read_reg_locked(
      serdev, ssf_mems_setting_registers[setting], &reg_value, timeout_ms);
  mutex_unlock(&data->protocol.bus_lock);
  if (ret == 0)
    *value = setting == SSF_MEMS_SETTING_FEATURE_ENABLE ? reg_value >> 10 : reg_value;
  return ret;
}

/* 仅允许采样率索引 0..9 和特征掩码 0..63；不改报警位，不发送保存/重启。
 * 固件自行将采样率变化保存至 Flash。相同值不重复写，写后读回验证。
 * 返回 0 成功/无需写入，-EINVAL 参数/范围非法，-ENODEV 无驱动数据，
 * -EIO 读回值不符；其余错误见 modbus_request.h 的请求返回值约定。 */
int ssf_mems_protocol_write_setting(struct serdev_device *serdev,
                                   enum ssf_mems_sensor_setting setting,
                                   u16 value, unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data;
  u16 reg_value, desired, verified;
  u16 reg;
  int ret;

  if (serdev == NULL ||
      (setting != SSF_MEMS_SETTING_SAMPLING_RATE &&
       setting != SSF_MEMS_SETTING_FEATURE_ENABLE) ||
      (setting == SSF_MEMS_SETTING_SAMPLING_RATE &&
       value > SSF_MEMS_SAMPLING_RATE_MAX_INDEX) ||
      (setting == SSF_MEMS_SETTING_FEATURE_ENABLE && value > 63))
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;
  reg = ssf_mems_setting_registers[setting];
  mutex_lock(&data->protocol.bus_lock);
  ret = ssf_mems_modbus_read_reg_locked(serdev, reg, &reg_value, timeout_ms);
  if (ret != 0)
    goto out;
  /* 只替换高六位特征使能，保留报警等低十位；同值不写，减少重复配置。 */
  desired = setting == SSF_MEMS_SETTING_FEATURE_ENABLE ?
                (reg_value & 0x03ff) | (value << 10) : value;
  if (desired == reg_value)
    goto out;
  ret = ssf_mems_modbus_write_reg_locked(serdev, reg, desired, timeout_ms);
  if (ret != 0)
    goto out;
  ret = ssf_mems_modbus_read_reg_locked(serdev, reg, &verified, timeout_ms);
  if (ret == 0 && verified != desired)
    ret = -EIO;
out:
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

/* 初始化协议状态并应用启动属性；不配置 UART、不发送 Modbus。
 * 返回 0 成功（属性缺失时使用默认值），-EINVAL 空设备/非法地址或校验位。 */
int ssf_mems_protocol_init(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_protocol_state *state;

  if (data == NULL || data->serdev == NULL)
    return -EINVAL;
  data->slave_id = SSF_MEMS_MODBUS_DEFAULT_SLAVE_ID;
  state = &data->protocol;
  state->baudrate = SSF_MEMS_BAUDRATE_DEFAULT;
  state->host_baudrate =
      ssf_mems_baudrates_table[SSF_MEMS_BAUDRATE_DEFAULT];
  state->parity = SSF_MEMS_PARITY_NONE;
  state->mode = SSF_MEMS_LINK_MODBUS;
  state->raw_start_until = jiffies;
  mutex_init(&state->lock);
  mutex_init(&state->bus_lock);
  memset(&state->features, 0, sizeof(state->features));
  state->valid = false;
  ssf_mems_raw_init(data);
  ssf_mems_ota_init(data);
  return ssf_mems_protocol_parse_properties(data);
}

/* 波特率缺失、读失败或不受支持时使用 9600；地址和校验位非法仍报错。
 * 解析启动属性；返回 0 成功或已使用默认值，-EINVAL 从站地址/校验位非法。 */
static int ssf_mems_protocol_parse_properties(struct ssf_mems_xyzs_data *data) {
  struct device *dev = &data->serdev->dev;
  const char *parity;
  u32 value;
  int ret;

  ret = device_property_read_u32(dev, "sange-cbm,slave-id", &value);
  if (ret != 0) {
    dev_info(dev, "could not read sange-cbm,slave-id (%d), using default slave ID 1\n",
             ret);
  } else {
    if (value < 1 || value > 247) {
      dev_err(dev, "invalid sange-cbm,slave-id: %u (expected 1..247)\n", value);
      return -EINVAL;
    }
    data->slave_id = value;
  }

  ret = device_property_read_u32(dev, "current-speed", &value);
  if (ret != 0) {
    dev_info(dev, "could not read current-speed (%d), using default baud rate 9600\n",
             ret);
  } else {
    ret = ssf_mems_baudrate_from_value(value, &data->protocol.baudrate);
    if (ret != 0) {
      dev_info(dev, "unsupported current-speed %u, using default baud rate 9600\n",
               value);
      data->protocol.baudrate = SSF_MEMS_BAUDRATE_DEFAULT;
      data->protocol.host_baudrate = 9600;
    } else {
      data->protocol.host_baudrate = value;
    }
  }

  ret = device_property_read_string(dev, "sange-cbm,parity", &parity);
  if (ret != 0) {
    dev_info(dev, "could not read sange-cbm,parity (%d), using default parity none\n",
             ret);
    return 0;
  }
  if (strcmp(parity, "none") == 0)
    data->protocol.parity = SSF_MEMS_PARITY_NONE;
  else if (strcmp(parity, "odd") == 0)
    data->protocol.parity = SSF_MEMS_PARITY_ODD;
  else if (strcmp(parity, "even") == 0)
    data->protocol.parity = SSF_MEMS_PARITY_EVEN;
  else {
    dev_err(dev, "invalid sange-cbm,parity: %s (expected none/odd/even)\n", parity);
    return -EINVAL;
  }

  return 0;
}

/* 同步处理完整帧；返回 0 已认领或已丢弃，-EINVAL
 * 空上下文/帧；未认领帧不额外解析、不缓存，由接收层释放。 */
int ssf_mems_protocol_handle_frame(void *context, const u8 *buf, size_t len) {
  struct ssf_mems_xyzs_data *data = context;

  if (data == NULL || buf == NULL)
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

  if (registers == NULL || result == NULL || count > 0x10000U - start_reg)
    return -EINVAL;

  for (i = 0; i < count; i++) {
    const struct ssf_reg_desc *reg = ssf_mems_modbus_find_reg(start_reg + i);
    u16 value;
    u8 *dest;

    if (reg == NULL || reg->feature_width == 0 || (reg->access & SSF_REG_READ) == 0)
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

  if (registers == NULL || result == NULL)
    return -EINVAL;
  block = ssf_mems_modbus_find_block_cmd(SSF_BLOCK_READ_ALL_FEATURES);
  if (block == NULL)
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

    if (reg->feature_width == 0 || (reg->access & SSF_REG_READ) == 0 ||
        reg->read_format != SSF_RX_HOLDING_REGS || reg->display_reg < cursor ||
        reg->display_reg >= end)
      continue;
    if (next == NULL || reg->display_reg < next->display_reg)
      next = reg;
  }
  return next;
}

/* 以定点整数拆分方式打印单轴特征；无返回值，name/axis 必须有效。 */
static void ssf_mems_protocol_log_axis(
    const char *name, const struct ssf_mems_axis_features *axis) {
  pr_info("ssf_mems: %s acc_rms=%u.%02u g peak=%u.%02u g "
          "peak_to_peak=%u.%02u g high_freq_acc_rms=%u.%02u g "
          "velocity_rms=%u.%02u mm/s low_freq_velocity_rms=%u.%02u mm/s\n",
          name,
          axis->acc_rms_x100 / SSF_MEMS_CENTI_SCALE,
          axis->acc_rms_x100 % SSF_MEMS_CENTI_SCALE,
          axis->acc_peak_x100 / SSF_MEMS_CENTI_SCALE,
          axis->acc_peak_x100 % SSF_MEMS_CENTI_SCALE,
          axis->acc_peak_to_peak_x100 / SSF_MEMS_CENTI_SCALE,
          axis->acc_peak_to_peak_x100 % SSF_MEMS_CENTI_SCALE,
          axis->high_freq_acc_rms_x100 / SSF_MEMS_CENTI_SCALE,
          axis->high_freq_acc_rms_x100 % SSF_MEMS_CENTI_SCALE,
          axis->velocity_rms_x100 / SSF_MEMS_CENTI_SCALE,
          axis->velocity_rms_x100 % SSF_MEMS_CENTI_SCALE,
          axis->low_freq_velocity_rms_x100 / SSF_MEMS_CENTI_SCALE,
          axis->low_freq_velocity_rms_x100 % SSF_MEMS_CENTI_SCALE);
}

/* 调试最新一批特征，不是瞬时原始加速度；定点输出，避免内核浮点运算。
 * 保留为独立接口，结束测试后仅移除 store_features 中的调用即可。
 * 无返回值，仅用于调试。 */
void ssf_mems_protocol_log_features(
    const struct ssf_mems_sensor_data *features) {
  if (features == NULL)
    return;

  ssf_mems_protocol_log_axis("X", &features->x);
  ssf_mems_protocol_log_axis("Y", &features->y);
  ssf_mems_protocol_log_axis("Z", &features->z);
  pr_info("ssf_mems: sound_rms=%u.%02u dB sound_peak=%u.%02u dB "
          "sound_peak_to_peak=%u.%02u dB startup_flags=0x%02x\n",
          features->sound.rms_db_x100 / SSF_MEMS_CENTI_SCALE,
          features->sound.rms_db_x100 % SSF_MEMS_CENTI_SCALE,
          features->sound.peak_db_x100 / SSF_MEMS_CENTI_SCALE,
          features->sound.peak_db_x100 % SSF_MEMS_CENTI_SCALE,
          features->sound.peak_to_peak_db_x100 / SSF_MEMS_CENTI_SCALE,
          features->sound.peak_to_peak_db_x100 % SSF_MEMS_CENTI_SCALE,
          (unsigned int)features->startup_flags);
}

/* 原子替换特征值缓存；未读取或删除的字段保持本轮初始化的零值。
 * 返回 0 成功，-EINVAL 设备或输入快照为空。 */
int
ssf_mems_protocol_store_features(struct ssf_mems_xyzs_data *data,
                                 const struct ssf_mems_sensor_data *features) {
  struct ssf_mems_protocol_state *state;

  if (data == NULL || features == NULL)
    return -EINVAL;
  state = &data->protocol;

  mutex_lock(&state->lock);
  state->features = *features;
  state->valid = true;
  mutex_unlock(&state->lock);
  /* 测试用：每次成功刷新打印一次，即使特征值与上次相同。
   * 停用时只删除/注释下面这一行，保留打印函数。 */
  ssf_mems_protocol_log_features(features);
  return 0;
}

/* 断线或过期时使缓存失效；无返回值，空设备直接退出，保留缓存内容。 */
void ssf_mems_protocol_invalidate_features(struct ssf_mems_xyzs_data *data) {
  if (data == NULL)
    return;
  mutex_lock(&data->protocol.lock);
  data->protocol.valid = false;
  mutex_unlock(&data->protocol.lock);
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

  if (serdev == NULL)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;
  state = &data->protocol;
  mutex_lock(&state->bus_lock);

  block = ssf_mems_modbus_find_block_cmd(SSF_BLOCK_READ_ALL_FEATURES);
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
        ssf_mems_protocol_next_feature(cursor, end);
    u16 count = 1;

    if (first == NULL)
      break;
    while (count < ARRAY_SIZE(registers) &&
           (u32)first->display_reg + count < end) {
      const struct ssf_reg_desc *reg =
          ssf_mems_modbus_find_reg(first->display_reg + count);

      if (reg == NULL || reg->feature_width == 0 || (reg->access & SSF_REG_READ) == 0 ||
          reg->read_format != block->rx_format ||
          reg->protocol_addr != (u32)first->protocol_addr + count)
        break;
      count++;
    }

    ret = ssf_mems_modbus_read_block_range_locked(
        serdev, block, first->display_reg, count, registers,
        ARRAY_SIZE(registers), timeout_ms);
    if (ret != 0)
      goto out_unlock;
    ret = ssf_mems_protocol_decode_feature_range(first->display_reg, registers,
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

/* 返回 0 缓存复制成功，-EINVAL 空设备/输出指针，-ENODEV 无驱动数据，-ENODATA
 * 尚无有效缓存。 */
int ssf_mems_protocol_get_features(struct serdev_device *serdev,
                                   struct ssf_mems_sensor_data *result) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_protocol_state *state;

  if (serdev == NULL || result == NULL)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;

  state = &data->protocol;
  mutex_lock(&state->lock);
  if (state->valid == false) {
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

  if (serdev == NULL || values == NULL)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;
  block = ssf_mems_modbus_find_block_cmd(SSF_BLOCK_WRITE_WORK_PARAMETERS);
  if (block == NULL)
    return -ENOENT;

  mutex_lock(&data->protocol.bus_lock);
  ret = ssf_mems_modbus_write_block_locked(serdev, block, values,
                                           values_count, timeout_ms);
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

/* 将寄存器枚举转换为串口波特率；非法枚举返回 -EINVAL。 */
int ssf_mems_baudrate_to_value(enum ssf_mems_baudrate baudrate) {
  if ((unsigned int)baudrate >= SSF_MEMS_BAUDRATE_MAX)
    return -EINVAL;

  return ssf_mems_baudrates_table[baudrate];
}

/* 将数值速率写入输出枚举，9600 对应编码 3；返回 0 成功，
 * -EINVAL 输出指针为空或数值速率不受支持。 */
int ssf_mems_baudrate_from_value(unsigned int value,
                                 enum ssf_mems_baudrate *baudrate) {
  unsigned int i;

  if (baudrate == NULL)
    return -EINVAL;
  for (i = SSF_MEMS_BAUDRATE_2400; i < SSF_MEMS_BAUDRATE_MAX; i++) {
    if (ssf_mems_baudrates_table[i] == value) {
      *baudrate = i;
      return 0;
    }
  }
  return -EINVAL;
}

/* 设置主机速率；返回 0 成功（控制器可能选择近似速率），-EINVAL 枚举非法，
 * -EIO 控制器不能设置速率；实际值记录到 host_baudrate。 */
static int
ssf_mems_protocol_set_host_baudrate(struct ssf_mems_xyzs_data *data,
                                    enum ssf_mems_baudrate baudrate) {
  unsigned int actual;
  int requested;

  requested = ssf_mems_baudrate_to_value(baudrate);
  if (requested < 0)
    return requested;

  actual = serdev_device_set_baudrate(data->serdev, requested);
  if (actual == 0)
    return -EIO;
  if (actual != requested)
    dev_dbg(&data->serdev->dev,
            "requested baudrate %d, controller selected %u\n", requested,
            actual);
  data->protocol.host_baudrate = actual;
  return 0;
}

/* 将固件枚举转换成 Linux serdev 的主机校验位；非法枚举不配置 UART。
 * 返回 0 主机校验位设置成功，-EINVAL 枚举非法；
 * 其他负 errno 来自串口控制器（如 -EOPNOTSUPP 不支持校验位设置）。 */
static int ssf_mems_protocol_set_host_parity(struct ssf_mems_xyzs_data *data,
                                           enum ssf_mems_parity parity) {
  enum serdev_parity host_parity;

  switch (parity) {
  case SSF_MEMS_PARITY_NONE:
    host_parity = SERDEV_PARITY_NONE;
    break;
  case SSF_MEMS_PARITY_ODD:
    host_parity = SERDEV_PARITY_ODD;
    break;
  case SSF_MEMS_PARITY_EVEN:
    host_parity = SERDEV_PARITY_EVEN;
    break;
  default:
    return -EINVAL;
  }

  return serdev_device_set_parity(data->serdev, host_parity);
}

/* 40060 的 FC03 是定长触发命令，无普通应答；只接收后续私有 XYZ 包。
 * 持 bus_lock 发送，避免普通请求执行器错误地等待 Modbus 回包。
 * 返回 0 启动帧完整发送，-ENOENT 40060 表项缺失，-EIO 串口短写；
 * 其他负值来自组帧（含义见 modbus.h）或串口发送。 */
static int ssf_mems_protocol_raw_command(struct ssf_mems_xyzs_data *data) {
  struct ssf_modbus_transfer transfer = {0};
  const struct ssf_reg_desc *reg = ssf_mems_modbus_find_reg(40060);
  u8 frame[8];
  int len, ret;

  if (reg == NULL)
    return -ENOENT;
  transfer.frame = ssf_mems_modbus_find_frame(SSF_MEMS_MODBUS_FUNC_READ);
  transfer.protocol_addr = reg->protocol_addr;
  transfer.reg_count = 1;
  len = ssf_mems_modbus_build_request(&transfer, data->slave_id,
                                      NULL, 0, frame, sizeof(frame));
  if (len < 0)
    return len;
  ret = serdev_device_write(data->serdev, frame, len, msecs_to_jiffies(200));
  if (ret < 0)
    return ret;
  serdev_device_wait_until_sent(data->serdev, msecs_to_jiffies(200));
  return ret == len ? 0 : -EIO;
}

/* 排空旧格式的候选帧，再在解析锁下切换接收器；不改变 UART 波特率。
 * 清理残留帧并切换原始接收器；无返回值，速率保持不变。 */
static void ssf_mems_protocol_raw_receiver(struct ssf_mems_xyzs_data *data,
                                          bool active) {
  if (READ_ONCE(data->raw.active) == active)
    return;
  ssf_mems_modbus_receive_flush(data);
  mutex_lock(&data->modbus_rx.parse_lock);
  data->raw.used = 0;
  WRITE_ONCE(data->raw.active, active);
  mutex_unlock(&data->modbus_rx.parse_lock);
}

/* 卸载只阻止下一批，并停止发布；已经发送的有限批次仍必须收尾。
 * 返回 true 采集或请求模块正在关闭，false 可以开始下一批。 */
static bool ssf_mems_protocol_raw_stopping(struct ssf_mems_xyzs_data *data) {
  return READ_ONCE(data->acquisition.stopping) == true ||
         READ_ONCE(data->modbus_req.shutting_down) == true;
}

/* 保持解析器运行，确认最后一个串口字节后至少安静 300 ms。
 * 返回 true 已确认连续静默 300 ms，false 在 1 s 预算内仍未静默。 */
static bool ssf_mems_protocol_raw_quiet(struct ssf_mems_xyzs_data *data) {
  unsigned long began = jiffies;
  unsigned long deadline = began + msecs_to_jiffies(1000);
  unsigned long quiet = msecs_to_jiffies(SSF_MEMS_RAW_QUIET_MS);

  do {
    unsigned long last = ssf_mems_modbus_receive_last_activity(data);

    if (time_before(last, began) == true)
      last = began;
    if (time_after_eq(jiffies, last + quiet) == true)
      return true;
    msleep(20);
  } while (time_before(jiffies, deadline) == true);
  return false;
}

/* 批次自然结束后只读确认 Modbus；异常/卸载也不发送 40059 或保存命令。
 * 未见末包时等满原始启动/上传预算，不能把首包前的安静误判为已经结束。
 * 只读验证失败则保留恢复门禁，禁止立即开始新批次或写入参数。
 * 返回 0 已恢复普通通信，-ETIMEDOUT 未静默/读回超时，
 * -EPROTO 波特率值非法；其他负值及含义同请求层的只读恢复接口。 */
static int ssf_mems_protocol_raw_finish(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_raw_state *raw = &data->raw;
  u16 reported;
  int ret;

  WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_RAW_RECOVERING);
  mutex_lock(&data->modbus_rx.parse_lock);
  WRITE_ONCE(raw->publishing, false);
  mutex_unlock(&data->modbus_rx.parse_lock);
  while (READ_ONCE(raw->batch_done) == false &&
         time_before(jiffies, data->protocol.raw_start_until) == true) {
    msleep(20);
  }
  if (ssf_mems_protocol_raw_quiet(data) == false)
    return -ETIMEDOUT;
  mutex_lock(&data->modbus_rx.parse_lock);
  if (raw->used != 0)
    WRITE_ONCE(raw->drain_crc_errors, raw->drain_crc_errors + 1U);
  mutex_unlock(&data->modbus_rx.parse_lock);
  ssf_mems_protocol_raw_receiver(data, false);
  ret = ssf_mems_modbus_read_reg_recovery_locked(data->serdev, 40102,
                                                &reported, 500);
  if (ret == 0 && reported >= SSF_MEMS_BAUDRATE_MAX)
    ret = -EPROTO;
  if (ret == 0) {
    WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_MODBUS);
  } else {
    ssf_mems_protocol_raw_receiver(data, true);
    dev_warn_ratelimited(&data->serdev->dev,
                         "finite batch completion unconfirmed: %d; read-only recovery, no stop/reset\n",
                         ret);
  }
  return ret;
}

/* 后台退避或卸载时收尾未确认批次；普通模式下不发送任何命令。
 * 返回 0 已确认/本来就是普通模式，-EINVAL 空设备，-ETIMEDOUT 无法确认静默，
 * -EPROTO 波特率寄存器异常；其他负值来自收尾的只读请求。 */
int ssf_mems_protocol_recover_raw(struct ssf_mems_xyzs_data *data) {
  int ret = 0;

  if (data == NULL || data->serdev == NULL)
    return -EINVAL;
  mutex_lock(&data->protocol.bus_lock);
  if (data->protocol.mode != SSF_MEMS_LINK_MODBUS)
    ret = ssf_mems_protocol_raw_finish(data);
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

/* 只配置 40061/40062 为零（固件默认 1024），不覆盖其余工作参数。
 * 普通请求模式下读、必要时写、再读回；长度寄存器本身不触发 Flash 保存。
 * 返回 0 已确认默认 1024 点长度，-EIO 长度读回不符；
 * 其他负值及含义来自请求层读写接口，不发送保存命令。 */
static int ssf_mems_protocol_raw_length(struct ssf_mems_xyzs_data *data) {
  const u16 zero[2] = {0, 0};
  u16 high, low;
  int ret;

  ret = ssf_mems_modbus_read_reg_locked(data->serdev, 40061, &high, 500);
  if (ret != 0)
    return ret;
  ret = ssf_mems_modbus_read_reg_locked(data->serdev, 40062, &low, 500);
  if (ret != 0 || (high == 0 && low == 0))
    return ret;
  ret = ssf_mems_modbus_write_regs_locked(data->serdev, 40061, 2,
                                          zero, ARRAY_SIZE(zero), 500);
  if (ret != 0)
    return ret;
  ret = ssf_mems_modbus_read_reg_locked(data->serdev, 40061, &high, 500);
  if (ret != 0)
    return ret;
  ret = ssf_mems_modbus_read_reg_locked(data->serdev, 40062, &low, 500);
  if (ret == 0 && (high != 0 || low != 0))
    ret = -EIO;
  return ret;
}

/* 窗口内重复完整的 1024 点批次，窗口到期不截断在途批次；全程独占总线。
 * 首包最长等 15 秒，整批额外保留线上时间和两秒余量；速率低也不误超时。
 * 返回 0 完整收完窗口，-EINVAL 参数非法，-EOPNOTSUPP 校验位/采样率不支持，
 * -ENOENT 必要表项缺失，-ENODEV 停止中，-ETIMEDOUT 启动/收尾超时，
 * -EILSEQ 批次不完整/包损坏，-ENOSPC IIO 丢样，-EPROTO 收尾寄存器异常，
 * -EIO 短写/配置读回不符，-ERESTARTSYS 等待被信号打断；其余为请求层错误。 */
int ssf_mems_protocol_collect_raw(struct ssf_mems_xyzs_data *data,
                                 unsigned int duration_ms) {
  struct ssf_mems_raw_state *raw;
  unsigned long window_until, started;
  unsigned int batch_ms;
  u16 sampling_index;
  int ret, finish_ret;
  long waited;

  if (data == NULL || data->serdev == NULL || duration_ms == 0)
    return -EINVAL;
  if (data->protocol.parity != SSF_MEMS_PARITY_NONE)
    return -EOPNOTSUPP;
  raw = &data->raw;
  mutex_lock(&data->protocol.bus_lock);
  if (data->protocol.mode != SSF_MEMS_LINK_MODBUS) {
    ret = ssf_mems_protocol_raw_finish(data);
    if (ret != 0)
      goto out;
  }
  if (ssf_mems_modbus_find_reg(40060) == NULL) {
    ret = -ENOENT;
    goto out;
  }
  ret = ssf_mems_modbus_read_reg_locked(data->serdev, 40052, &sampling_index, 500);
  if (ret != 0)
    goto out;
  if (sampling_index > 8U) {
    ret = -EOPNOTSUPP;
    goto out;
  }
  ret = ssf_mems_protocol_raw_length(data);
  if (ret != 0)
    goto out;
  if (ssf_mems_protocol_raw_stopping(data) == true) {
    ret = -ENODEV;
    goto out;
  }
  ssf_mems_protocol_invalidate_features(data);
  mutex_lock(&data->modbus_rx.parse_lock);
  WRITE_ONCE(raw->packets, 0);
  WRITE_ONCE(raw->discontinuities, 0);
  WRITE_ONCE(raw->crc_errors, 0);
  WRITE_ONCE(raw->buffer_errors, 0);
  WRITE_ONCE(raw->drain_packets, 0);
  WRITE_ONCE(raw->drain_discontinuities, 0);
  WRITE_ONCE(raw->drain_crc_errors, 0);
  WRITE_ONCE(raw->batches, 0);
  WRITE_ONCE(raw->ignored_bytes, 0);
  WRITE_ONCE(raw->start_wait_ms, 0);
  WRITE_ONCE(raw->stop_attempts, 0);
  WRITE_ONCE(raw->sampling_rate_index, sampling_index);
  mutex_unlock(&data->modbus_rx.parse_lock);
  batch_ms = SSF_MEMS_RAW_START_TIMEOUT_MS + 2000U +
      DIV_ROUND_UP(SSF_MEMS_RAW_BATCH_PACKETS * SSF_MEMS_RAW_PACKET_BYTES * 10000U,
                   max(1U, data->protocol.host_baudrate));
  window_until = jiffies + msecs_to_jiffies(duration_ms);
  do {
    ssf_mems_protocol_raw_receiver(data, true);
    mutex_lock(&data->modbus_rx.parse_lock);
    raw->have_sequence = false;
    raw->next_sequence = 0;
    WRITE_ONCE(raw->batch_packets, 0);
    WRITE_ONCE(raw->batch_done, false);
    WRITE_ONCE(raw->stream_seen, false);
    WRITE_ONCE(raw->publishing, true);
    mutex_unlock(&data->modbus_rx.parse_lock);
    msleep(max(5U, DIV_ROUND_UP(35000U,
                               max(1U, data->protocol.host_baudrate))));
    if (ssf_mems_protocol_raw_stopping(data) == true) {
      WRITE_ONCE(raw->publishing, false);
      ssf_mems_protocol_raw_receiver(data, false);
      ret = -ENODEV;
      break;
    }
    WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_RAW_STARTING);
    started = jiffies;
    data->protocol.raw_start_until = started + msecs_to_jiffies(batch_ms);
    ret = ssf_mems_protocol_raw_command(data);
    if (ret == 0) {
      waited = wait_event_interruptible_timeout(
          raw->waitq, READ_ONCE(raw->batch_packets) != 0 ||
                          ssf_mems_protocol_raw_stopping(data) == true,
          msecs_to_jiffies(SSF_MEMS_RAW_START_TIMEOUT_MS));
      WRITE_ONCE(raw->start_wait_ms, jiffies_to_msecs(jiffies - started));
      if (waited <= 0)
        ret = waited < 0 ? (int)waited : -ETIMEDOUT;
    }
    if (ret == 0 && ssf_mems_protocol_raw_stopping(data) == false &&
        READ_ONCE(raw->batch_done) == false) {
      WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_RAW_STREAMING);
      waited = wait_event_interruptible_timeout(
          raw->waitq, READ_ONCE(raw->batch_done) == true ||
                          ssf_mems_protocol_raw_stopping(data) == true,
          time_before(jiffies, data->protocol.raw_start_until) == true ?
              data->protocol.raw_start_until - jiffies : 1UL);
      if (waited <= 0)
        ret = waited < 0 ? (int)waited : -ETIMEDOUT;
    }
    if (ret != 0 || ssf_mems_protocol_raw_stopping(data) == true)
      WRITE_ONCE(raw->publishing, false);
    finish_ret = ssf_mems_protocol_raw_finish(data);
    if (finish_ret != 0)
      ret = finish_ret;
    if (ret == 0 && (raw->batch_packets != SSF_MEMS_RAW_BATCH_PACKETS ||
                    raw->discontinuities != 0 || raw->crc_errors != 0 ||
                    raw->drain_discontinuities != 0 || raw->drain_crc_errors != 0 ||
                    raw->ignored_bytes != 0))
      ret = -EILSEQ;
    if (ret == 0 && raw->buffer_errors != 0)
      ret = -ENOSPC;
    if (ret == 0)
      WRITE_ONCE(raw->batches, raw->batches + 1U);
    if (ssf_mems_protocol_raw_stopping(data) == true)
      ret = -ENODEV;
    if (ret != 0)
      break;
  } while (time_before(jiffies, window_until) == true);
  dev_info(&data->serdev->dev,
           "finite raw capture: %u batches, %u packets, %u sequence gaps, %u bad candidates, %u IIO drops; drain %u/%u/%u, ignored %u, start %u ms, status %d\n",
           raw->batches, raw->packets, raw->discontinuities, raw->crc_errors,
           raw->buffer_errors, raw->drain_packets, raw->drain_discontinuities,
           raw->drain_crc_errors, raw->ignored_bytes, raw->start_wait_ms, ret);
out:
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

/* 仅配置启动时的主机 UART，不修改传感器的暂存或持久化参数。
 * 返回 0 成功，-EINVAL 设备/枚举非法，-EIO 控制器无法设置速率；
 * 其他负 errno 来自 serdev 校验位设置（如 -EOPNOTSUPP 控制器不支持）。 */
int ssf_mems_protocol_configure_serial(struct ssf_mems_xyzs_data *data) {
  int ret;

  if (data == NULL || data->serdev == NULL ||
      (unsigned int)data->protocol.parity >= SSF_MEMS_PARITY_MAX)
    return -EINVAL;

  mutex_lock(&data->protocol.bus_lock);
  ret = ssf_mems_protocol_set_host_baudrate(data, data->protocol.baudrate);
  if (ret == -EIO && data->protocol.baudrate != SSF_MEMS_BAUDRATE_DEFAULT &&
      data->protocol.baudrate != SSF_MEMS_BAUDRATE_9600) {
    dev_info(&data->serdev->dev,
             "controller rejected startup baud rate %d, using default baud rate 9600\n",
             ssf_mems_baudrate_to_value(data->protocol.baudrate));
    data->protocol.baudrate = SSF_MEMS_BAUDRATE_DEFAULT;
    ret = ssf_mems_protocol_set_host_baudrate(data, data->protocol.baudrate);
  }
  if (ret != 0)
    goto out_unlock;
  serdev_device_set_flow_control(data->serdev, false);
  ret = ssf_mems_protocol_set_host_parity(data, data->protocol.parity);

out_unlock:
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

/* 只尝试一次读取 40102，并校验寄存器值是受支持的枚举。
 * 返回 0 读取成功，-EPROTO 寄存器枚举非法；其他负值同单寄存器读取。 */
static int ssf_mems_protocol_read_baudrate_once(
    struct serdev_device *serdev, enum ssf_mems_baudrate *baudrate,
    unsigned int timeout_ms) {
  u16 value;
  int ret;

  ret = ssf_mems_modbus_read_reg_locked(serdev, 40102, &value, timeout_ms);
  if (ret != 0)
    return ret;
  if (value >= SSF_MEMS_BAUDRATE_MAX)
    return -EPROTO;

  *baudrate = value;
  return 0;
}

/* 判断某个候选波特率是否已经尝试过，避免重复扫描 */
/* 返回 true 实际速率已由入口/更早枚举覆盖，false 尚未尝试，避免重复扫描 9600。 */
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

/* 调用者持 bus_lock，且已收到保存应答并切换主机 UART。
 * 固件的 Flash 操作和启动初始化耗时不固定，允许超时后只读重试。
 * 不能重发 40102/40110；异常应答、非法值、信号和卸载不作为启动慢处理。
 * 返回 0 新速率验证通过，-ENODEV 关闭中，-EPROTO 读回值非法/不符，
 * -ETIMEDOUT 所有验证均超时；其他负值同单寄存器读取，不重发保存命令。 */
static int ssf_mems_protocol_verify_baudrate_after_reboot(
    struct ssf_mems_xyzs_data *data, enum ssf_mems_baudrate requested,
    enum ssf_mems_baudrate *verified, unsigned int timeout_ms) {
  enum ssf_mems_baudrate found;
  unsigned int attempt;
  int ret;

  for (attempt = 1; attempt <= SSF_MEMS_BAUDRATE_VERIFY_ATTEMPTS; attempt++) {
    if (READ_ONCE(data->modbus_req.shutting_down) == true)
      return -ENODEV;
    ret = ssf_mems_protocol_read_baudrate_once(data->serdev, &found, timeout_ms);
    if (ret == 0) {
      if (ssf_mems_baudrates_table[found] !=
          ssf_mems_baudrates_table[requested])
        return -EPROTO;
      *verified = found;
      return 0;
    }
    if (ret != -ETIMEDOUT || attempt == SSF_MEMS_BAUDRATE_VERIFY_ATTEMPTS)
      return ret;
    dev_info(&data->serdev->dev,
             "baud change: verification %u/%u timed out at host %u; retrying read-only 40102\n",
             attempt, SSF_MEMS_BAUDRATE_VERIFY_ATTEMPTS,
             data->protocol.host_baudrate);
    /* 短等待可快速响应卸载；迟到应答的隔离仍由请求层保证。 */
    msleep(SSF_MEMS_BAUDRATE_VERIFY_RETRY_MS);
  }
  return -ETIMEDOUT;
}

/* 普通读回证明候选速率可通信；暂存寄存器不一定已经生效。
 * 旧连续模式遗留的 1 Mbaud 不属于定长协议，拒绝自动保存或反复停流。
 * 返回 0 接受已响应的速率，-EPROTO 疑似旧的 1 Mbaud 连续模式；不写传感器。 */
static int ssf_mems_protocol_accept_baudrate(
    struct ssf_mems_xyzs_data *data, enum ssf_mems_baudrate candidate,
    enum ssf_mems_baudrate reported, enum ssf_mems_baudrate *accepted) {
  if (candidate == SSF_MEMS_BAUDRATE_1000000 &&
      ssf_mems_baudrates_table[reported] != 1000000) {
    dev_warn(&data->serdev->dev,
             "possible legacy continuous mode at 1000000; refusing automatic save/reset\n");
    return -EPROTO;
  }
  *accepted = ssf_mems_baudrates_table[reported] ==
                      ssf_mems_baudrates_table[candidate] ? reported : candidate;
  return 0;
}

/* 固件先暂存 40102；40110=1 的旧速率回显后保存并重启才真正生效。
 * 返回 0 已确认新速率，-EINVAL 参数非法，-ENODEV 无设备/停止中，
 * -EPROTO 读回速率异常/与目标不符，-EIO 控制器拒绝配置；
 * 其余错误见 modbus_request.h 的请求返回值约定。保存后失败不重复写 Flash。 */
int ssf_mems_protocol_set_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate baudrate,
                                   unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_modbus_transfer planned;
  enum ssf_mems_baudrate found;
  int ret;

  if (serdev == NULL || (unsigned int)baudrate >= SSF_MEMS_BAUDRATE_MAX)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;

  /* 保存命令不可用时，不先修改传感器的暂存值。 */
  ret = ssf_mems_modbus_plan_request(SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE,
                                     40102, 1, NULL, &planned);
  if (ret != 0)
    return ret;
  ret = ssf_mems_modbus_plan_request(SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE,
                                     40110, 1, NULL, &planned);
  if (ret != 0)
    return ret;

  mutex_lock(&data->protocol.bus_lock);
  dev_info(&serdev->dev,
           "baud change: host %u, requested %d; staging 40102=%u\n",
           data->protocol.host_baudrate,
           ssf_mems_baudrate_to_value(baudrate), (unsigned int)baudrate);
  ret = ssf_mems_modbus_write_reg_locked(serdev, 40102, baudrate,
                                         timeout_ms);
  if (ret != 0) {
    dev_err_ratelimited(&serdev->dev,
                        "baud change failed at 40102 staging: %d (host %u, requested %d)\n",
                        ret, data->protocol.host_baudrate,
                        ssf_mems_baudrate_to_value(baudrate));
    goto out_unlock;
  }

  ssf_mems_protocol_invalidate_features(data);
  dev_info(&serdev->dev,
           "baud change: 40102 acknowledged; saving/rebooting via 40110=1 at host %u\n",
           data->protocol.host_baudrate);
  ret = ssf_mems_modbus_write_reg_locked(serdev, 40110, 1, timeout_ms);
  if (ret != 0) {
    /* 超时/部分发送时不能排除命令已生效；即使失败也留足重启静默期。
     * 不把丢失的保存应答当成功，不自动重发 Flash 保存命令。 */
    if (ret != -EREMOTEIO && ret != -ENOMEM && ret != -ENODEV)
      msleep(SSF_MEMS_SENSOR_REBOOT_DELAY_MS);
    dev_err_ratelimited(&serdev->dev,
                        "failed to confirm config save/reboot at 40110: %d (host %u, requested %d)\n",
                        ret, data->protocol.host_baudrate,
                        ssf_mems_baudrate_to_value(baudrate));
    goto out_unlock;
  }

  /* 不可被信号提前打断而让其他请求过早发送；持 bus_lock，不持 req.lock。 */
  msleep(SSF_MEMS_SENSOR_REBOOT_DELAY_MS);
  if (READ_ONCE(data->modbus_req.shutting_down) == true) {
    ret = -ENODEV;
    goto out_unlock;
  }
  ssf_mems_modbus_receive_flush(data);
  ret = ssf_mems_protocol_set_host_baudrate(data, baudrate);
  if (ret != 0) {
    dev_err_ratelimited(&serdev->dev,
                        "baud change failed to configure host UART: %d (requested %d)\n",
                        ret, ssf_mems_baudrate_to_value(baudrate));
    goto out_unlock;
  }

  /* 保存已经确认，不能在验证失败时假装仍使用旧速率。 */
  data->protocol.baudrate = baudrate;
  dev_info(&serdev->dev,
           "baud change: verifying 40102 after reboot at host %u\n",
           data->protocol.host_baudrate);
  ret = ssf_mems_protocol_verify_baudrate_after_reboot(
      data, baudrate, &found, timeout_ms);
  if (ret != 0)
    dev_err_ratelimited(&serdev->dev,
                        "failed to verify baud rate after reboot: %d (host %u, requested %d)\n",
                        ret, data->protocol.host_baudrate,
                        ssf_mems_baudrate_to_value(baudrate));
  else {
    data->protocol.baudrate = found;
    dev_info(&serdev->dev, "baud change verified: sensor %d, host %u\n",
             ssf_mems_baudrate_to_value(found), data->protocol.host_baudrate);
  }

out_unlock:
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

/* 先以当前速率读取；仅当没有可匹配应答而超时时扫描其他枚举速率。
 * 返回 0 匹配成功，-EINVAL 参数非法，-ENODEV 无设备/停止中，
 * -EPROTO 寄存器值异常/疑似旧连续模式，-EIO 主机调速失败，
 * -ETIMEDOUT 所有候选均无应答；其余错误见 modbus_request.h 的请求返回值约定。 */
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

  if (serdev == NULL || baudrate == NULL)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;
  state = &data->protocol;

  mutex_lock(&state->bus_lock);
  if (state->mode != SSF_MEMS_LINK_MODBUS) {
    ret = ssf_mems_protocol_raw_finish(data);
    if (ret != 0)
      goto out_unlock;
  }
  initial = state->baudrate;
  if ((unsigned int)initial >= SSF_MEMS_BAUDRATE_MAX) {
    ret = -EINVAL;
    goto out_unlock;
  }

  ret = ssf_mems_protocol_read_baudrate_once(serdev, &found, timeout_ms);
  if (ret == 0) {
    /* 成功应答证明当前主机速率可通信，40102 可能仍是尚未保存的值。 */
    ret = ssf_mems_protocol_accept_baudrate(data, initial, found, &found);
    if (ret != 0)
      goto out_unlock;
    state->baudrate = found;
    *baudrate = found;
    goto out_unlock;
  }
  if (ret != -ETIMEDOUT)
    goto out_unlock;

  for (i = 0; i < SSF_MEMS_BAUDRATE_MAX; i++) {
    enum ssf_mems_baudrate candidate = i;

    if (ssf_mems_protocol_baudrate_tried(candidate, initial) == true)
      continue;

    ssf_mems_modbus_receive_flush(data);
    ret = ssf_mems_protocol_set_host_baudrate(data, candidate);
    if (ret != 0)
      goto restore_initial;

    ret = ssf_mems_protocol_read_baudrate_once(serdev, &found, timeout_ms);
    if (ret == 0) {
      ret = ssf_mems_protocol_accept_baudrate(data, candidate, found, &found);
      if (ret != 0)
        goto restore_initial;
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
  if (restore_ret != 0)
    ret = restore_ret;

out_unlock:
  mutex_unlock(&state->bus_lock);
  return ret;
}
