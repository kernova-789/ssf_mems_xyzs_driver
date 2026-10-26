#include "protocol.h"
#include "core.h"
#include "modbus/modbus.h"
#include "modbus/modbus_receive.h"
#include "modbus/modbus_request.h"
#include "model/modbus_table.h"
#include "raw/raw_capture.h"

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/property.h>
#include <linux/string.h>

/* 读取设备树中的启动通信参数。
 * 返回：0 成功或使用默认值，-EINVAL 从站地址或校验位无效。
 */
static int ssf_mems_protocol_parse_properties(struct ssf_mems_data *data);

/* 高层配置项对应表项的稳定名称；具体地址由寄存器表决定。 */
static const char * const ssf_mems_setting_registers[SSF_MEMS_SETTING_MAX] = {
    [SSF_MEMS_SETTING_SAMPLING_RATE] = "sampling_rate",
    [SSF_MEMS_SETTING_SAMPLING_LENGTH] = "sampling_length",
    [SSF_MEMS_SETTING_PARAMETER_SWITCH] = "parameter_switch",
    [SSF_MEMS_SETTING_FEATURE_ENABLE] = "parameter_switch",
    [SSF_MEMS_SETTING_FIRMWARE_VERSION] = "firmware_version",
};

/* 读取一个传感器配置项。
 * 返回：0 成功，负数为参数、缺失表项或通信错误。
 */
int ssf_mems_protocol_read_setting(struct serdev_device *serdev,
                                  enum ssf_mems_sensor_setting setting,
                                  u16 *value, unsigned int timeout_ms) {
  struct ssf_mems_data *data;
  u16 reg_value;
  int ret;

  if (serdev == NULL || value == NULL ||
      (unsigned int)setting >= SSF_MEMS_SETTING_MAX)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;
  if (ssf_mems_sensor_find_reg_by_name(
          data->sensor, ssf_mems_setting_registers[setting]) == NULL)
    return -EOPNOTSUPP;
  mutex_lock(&data->protocol.bus_lock);
  ret = ssf_mems_modbus_read_reg_by_name_locked(
      serdev, ssf_mems_setting_registers[setting], &reg_value, timeout_ms);
  mutex_unlock(&data->protocol.bus_lock);
  if (ret == 0) {
    if (setting == SSF_MEMS_SETTING_FEATURE_ENABLE)
      reg_value = (reg_value & data->sensor->profile.feature_enable_mask) >>
                  data->sensor->profile.feature_enable_shift;
    *value = reg_value;
  }
  return ret;
}

/* 修改采样率、普通采样长度或特征使能配置。
 * 返回：0 成功或无需修改，负数为参数、通信或读回错误。
 * 同值不重复写入，写后读回并保留原有报警位。
 */
int ssf_mems_protocol_write_setting(struct serdev_device *serdev,
                                   enum ssf_mems_sensor_setting setting,
                                   u16 value, unsigned int timeout_ms) {
  struct ssf_mems_data *data;
  const struct ssf_reg_desc *reg;
  u16 reg_value, desired, verified;
  int ret;

  if (serdev == NULL ||
      (setting != SSF_MEMS_SETTING_SAMPLING_RATE &&
       setting != SSF_MEMS_SETTING_SAMPLING_LENGTH &&
       setting != SSF_MEMS_SETTING_FEATURE_ENABLE))
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL || data->sensor == NULL)
    return -ENODEV;
  if (setting == SSF_MEMS_SETTING_FEATURE_ENABLE &&
      data->sensor->profile.feature_enable_mask == 0)
    return -EOPNOTSUPP;
  if ((setting == SSF_MEMS_SETTING_SAMPLING_RATE &&
       value >= data->sensor->profile.sampling_frequency_count) ||
      (setting == SSF_MEMS_SETTING_SAMPLING_LENGTH &&
       value >= data->sensor->profile.sampling_length_count) ||
      (setting == SSF_MEMS_SETTING_FEATURE_ENABLE &&
       value > (data->sensor->profile.feature_enable_mask >>
                data->sensor->profile.feature_enable_shift)))
    return -EINVAL;
  reg = ssf_mems_sensor_find_reg_by_name(data->sensor, ssf_mems_setting_registers[setting]);
  if (reg == NULL)
    return -ENOENT;
  mutex_lock(&data->protocol.bus_lock);
  ret = ssf_mems_modbus_read_reg_locked(serdev, reg->display_reg, &reg_value,
                                       timeout_ms);
  if (ret != 0)
    goto out;
  /* 按表中掩码替换特征使能，保留其他位；同值不写。 */
  desired = value;
  if (setting == SSF_MEMS_SETTING_FEATURE_ENABLE)
    desired = (reg_value & ~data->sensor->profile.feature_enable_mask) |
              (value << data->sensor->profile.feature_enable_shift);
  if (desired == reg_value)
    goto out;
  ret = ssf_mems_modbus_write_reg_locked(serdev, reg->display_reg, desired,
                                        timeout_ms);
  if (ret != 0)
    goto out;
  ret = ssf_mems_modbus_read_reg_locked(serdev, reg->display_reg, &verified,
                                       timeout_ms);
  if (ret == 0 && verified != desired)
    ret = -EIO;
out:
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

/* 初始化协议状态并读取启动配置。
 * 返回：0 成功，-EINVAL 设备或启动配置无效。
 * 只初始化状态，不发送串口命令。
 */
int ssf_mems_protocol_init(struct ssf_mems_data *data) {
  struct ssf_mems_protocol_state *state;
  const char *bad_name;
  int ret;

  if (data == NULL || data->serdev == NULL)
    return -EINVAL;
  ret = ssf_mems_sensor_validate(data->sensor, &bad_name);
  if (ret != 0) {
    dev_err(&data->serdev->dev, "invalid sensor table: %s\n", bad_name);
    return ret;
  }
  data->slave_id = SSF_MEMS_MODBUS_DEFAULT_SLAVE_ID;
  state = &data->protocol;
  state->baudrate = SSF_MEMS_BAUDRATE_DEFAULT;
  /* UART 配置前是启动提示值，configure_serial 后才是控制器实际值。 */
  state->host_baudrate =
      data->sensor->profile.baudrates[SSF_MEMS_BAUDRATE_DEFAULT];
  state->parity = SSF_MEMS_PARITY_NONE;
  state->mode = SSF_MEMS_LINK_MODBUS;
  state->raw_start_until = jiffies;
  mutex_init(&state->lock);
  mutex_init(&state->bus_lock);
  memset(&state->features, 0, sizeof(state->features));
  state->valid = false;
  ssf_mems_raw_init(data);
  ssf_mems_sound_init(data);
  ssf_mems_ota_init(data);
  return ssf_mems_protocol_parse_properties(data);
}

/* 读取设备树中的启动通信参数。
 * 返回：0 成功或使用默认值，-EINVAL 从站地址或校验位无效。
 */
static int ssf_mems_protocol_parse_properties(struct ssf_mems_data *data) {
  struct device *dev = &data->serdev->dev;
  const char *parity;
  enum ssf_mems_baudrate baudrate;
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
    ret = ssf_mems_baudrate_from_value(data->sensor, value, &baudrate);
    if (ret != 0) {
      dev_info(dev, "unsupported current-speed %u, using default baud rate 9600\n",
               value);
    } else {
      /* 默认值统一在 init 中设置；仅合法属性才能覆盖。 */
      data->protocol.baudrate = baudrate;
      data->protocol.host_baudrate = ssf_mems_baudrate_to_value(data->sensor, baudrate);
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

/* 把完整 Modbus 帧交给请求层匹配。
 * 返回：0 已处理或丢弃，-EINVAL 参数为空。
 * 帧数据只在本次调用期间有效。
 */
int ssf_mems_protocol_handle_frame(void *context, const u8 *buf, size_t len) {
  struct ssf_mems_data *data = context;

  if (data == NULL || buf == NULL)
    return -EINVAL;
  ssf_mems_modbus_claim_frame(data, buf, len);
  return 0;
}

/* 替换最新的特征缓存。
 * 返回：0 成功，-EINVAL 参数为空。
 * 加锁更新，避免读到部分新数据。
 */
int
ssf_mems_protocol_store_features(struct ssf_mems_data *data,
                                 const struct ssf_mems_sensor_data *features) {
  struct ssf_mems_protocol_state *state;

  if (data == NULL || features == NULL)
    return -EINVAL;
  state = &data->protocol;

  mutex_lock(&state->lock);
  state->features = *features;
  state->valid = true;
  mutex_unlock(&state->lock);
  return 0;
}

/* 把特征缓存标记为无效。
 * 返回：无。
 */
void ssf_mems_protocol_invalidate_features(struct ssf_mems_data *data) {
  if (data == NULL)
    return;
  mutex_lock(&data->protocol.lock);
  data->protocol.valid = false;
  mutex_unlock(&data->protocol.lock);
}

/* 复制最新的有效特征缓存。
 * 返回：0 成功，-ENODATA 无有效缓存，其他负数为参数或设备错误。
 */
int ssf_mems_protocol_get_features(struct serdev_device *serdev,
                                   struct ssf_mems_sensor_data *result) {
  struct ssf_mems_data *data;
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

/* 写入块表规定的十个工作参数。
 * 返回：0 成功，负数为参数、表项或通信错误。
 */
int ssf_mems_protocol_write_work_parameters(struct serdev_device *serdev,
                                            const u16 *values,
                                            size_t values_count,
                                            unsigned int timeout_ms) {
  struct ssf_mems_data *data;
  const struct ssf_block_cmd_desc *block;
  int ret;

  if (serdev == NULL || values == NULL)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;
  block = ssf_mems_sensor_find_block(data->sensor, SSF_BLOCK_WRITE_WORK_PARAMETERS);
  if (block == NULL)
    return -ENOENT;

  mutex_lock(&data->protocol.bus_lock);
  ret = ssf_mems_modbus_write_block_locked(serdev, block, values,
                                           values_count, timeout_ms);
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

/* 把波特率编码转换为 bit/s 数值。
 * 返回：正数为波特率，-EINVAL 编码无效。
 */
int ssf_mems_baudrate_to_value(const struct ssf_mems_sensor *sensor, enum ssf_mems_baudrate baudrate) {
  if (sensor == NULL || (unsigned int)baudrate >= SSF_MEMS_BAUDRATE_MAX ||
      (unsigned int)baudrate >= sensor->profile.baudrate_count)
    return -EINVAL;

  return sensor->profile.baudrates[baudrate];
}

/* 把 bit/s 数值转换为波特率编码。
 * 返回：0 成功，-EINVAL 参数或速率无效。
 * 9600 对应显式编码 3。
 */
int ssf_mems_baudrate_from_value(const struct ssf_mems_sensor *sensor, unsigned int value,
                                 enum ssf_mems_baudrate *baudrate) {
  unsigned int i;

  if (sensor == NULL || baudrate == NULL)
    return -EINVAL;
  for (i = SSF_MEMS_BAUDRATE_2400; i < SSF_MEMS_BAUDRATE_MAX &&
       i < sensor->profile.baudrate_count; i++) {
    if (sensor->profile.baudrates[i] == value) {
      *baudrate = i;
      return 0;
    }
  }
  return -EINVAL;
}

/* 设置主机 UART 波特率。
 * 返回：0 成功，-EINVAL 编码无效，-EIO 控制器设置失败。
 * 把控制器实际速率记录到 host_baudrate。
 */
static int
ssf_mems_protocol_set_host_baudrate(struct ssf_mems_data *data,
                                    enum ssf_mems_baudrate baudrate) {
  unsigned int actual;
  int requested;

  requested = ssf_mems_baudrate_to_value(data->sensor, baudrate);
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

/* 设置主机 UART 校验位。
 * 返回：0 成功，负数为枚举或控制器错误。
 */
static int ssf_mems_protocol_set_host_parity(struct ssf_mems_data *data,
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

/* 按启动配置设置主机 UART。
 * 返回：0 成功，负数为参数或控制器错误。
 * 非默认启动速率被拒绝时回退到 9600。
 */
int ssf_mems_protocol_configure_serial(struct ssf_mems_data *data) {
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
             ssf_mems_baudrate_to_value(data->sensor, data->protocol.baudrate));
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

/* 读取一次传感器波特率编码。
 * 返回：0 成功，-EPROTO 编码无效，其他负数为读取错误。
 * 调用者须持 bus_lock。
 */
static int ssf_mems_protocol_read_baudrate_once(
    struct serdev_device *serdev, enum ssf_mems_baudrate *baudrate,
    unsigned int timeout_ms) {
  struct ssf_mems_data *data = serdev_device_get_drvdata(serdev);
  u16 value;
  int ret;

  ret = ssf_mems_modbus_read_reg_by_name_locked(serdev, "baudrate", &value,
                                               timeout_ms);
  if (ret != 0)
    return ret;
  if (ssf_mems_baudrate_to_value(data->sensor, value) < 0)
    return -EPROTO;

  *baudrate = value;
  return 0;
}

/* 检查候选波特率是否已经尝试过。
 * 返回：true 已尝试，false 未尝试。
 * 按 bit/s 比较，避免重复扫描两个 9600 编码。
 */
static bool ssf_mems_protocol_baudrate_tried(
    struct ssf_mems_data *data, enum ssf_mems_baudrate candidate, enum ssf_mems_baudrate initial) {
  unsigned int i;
  int value = data->sensor->profile.baudrates[candidate];

  if (value == data->sensor->profile.baudrates[initial])
    return true;
  for (i = 0; i < (unsigned int)candidate; i++) {
    if (data->sensor->profile.baudrates[i] == value &&
        data->sensor->profile.baudrates[i] != data->sensor->profile.baudrates[initial])
      return true;
  }
  return false;
}

/* 确认重启后的波特率与目标一致。
 * 返回：0 一致，-EPROTO 编码异常或不符，其他负数为读取错误。
 * 调用者持 bus_lock；仅超时重试读取，不重复保存。
 */
static int ssf_mems_protocol_verify_baudrate_after_reboot(
    struct ssf_mems_data *data, enum ssf_mems_baudrate requested,
    enum ssf_mems_baudrate *verified, unsigned int timeout_ms) {
  enum ssf_mems_baudrate found;
  unsigned int attempt;
  int ret;

  for (attempt = 1; attempt <= SSF_MEMS_BAUDRATE_VERIFY_ATTEMPTS; attempt++) {
    if (READ_ONCE(data->modbus_req.shutting_down) == true)
      return -ENODEV;
    ret = ssf_mems_protocol_read_baudrate_once(data->serdev, &found, timeout_ms);
    if (ret == 0) {
      if (data->sensor->profile.baudrates[found] !=
          data->sensor->profile.baudrates[requested])
        return -EPROTO;
      *verified = found;
      return 0;
    }
    if (ret != -ETIMEDOUT || attempt == SSF_MEMS_BAUDRATE_VERIFY_ATTEMPTS)
      return ret;
    dev_info(&data->serdev->dev,
             "baud change: verification %u/%u timed out at host %u; retrying read-only baudrate\n",
             attempt, SSF_MEMS_BAUDRATE_VERIFY_ATTEMPTS,
             data->protocol.host_baudrate);
    /* 短等待可快速响应卸载；迟到应答的隔离仍由请求层保证。 */
    msleep(SSF_MEMS_BAUDRATE_VERIFY_RETRY_MS);
  }
  return -ETIMEDOUT;
}

/* 选择已经验证可通信的波特率编码。
 * 返回：最终采用的编码。
 * 速率相同时保留 reported，不同时采用 candidate。
 */
static enum ssf_mems_baudrate ssf_mems_protocol_accept_baudrate(
    struct ssf_mems_data *data, enum ssf_mems_baudrate candidate, enum ssf_mems_baudrate reported) {
  return data->sensor->profile.baudrates[reported] ==
                 data->sensor->profile.baudrates[candidate] ? reported : candidate;
}

/* 修改并验证传感器通信波特率。
 * 返回：0 新速率已确认，负数为参数、通信或验证错误。
 * 旧速率下暂存并保存重启，再切换主机验证；不重发保存。
 */
int ssf_mems_protocol_set_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate baudrate,
                                   unsigned int timeout_ms) {
  struct ssf_mems_data *data;
  struct ssf_modbus_transfer planned;
  const struct ssf_reg_desc *baud_reg;
  const struct ssf_reg_desc *save_reg;
  enum ssf_mems_baudrate found;
  int ret;

  if (serdev == NULL)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;

  if (ssf_mems_baudrate_to_value(data->sensor, baudrate) < 0)
    return -EINVAL;
  baud_reg = ssf_mems_sensor_find_reg_by_name(data->sensor, "baudrate");
  save_reg = ssf_mems_sensor_find_reg_by_name(data->sensor, "save_config_and_reboot");
  if (baud_reg == NULL || save_reg == NULL)
    return -ENOENT;
  /* 保存命令不可用时，不先修改传感器的暂存值。 */
  ret = ssf_mems_modbus_plan_request(data->sensor, SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE,
                                     baud_reg->display_reg, 1, NULL, &planned);
  if (ret != 0)
    return ret;
  ret = ssf_mems_modbus_plan_request(data->sensor, SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE,
                                     save_reg->display_reg, 1, NULL, &planned);
  if (ret != 0)
    return ret;

  mutex_lock(&data->protocol.bus_lock);
  /* 同值也读回确认，防止把主机缓存或未保存的暂存值当作传感器配置。 */
  if (data->protocol.host_baudrate == ssf_mems_baudrate_to_value(data->sensor, baudrate) &&
      ssf_mems_baudrate_to_value(data->sensor, data->protocol.baudrate) ==
          ssf_mems_baudrate_to_value(data->sensor, baudrate)) {
    ret = ssf_mems_protocol_read_baudrate_once(serdev, &found, timeout_ms);
    if (ret != 0)
      goto out_unlock;
    if (ssf_mems_baudrate_to_value(data->sensor, found) == ssf_mems_baudrate_to_value(data->sensor, baudrate)) {
      data->protocol.baudrate = found;
      goto out_unlock;
    }
  }
  dev_info(&serdev->dev,
           "baud change: host %u, requested %d; staging %u=%u\n",
           data->protocol.host_baudrate,
           ssf_mems_baudrate_to_value(data->sensor, baudrate), baud_reg->display_reg,
           (unsigned int)baudrate);
  ret = ssf_mems_modbus_write_reg_locked(serdev, baud_reg->display_reg, baudrate,
                                         timeout_ms);
  if (ret != 0) {
    dev_err_ratelimited(&serdev->dev,
                        "baud change failed at %u staging: %d (host %u, requested %d)\n",
                        baud_reg->display_reg, ret, data->protocol.host_baudrate,
                        ssf_mems_baudrate_to_value(data->sensor, baudrate));
    goto out_unlock;
  }

  ssf_mems_protocol_invalidate_features(data);
  dev_info(&serdev->dev,
           "baud change: %u acknowledged; saving/rebooting via %u=1 at host %u\n",
           baud_reg->display_reg, save_reg->display_reg, data->protocol.host_baudrate);
  ret = ssf_mems_modbus_write_reg_locked(serdev, save_reg->display_reg, 1, timeout_ms);
  if (ret != 0) {
    /* 超时/部分发送时不能排除命令已生效；即使失败也留足重启静默期。
     * 不把丢失的保存应答当成功，不自动重发 Flash 保存命令。 */
    if (ret != -EREMOTEIO && ret != -ENOMEM && ret != -ENODEV)
      msleep(SSF_MEMS_SENSOR_REBOOT_DELAY_MS);
    dev_err_ratelimited(&serdev->dev,
                        "failed to confirm config save/reboot at %u: %d (host %u, requested %d)\n",
                        save_reg->display_reg, ret, data->protocol.host_baudrate,
                        ssf_mems_baudrate_to_value(data->sensor, baudrate));
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
                        ret, ssf_mems_baudrate_to_value(data->sensor, baudrate));
    goto out_unlock;
  }

  /* 保存已经确认，不能在验证失败时假装仍使用旧速率。 */
  data->protocol.baudrate = baudrate;
  dev_info(&serdev->dev,
           "baud change: verifying %u after reboot at host %u\n",
           baud_reg->display_reg, data->protocol.host_baudrate);
  ret = ssf_mems_protocol_verify_baudrate_after_reboot(
      data, baudrate, &found, timeout_ms);
  if (ret != 0)
    dev_err_ratelimited(&serdev->dev,
                        "failed to verify baud rate after reboot: %d (host %u, requested %d)\n",
                        ret, data->protocol.host_baudrate,
                        ssf_mems_baudrate_to_value(data->sensor, baudrate));
  else {
    data->protocol.baudrate = found;
    dev_info(&serdev->dev, "baud change verified: sensor %d, host %u\n",
             ssf_mems_baudrate_to_value(data->sensor, found), data->protocol.host_baudrate);
  }

out_unlock:
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

/* 确认传感器当前可通信的波特率。
 * 返回：0 匹配成功，负数为参数、通信或编码错误。
 * 当前速率超时才扫描其他速率，失败时尝试恢复入口速率。
 */
int ssf_mems_protocol_get_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate *baudrate,
                                   unsigned int timeout_ms) {
  struct ssf_mems_data *data;
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
    ret = ssf_mems_raw_finish_locked(data);
    if (ret != 0)
      goto out_unlock;
  }
  initial = state->baudrate;
  if (ssf_mems_baudrate_to_value(data->sensor, initial) < 0) {
    ret = -EINVAL;
    goto out_unlock;
  }

  ret = ssf_mems_protocol_read_baudrate_once(serdev, &found, timeout_ms);
  if (ret == 0) {
    /* 成功应答证明当前主机速率可通信，40102 可能仍是尚未保存的值。 */
    found = ssf_mems_protocol_accept_baudrate(data, initial, found);
    state->baudrate = found;
    *baudrate = found;
    goto out_unlock;
  }
  if (ret != -ETIMEDOUT)
    goto out_unlock;

  for (i = 0; i < SSF_MEMS_BAUDRATE_MAX &&
       i < data->sensor->profile.baudrate_count; i++) {
    enum ssf_mems_baudrate candidate = i;

    if (ssf_mems_protocol_baudrate_tried(data, candidate, initial) == true)
      continue;

    ssf_mems_modbus_receive_flush(data);
    ret = ssf_mems_protocol_set_host_baudrate(data, candidate);
    if (ret != 0)
      goto restore_initial;

    ret = ssf_mems_protocol_read_baudrate_once(serdev, &found, timeout_ms);
    if (ret == 0) {
      found = ssf_mems_protocol_accept_baudrate(data, candidate, found);
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
