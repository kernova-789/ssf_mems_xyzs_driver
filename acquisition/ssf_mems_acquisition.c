// SPDX-License-Identifier: GPL-2.0-only
#include "ssf_mems_acquisition.h"

#include "core.h"
#include "modbus/modbus_types.h"
#include "transport/serial_transport.h"
#include "ssf_mems_iio.h"

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/property.h>

#define SSF_MEMS_ACQ_STARTUP_MS 1000U /* 加载后首次通信前的等待，ms。 */
#define SSF_MEMS_ACQ_TIMEOUT_MS 200U /* 普通请求的基础应答等待，ms。 */
#define SSF_MEMS_ACQ_SWITCH_SETTLE_MS 20U /* 协议层重启验证后的额外间隔，ms。 */
#define SSF_MEMS_ACQ_RETRY_INITIAL_MS 1000U /* 首次连接失败的退避时长，ms。 */
#define SSF_MEMS_ACQ_RETRY_MAX_MS 30000U /* 指数退避的最大时长，ms。 */
#define SSF_MEMS_ACQ_WINDOW_MS 5000U /* 默认原始/特征各采集 5 秒。 */

/* 检查采集是否需要停止。
 * 返回：true 停止，false 继续。
 */
static bool ssf_mems_acquisition_stopping(struct ssf_mems_acquisition_state *acq) {
  return READ_ONCE(acq->stopping) == true || kthread_should_stop() == true;
}

/* 等待下一轮采集。
 * 返回：无。
 * 停止或配置变化可提前唤醒。
 */
static void ssf_mems_acquisition_wait(struct ssf_mems_acquisition_state *acq,
                                     unsigned int delay_ms, bool reconfigure) {
  wait_event_interruptible_timeout(
      acq->waitq, ssf_mems_acquisition_stopping(acq) == true ||
                      (reconfigure == true && (READ_ONCE(acq->reconnect) == true ||
                                              READ_ONCE(acq->config_changed) == true ||
                                              READ_ONCE(acq->poll_changed) == true)),
      msecs_to_jiffies(delay_ms));
}

/* 标记设备离线并使特征缓存失效。
 * 返回：无。
 * 调用者须持采集锁。
 */
static void ssf_mems_acquisition_offline(struct ssf_mems_data *data) {
  mutex_lock(&data->acquisition.status_lock);
  data->acquisition.online = false;
  mutex_unlock(&data->acquisition.status_lock);
  ssf_mems_protocol_invalidate_features(data);
}

/* 计算特征读取的超时时间。
 * 返回：超时毫秒数。
 * 按实际串口速率预留响应传输时间。
 */
static unsigned int ssf_mems_acquisition_read_timeout(
    struct ssf_mems_data *data) {
  return SSF_MEMS_ACQ_TIMEOUT_MS +
         ssf_mems_serial_wire_ms(data, 5U + SSF_MEMS_MODBUS_READ_MAX_REGS * 2U);
}

/* 读取并发布一份特征。
 * 返回：0 读取成功，负数为停止或读取错误。
 * IIO 推送失败只记录日志。
 */
static int ssf_mems_acquisition_sample(struct ssf_mems_data *data) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  struct ssf_mems_sensor_data features;
  unsigned long started = jiffies;
  unsigned long now;
  int ret;

  ret = data->sensor_ops->read_features(data,
                                       ssf_mems_acquisition_read_timeout(data));
  if (ret != 0)
    return ret;
  if (ssf_mems_acquisition_stopping(acq) == true)
    return -ENODEV;
  ret = ssf_mems_protocol_get_features(data->serdev, &features);
  if (ret != 0)
    return ret;

  now = jiffies;
  mutex_lock(&acq->status_lock);
  acq->last_sample = now;
  acq->have_sample = true;
  acq->online = true;
  ssf_mems_acquisition_policy_sample(&acq->policy, &features,
                                    jiffies_to_msecs(now - started), now);
  mutex_unlock(&acq->status_lock);
  ret = ssf_mems_iio_publish_features(data, &features, 0);
  /* buffer 消费错误不等于串口通信错误，缓存仍是一份完整的有效结果。 */
  if (ret != 0)
    dev_warn_ratelimited(&data->serdev->dev,
                        "failed to push feature sample to IIO buffer: %d\n", ret);
  return 0;
}

/* 匹配通信速率并配置采样参数。
 * 返回：0 连接成功，负数为停止或通信错误。
 */
static int ssf_mems_acquisition_connect(struct ssf_mems_data *data) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  enum ssf_mems_baudrate found;
  int ret;

  acq->phase = SSF_MEMS_ACQ_MATCH;
  ret = ssf_mems_protocol_get_baudrate(data->serdev, &found,
                                      SSF_MEMS_ACQ_TIMEOUT_MS);
  if (ret != 0 || ssf_mems_acquisition_stopping(acq) == true)
    return ret != 0 ? ret : -ENODEV;

  dev_info(&data->serdev->dev,
           "sensor matched: baud %d, target %d, sampling target index %u\n",
           ssf_mems_baudrate_to_value(data->sensor, found),
           ssf_mems_baudrate_to_value(data->sensor, acq->target_baudrate),
           acq->target_sampling_rate_index);
  if (ssf_mems_baudrate_to_value(data->sensor, found) !=
      ssf_mems_baudrate_to_value(data->sensor, acq->target_baudrate)) {
    acq->phase = SSF_MEMS_ACQ_SWITCH;
    ret = ssf_mems_protocol_set_baudrate(data->serdev, acq->target_baudrate,
                                        SSF_MEMS_ACQ_TIMEOUT_MS);
    if (ret != 0)
      return ret;
  }

  /* 协议层已完成保存、重启等待和波特率验证；这里另读完整特征确认可采集。
   * 不修改校验位，20 ms 只作为后续采集前的额外间隔。 */
  acq->phase = SSF_MEMS_ACQ_VERIFY;
  ssf_mems_acquisition_wait(acq, SSF_MEMS_ACQ_SWITCH_SETTLE_MS, false);
  if (ssf_mems_acquisition_stopping(acq) == true)
    return -ENODEV;

  /* 波特率确认后配置采样率，不额外发送保存/重启命令。
   * 已持 acq->lock，直接调用协议接口，避免重复获取采集锁。 */
  acq->phase = SSF_MEMS_ACQ_CONFIGURE;
  ret = ssf_mems_protocol_write_setting(
      data->serdev, SSF_MEMS_SETTING_SAMPLING_RATE,
      acq->target_sampling_rate_index,
      ssf_mems_acquisition_read_timeout(data));
  if (ret != 0)
    dev_err_ratelimited(&data->serdev->dev,
                        "failed to configure sampling rate index %u: %d\n",
                        acq->target_sampling_rate_index, ret);
  if (ret != 0 || ssf_mems_acquisition_stopping(acq) == true)
    return ret != 0 ? ret : -ENODEV;

  dev_info(&data->serdev->dev,
           "sampling configuration verified: index %u; firmware applies after current batch\n",
           acq->target_sampling_rate_index);

  acq->phase = SSF_MEMS_ACQ_VERIFY;
  ssf_mems_acquisition_policy_restart(&acq->policy, jiffies);
  if (acq->mode == SSF_MEMS_ACQ_MODE_RAW) {
    /* 速率和采样配置已读回；纯原始模式不额外读取或发布特征。 */
    mutex_lock(&acq->status_lock);
    acq->online = true;
    acq->have_sample = false;
    mutex_unlock(&acq->status_lock);
    ssf_mems_protocol_invalidate_features(data);
  } else {
    ret = ssf_mems_acquisition_sample(data);
    if (ret != 0)
      return ret;
  }

  acq->phase = SSF_MEMS_ACQ_POLL;
  acq->feature_until = jiffies + msecs_to_jiffies(acq->feature_duration_ms);
  acq->retry_ms = SSF_MEMS_ACQ_RETRY_INITIAL_MS;
  /* 采样率配置已读回，实际测量生效时间由设备决定。 */
  dev_info(&data->serdev->dev,
           "sensor online, baud rate %d, sampling rate index %u, poll period %u ms\n",
           ssf_mems_baudrate_to_value(data->sensor, acq->target_baudrate),
           acq->target_sampling_rate_index,
           acq->policy.interval_ms);
  return 0;
}

/* 按当前窗口执行一轮原始或特征采集。
 * 返回：0 成功，负数为采集或恢复错误。
 * 调用者持采集锁；原始窗口结束后才恢复特征读取。
 */
static int ssf_mems_acquisition_collect(struct ssf_mems_data *data) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  int ret;

  if (acq->mode == SSF_MEMS_ACQ_MODE_RAW ||
      (acq->mode == SSF_MEMS_ACQ_MODE_ALTERNATING &&
       time_after_eq(jiffies, acq->feature_until) == true)) {
    acq->phase = SSF_MEMS_ACQ_RAW;
    mutex_lock(&acq->status_lock);
    acq->have_sample = false;
    mutex_unlock(&acq->status_lock);
    ret = data->sensor_ops->collect_raw(data, acq->raw_duration_ms,
                                       acq->raw_batch_samples);
    if (READ_ONCE(data->protocol.mode) != SSF_MEMS_LINK_MODBUS) {
      /* 定长上传未确认结束时不能轮询特征；connect 先完成只读恢复。 */
      ssf_mems_acquisition_offline(data);
      acq->phase = SSF_MEMS_ACQ_BACKOFF;
    } else {
      acq->phase = SSF_MEMS_ACQ_POLL;
    }
    acq->feature_until = jiffies + msecs_to_jiffies(acq->feature_duration_ms);
    if (ret == 0 || ret == -ENOSPC || ret == -EILSEQ)
      ssf_mems_acquisition_policy_contact(&acq->policy, jiffies);
    /* 参数不支持时，交替模式继续特征窗口；纯原始模式保持原选择。 */
    if (acq->phase == SSF_MEMS_ACQ_POLL &&
        (ret == 0 || ret == -EOPNOTSUPP) &&
        acq->mode != SSF_MEMS_ACQ_MODE_RAW &&
        ssf_mems_acquisition_stopping(acq) == false)
      ret = ssf_mems_acquisition_sample(data);
  } else {
    ret = ssf_mems_acquisition_sample(data);
  }
  return ret;
}

/* 记录采集错误并在持续通信失败时转入重连。
 * 返回：无。
 * 调用者持采集锁；缓冲区错误不直接判定链路离线。
 */
static void ssf_mems_acquisition_error(struct ssf_mems_data *data, int ret) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;

  if (ret == -EOPNOTSUPP)
    return;
  if (acq->have_sample == true &&
      time_after_eq(jiffies, acq->last_sample +
                                msecs_to_jiffies(SSF_MEMS_LINK_FAILURE_MS)) == true)
    ssf_mems_protocol_invalidate_features(data);
  if (ssf_mems_acquisition_policy_error(&acq->policy, ret, jiffies) == true) {
    ssf_mems_acquisition_offline(data);
    acq->phase = SSF_MEMS_ACQ_BACKOFF;
    dev_warn_ratelimited(&data->serdev->dev,
                        "sensor communication lost, restarting baud discovery\n");
  } else if (ret != -ENODEV) {
    dev_warn_ratelimited(&data->serdev->dev,
                        "sensor acquisition failed: %d\n", ret);
  }
}

/* 计算本轮结束后的等待时间。
 * 返回：等待毫秒数，保留帧间隔并兼顾交替窗口截止时间。
 */
static unsigned int ssf_mems_acquisition_next_delay(
    struct ssf_mems_data *data, unsigned long started) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  unsigned int delay_ms;

  if (acq->phase == SSF_MEMS_ACQ_BACKOFF) {
    delay_ms = SSF_MEMS_ACQ_RETRY_INITIAL_MS;
  } else {
    /* 周期从本轮开始计算，扣掉已经用于通信的时间，不补发积压轮次。 */
    unsigned int elapsed_ms = jiffies_to_msecs(jiffies - started);
    unsigned int gap_ms = ssf_mems_serial_gap_ms(data);

    delay_ms = acq->mode == SSF_MEMS_ACQ_MODE_RAW ? gap_ms :
                   (acq->policy.interval_ms > elapsed_ms ?
                        acq->policy.interval_ms - elapsed_ms : gap_ms);
    /* 不因为特征轮询间隔长于窗口而明显推迟切换到原始采集。 */
    if (acq->mode == SSF_MEMS_ACQ_MODE_ALTERNATING &&
        time_before(jiffies, acq->feature_until) == true)
      delay_ms = min(delay_ms,
                     (unsigned int)jiffies_to_msecs(acq->feature_until - jiffies));
    delay_ms = max(gap_ms, delay_ms);
  }
  return delay_ms;
}

/* 在后台执行采集和重连。
 * 返回：0 表示线程已停止。
 * 通信失败后等待一段时间再重试。
 */
static int ssf_mems_acquisition_thread(void *context) {
  struct ssf_mems_data *data = context;
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  unsigned int delay_ms;
  unsigned long started;
  int ret;

  ssf_mems_acquisition_wait(acq, SSF_MEMS_ACQ_STARTUP_MS, false);
  while (ssf_mems_acquisition_stopping(acq) == false) {
    /* 整个采集步骤与 sysfs 配置互斥，轮间休眠在解锁后执行。 */
    mutex_lock(&acq->lock);
    if (ssf_mems_acquisition_stopping(acq) == true) {
      mutex_unlock(&acq->lock);
      break;
    }
    if (acq->firmware_blocked == true) {
      /* 末包结果不确定时只等待人工处理，不能用自动重连继续写配置。 */
      mutex_unlock(&acq->lock);
      ssf_mems_acquisition_wait(acq, SSF_MEMS_ACQ_RETRY_INITIAL_MS, false);
      continue;
    }
    if (acq->config_changed == true) {
      WRITE_ONCE(acq->config_changed, false);
      /* 交替模式修改后从完整特征窗口开始；纯原始模式下不使用此截止时间。 */
      acq->feature_until = jiffies + msecs_to_jiffies(acq->feature_duration_ms);
    }
    WRITE_ONCE(acq->poll_changed, false);
    started = jiffies;
    if (acq->phase != SSF_MEMS_ACQ_POLL || acq->reconnect == true) {
      /* 未上线/请求重连时先验证链路，再允许发布采集数据。 */
      WRITE_ONCE(acq->reconnect, false);
      ret = ssf_mems_acquisition_connect(data);
      if (ret != 0) {
        ssf_mems_acquisition_offline(data);
        acq->phase = SSF_MEMS_ACQ_BACKOFF;
        delay_ms = acq->retry_ms;
        acq->retry_ms = min(SSF_MEMS_ACQ_RETRY_MAX_MS, acq->retry_ms * 2U);
        if (ssf_mems_acquisition_stopping(acq) == false)
          dev_warn_ratelimited(&data->serdev->dev,
                              "sensor connection failed: %d, retry in %u ms\n",
                              ret, delay_ms);
      } else {
        /* 纯原始模式完成配置后立即开始批次，其他模式保留特征读取间隔。 */
        delay_ms = acq->mode == SSF_MEMS_ACQ_MODE_RAW ?
                       ssf_mems_serial_gap_ms(data) : acq->policy.interval_ms;
        if (acq->mode == SSF_MEMS_ACQ_MODE_ALTERNATING)
          delay_ms = min(delay_ms, acq->feature_duration_ms);
      }
    } else if (acq->mode == SSF_MEMS_ACQ_MODE_RAW && acq->raw_manual) {
      /* 用户显式窗口期间采集锁由触发请求持有；空闲时不发送采集命令。 */
      delay_ms = SSF_MEMS_ACQ_RETRY_INITIAL_MS;
    } else {
      ret = ssf_mems_acquisition_collect(data);
      if (ret != 0)
        ssf_mems_acquisition_error(data, ret);
      delay_ms = ssf_mems_acquisition_next_delay(data, started);
      /* 纯原始模式遇到不支持的组合时限速重试，不进行高速忙轮询。 */
      if (ret == -EOPNOTSUPP)
        delay_ms = max(delay_ms, SSF_MEMS_ACQ_RETRY_INITIAL_MS);
    }
    mutex_unlock(&acq->lock);
    ssf_mems_acquisition_wait(acq, delay_ms, true);
  }
  mutex_lock(&acq->lock);
  ssf_mems_acquisition_offline(data);
  acq->phase = SSF_MEMS_ACQ_STOPPED;
  mutex_unlock(&acq->lock);
  return 0;
}

/* 读取设备树中的采集窗口时长。
 * 返回：窗口毫秒数，无效时使用默认值。
 * allow_zero 为 true 时允许返回 0。
 */
static unsigned int ssf_mems_acquisition_window(struct device *dev,
                                               const char *property,
                                               bool allow_zero) {
  u32 value;
  int ret = device_property_read_u32(dev, property, &value);

  if (ret == 0 && ((allow_zero == true && value == 0) ||
                   (value >= SSF_MEMS_ACQ_MIN_DURATION_MS &&
                    value <= SSF_MEMS_ACQ_MAX_DURATION_MS)))
    return value;
  dev_info(dev, "could not use %s, using default %u ms\n",
           property, SSF_MEMS_ACQ_WINDOW_MS);
  return SSF_MEMS_ACQ_WINDOW_MS;
}

/* 初始化采集状态和窗口配置。
 * 返回：0 成功，-EINVAL 设备无效。
 */
int ssf_mems_acquisition_init(struct ssf_mems_data *data) {
  struct ssf_mems_acquisition_state *acq;

  if (data == NULL || data->serdev == NULL || data->sensor == NULL ||
      data->sensor_ops == NULL || data->sensor_ops->read_features == NULL ||
      data->sensor_ops->collect_raw == NULL)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_init(&acq->lock);
  mutex_init(&acq->status_lock);
  init_waitqueue_head(&acq->waitq);
  acq->thread = NULL;
  acq->stopping = false;
  acq->reconnect = false;
  acq->online = false;
  acq->firmware_blocked = false;
  acq->have_sample = false;
  acq->last_sample = 0;
  acq->phase = SSF_MEMS_ACQ_STARTUP;
  /* 设备树仅提供初次匹配提示；原始和特征通信共用型号表中的工作目标。 */
  acq->target_baudrate = data->sensor->profile.default_work_baudrate;
  acq->target_sampling_rate_index =
      data->sensor->profile.default_sampling_rate_index;
  acq->retry_ms = SSF_MEMS_ACQ_RETRY_INITIAL_MS;
  acq->raw_batch_samples = SSF_MEMS_RAW_BATCH_SAMPLES;
  acq->raw_manual = false;
  acq->raw_duration_ms = ssf_mems_acquisition_window(
      &data->serdev->dev, "sange-cbm,raw-duration-ms", true);
  acq->feature_duration_ms = ssf_mems_acquisition_window(
      &data->serdev->dev, "sange-cbm,feature-duration-ms", false);
  /* 兼容旧设备树 raw-duration-ms=0 的“仅特征”语义；保存合法原始窗口供以后切换。 */
  acq->mode = acq->raw_duration_ms == 0 ? SSF_MEMS_ACQ_MODE_FEATURE :
                                        SSF_MEMS_ACQ_MODE_ALTERNATING;
  if (acq->raw_duration_ms == 0)
    acq->raw_duration_ms = SSF_MEMS_ACQ_WINDOW_MS;
  if (data->protocol.parity != SSF_MEMS_PARITY_NONE) {
    acq->mode = SSF_MEMS_ACQ_MODE_FEATURE;
    dev_info(&data->serdev->dev,
             "finite raw mode requires no parity; using feature mode\n");
  }
  acq->config_changed = false;
  acq->poll_changed = false;
  acq->feature_until = 0;
  ssf_mems_acquisition_policy_init(&acq->policy, data->sensor, jiffies);
  dev_info(&data->serdev->dev,
           "acquisition configuration: target baud %d, sampling index %u\n",
           ssf_mems_baudrate_to_value(data->sensor, acq->target_baudrate),
           acq->target_sampling_rate_index);
  return 0;
}

/* 复制当前采集配置。
 * 返回：0 成功，-EINVAL 参数无效，-ENODEV 正在停止。
 * 只取短状态锁，不等待在途采集或升级。
 */
int ssf_mems_acquisition_get_config(
    struct ssf_mems_data *data, struct ssf_mems_acquisition_config *config) {
  struct ssf_mems_acquisition_state *acq;
  int ret = 0;

  if (data == NULL || config == NULL)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->status_lock);
  if (READ_ONCE(acq->stopping) == true) {
    ret = -ENODEV;
  } else {
    config->mode = acq->mode;
    config->raw_duration_ms = acq->raw_duration_ms;
    config->raw_batch_samples = acq->raw_batch_samples;
    config->raw_manual = acq->raw_manual;
    config->feature_duration_ms = acq->feature_duration_ms;
    config->poll = acq->policy.config;
  }
  mutex_unlock(&acq->status_lock);
  return ret;
}

/* 设置原始、特征或交替采集模式。
 * 返回：0 成功，负数为参数、停止或校验位错误。
 * 只改变主机调度，可能等待在途采集窗口完成。
 */
int ssf_mems_acquisition_set_mode(struct ssf_mems_data *data,
                                  enum ssf_mems_acquisition_mode mode) {
  struct ssf_mems_acquisition_state *acq;
  int ret = 0;

  if (data == NULL || (unsigned int)mode >= SSF_MEMS_ACQ_MODE_COUNT)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping) == true) {
    ret = -ENODEV;
  } else if (mode != SSF_MEMS_ACQ_MODE_FEATURE &&
             data->protocol.parity != SSF_MEMS_PARITY_NONE) {
    ret = -EOPNOTSUPP;
  } else if (acq->mode != mode) {
    mutex_lock(&acq->status_lock);
    acq->mode = mode;
    if (mode == SSF_MEMS_ACQ_MODE_RAW)
      acq->have_sample = false;
    mutex_unlock(&acq->status_lock);
    WRITE_ONCE(acq->config_changed, true);
    if (mode == SSF_MEMS_ACQ_MODE_RAW)
      ssf_mems_protocol_invalidate_features(data);
    wake_up_interruptible(&acq->waitq);
  }
  mutex_unlock(&acq->lock);
  return ret;
}

/* 设置一个采集窗口的时长。
 * 返回：0 成功，-EINVAL 参数无效，-ENODEV 正在停止。
 */
int ssf_mems_acquisition_set_duration(struct ssf_mems_data *data,
                                      bool raw_window, unsigned int duration_ms) {
  struct ssf_mems_acquisition_state *acq;
  unsigned int *window;
  int ret = 0;

  if (data == NULL || duration_ms < SSF_MEMS_ACQ_MIN_DURATION_MS ||
      duration_ms > SSF_MEMS_ACQ_MAX_DURATION_MS)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  window = raw_window == true ? &acq->raw_duration_ms : &acq->feature_duration_ms;
  if (READ_ONCE(acq->stopping) == true) {
    ret = -ENODEV;
  } else if (*window != duration_ms) {
    mutex_lock(&acq->status_lock);
    *window = duration_ms;
    mutex_unlock(&acq->status_lock);
    WRITE_ONCE(acq->config_changed, true);
    wake_up_interruptible(&acq->waitq);
  }
  mutex_unlock(&acq->lock);
  return ret;
}

/* 设置下一原始窗口的真实批次长度。
 * 返回：0 成功，-EINVAL 长度不支持，-ENODEV 正在停止。
 * 只保存主机配置，采集锁保证在途窗口不被改写。
 */
int ssf_mems_acquisition_set_raw_samples(struct ssf_mems_data *data, u32 samples) {
  struct ssf_mems_acquisition_state *acq;
  int ret = 0;

  if (data == NULL || !ssf_mems_sensor_raw_samples_valid(data->sensor, samples))
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping)) {
    ret = -ENODEV;
  } else {
    mutex_lock(&acq->status_lock);
    acq->raw_batch_samples = samples;
    mutex_unlock(&acq->status_lock);
  }
  mutex_unlock(&acq->lock);
  return ret;
}

/* 选择 raw 自动循环或显式窗口控制；在当前窗口结束后生效。 */
int ssf_mems_acquisition_set_raw_manual(struct ssf_mems_data *data, bool manual) {
  struct ssf_mems_acquisition_state *acq;
  int ret = 0;

  if (data == NULL)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping)) {
    ret = -ENODEV;
  } else {
    mutex_lock(&acq->status_lock);
    acq->raw_manual = manual;
    mutex_unlock(&acq->status_lock);
    WRITE_ONCE(acq->config_changed, true);
    wake_up_interruptible(&acq->waitq);
  }
  mutex_unlock(&acq->lock);
  return ret;
}

/* 同步采集一个原始窗口；不执行长度轮换，不额外改变采集模式。 */
int ssf_mems_acquisition_capture_window(struct ssf_mems_data *data) {
  struct ssf_mems_acquisition_state *acq;
  int ret;

  if (data == NULL)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping)) {
    ret = -ENODEV;
  } else if (acq->firmware_blocked) {
    ret = -EBUSY;
  } else if (!acq->raw_manual || acq->mode != SSF_MEMS_ACQ_MODE_RAW) {
    ret = -EINVAL;
  } else if (!acq->online || acq->phase != SSF_MEMS_ACQ_POLL) {
    ret = -ENODATA;
  } else {
    acq->phase = SSF_MEMS_ACQ_RAW;
    mutex_lock(&acq->status_lock);
    acq->have_sample = false;
    mutex_unlock(&acq->status_lock);
    ret = data->sensor_ops->collect_raw(data, acq->raw_duration_ms,
                                       acq->raw_batch_samples);
    if (READ_ONCE(data->protocol.mode) != SSF_MEMS_LINK_MODBUS) {
      ssf_mems_acquisition_offline(data);
      acq->phase = SSF_MEMS_ACQ_BACKOFF;
    } else {
      acq->phase = SSF_MEMS_ACQ_POLL;
    }
    if (ret == 0)
      ssf_mems_acquisition_policy_contact(&acq->policy, jiffies);
    else
      ssf_mems_acquisition_error(data, ret);
  }
  mutex_unlock(&acq->lock);
  return ret;
}

/* 一次性声音采集不改变振动长度、采样率或原有调度模式。 */
int ssf_mems_acquisition_capture_sound(struct ssf_mems_data *data) {
  struct ssf_mems_acquisition_state *acq;
  int ret;

  if (!data || !data->sensor_ops)
    return -EINVAL;
  if (!data->sensor_ops->collect_sound)
    return -EOPNOTSUPP;
  acq = &data->acquisition;
  ret = mutex_lock_interruptible(&acq->lock);
  if (ret)
    return ret;
  if (READ_ONCE(acq->stopping)) {
    ret = -ENODEV;
  } else if (acq->firmware_blocked) {
    ret = -EBUSY;
  } else if (!acq->online || acq->phase != SSF_MEMS_ACQ_POLL) {
    ret = -ENODATA;
  } else {
    acq->phase = SSF_MEMS_ACQ_RAW;
    mutex_lock(&acq->status_lock);
    acq->have_sample = false;
    mutex_unlock(&acq->status_lock);
    ret = data->sensor_ops->collect_sound(data);
    if (READ_ONCE(data->protocol.mode) != SSF_MEMS_LINK_MODBUS) {
      ssf_mems_acquisition_offline(data);
      acq->phase = SSF_MEMS_ACQ_BACKOFF;
    } else {
      acq->phase = SSF_MEMS_ACQ_POLL;
    }
    acq->feature_until = jiffies + msecs_to_jiffies(acq->feature_duration_ms);
    if (!ret)
      ssf_mems_acquisition_policy_contact(&acq->policy, jiffies);
    else
      ssf_mems_acquisition_error(data, ret);
  }
  mutex_unlock(&acq->lock);
  return ret;
}

/* 修改主机特征轮询配置；不截断在途原始批次，不写传感器配置。 */
int ssf_mems_acquisition_set_poll_setting(struct ssf_mems_data *data,
                                          enum ssf_mems_poll_setting setting,
                                          unsigned int value) {
  struct ssf_mems_acquisition_state *acq;
  struct ssf_mems_poll_config config;
  unsigned int previous_value;
  int ret = 0;

  if (data == NULL)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping) == true) {
    ret = -ENODEV;
    goto out;
  }
  config = acq->policy.config;
  switch (setting) {
  case SSF_MEMS_POLL_SETTING_MODE:
    previous_value = config.mode;
    config.mode = value;
    break;
  case SSF_MEMS_POLL_SETTING_FIXED_INTERVAL:
    previous_value = config.fixed_interval_ms;
    config.fixed_interval_ms = value;
    break;
  case SSF_MEMS_POLL_SETTING_MIN_INTERVAL:
    previous_value = config.min_interval_ms;
    config.min_interval_ms = value;
    break;
  case SSF_MEMS_POLL_SETTING_MAX_INTERVAL:
    previous_value = config.max_interval_ms;
    config.max_interval_ms = value;
    break;
  default:
    ret = -EINVAL;
    goto out;
  }
  if (previous_value == value)
    goto out;
  mutex_lock(&acq->status_lock);
  ret = ssf_mems_acquisition_policy_configure(&acq->policy, &config, jiffies);
  mutex_unlock(&acq->status_lock);
  if (ret == 0 &&
      (setting == SSF_MEMS_POLL_SETTING_MODE ||
       (acq->policy.config.mode == SSF_MEMS_POLL_FIXED &&
        setting == SSF_MEMS_POLL_SETTING_FIXED_INTERVAL) ||
       (acq->policy.config.mode == SSF_MEMS_POLL_ADAPTIVE &&
        (setting == SSF_MEMS_POLL_SETTING_MIN_INTERVAL ||
         setting == SSF_MEMS_POLL_SETTING_MAX_INTERVAL)))) {
    WRITE_ONCE(acq->poll_changed, true);
    wake_up_interruptible(&acq->waitq);
  }
out:
  mutex_unlock(&acq->lock);
  return ret;
}

/* 启动后台采集线程。
 * 返回：0 成功，负数为状态或线程创建错误。
 * 在 IIO 注册后调用。
 */
int ssf_mems_acquisition_start(struct ssf_mems_data *data) {
  struct task_struct *thread;

  if (data == NULL || data->indio_dev == NULL)
    return -EINVAL;
  if (READ_ONCE(data->acquisition.stopping) == true)
    return -ENODEV;
  if (data->acquisition.thread != NULL)
    return -EBUSY;
  thread = kthread_run(ssf_mems_acquisition_thread, data, "ssf-mems-%s",
                       dev_name(&data->serdev->dev));
  if (IS_ERR(thread) == true)
    return PTR_ERR(thread);
  data->acquisition.thread = thread;
  return 0;
}

/* 停止采集线程并收尾在途批次。
 * 返回：无。
 * 调用前关闭请求模块，完成后才能释放接收和 IIO 资源。
 */
void ssf_mems_acquisition_stop(struct ssf_mems_data *data) {
  struct ssf_mems_acquisition_state *acq;
  int ret;

  if (data == NULL)
    return;
  acq = &data->acquisition;
  WRITE_ONCE(acq->stopping, true);
  wake_up_interruptible(&acq->waitq);
  wake_up_interruptible(&data->raw.waitq);
  wake_up_interruptible(&data->sound.waitq);
  if (acq->thread != NULL) {
    kthread_stop(acq->thread);
    acq->thread = NULL;
  }
  ret = ssf_mems_protocol_recover_raw(data);
  if (ret != 0)
    dev_warn(&data->serdev->dev,
             "sensor raw mode remains unconfirmed during removal: %d\n", ret);
}

/* 确认当前通信波特率。
 * 返回：0 成功，负数为参数、停止、升级占用或通信错误。
 * 与后台采集共用采集锁。
 */
int ssf_mems_acquisition_get_baudrate(struct ssf_mems_data *data,
                                    enum ssf_mems_baudrate *baudrate) {
  struct ssf_mems_acquisition_state *acq;
  int ret;

  if (data == NULL || baudrate == NULL)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping) == true) {
    ret = -ENODEV;
    goto out;
  }
  if (acq->firmware_blocked == true) {
    ret = -EBUSY;
    goto out;
  }
  ret = ssf_mems_protocol_get_baudrate(data->serdev, baudrate,
                                      SSF_MEMS_ACQ_TIMEOUT_MS);
  if (ret != 0 || ssf_mems_baudrate_to_value(data->sensor, *baudrate) !=
                 ssf_mems_baudrate_to_value(data->sensor, acq->target_baudrate)) {
    ssf_mems_acquisition_offline(data);
    WRITE_ONCE(acq->reconnect, true);
    wake_up_interruptible(&acq->waitq);
  }
out:
  mutex_unlock(&acq->lock);
  return ret;
}

/* 修改波特率并更新重连目标。
 * 返回：0 成功，负数为参数、停止、升级占用或通信错误。
 * 与后台采集共用采集锁。
 */
int ssf_mems_acquisition_set_baudrate(struct ssf_mems_data *data,
                                    enum ssf_mems_baudrate baudrate) {
  struct ssf_mems_acquisition_state *acq;
  int ret;

  if (data == NULL || (unsigned int)baudrate >= SSF_MEMS_BAUDRATE_MAX)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping) == true) {
    ret = -ENODEV;
    goto out;
  }
  if (acq->firmware_blocked == true) {
    ret = -EBUSY;
    goto out;
  }
  ret = ssf_mems_protocol_set_baudrate(data->serdev, baudrate,
                                      SSF_MEMS_ACQ_TIMEOUT_MS);
  if (ret == 0)
    acq->target_baudrate = baudrate;
  /* 即使写入失败，传感器也可能已经切换，只是应答丢失；重新匹配实际速率。 */
  ssf_mems_acquisition_offline(data);
  WRITE_ONCE(acq->reconnect, true);
  wake_up_interruptible(&acq->waitq);
out:
  mutex_unlock(&acq->lock);
  return ret;
}

/* 读取一个传感器配置项。
 * 返回：0 成功，负数为参数、设备状态或通信错误。
 * 设备上线后才发送请求。
 */
int ssf_mems_acquisition_read_setting(struct ssf_mems_data *data,
                                     enum ssf_mems_sensor_setting setting,
                                     u16 *value) {
  struct ssf_mems_acquisition_state *acq;
  int ret;

  if (data == NULL || value == NULL)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping) == true)
    ret = -ENODEV;
  else if (acq->firmware_blocked == true)
    ret = -EBUSY;
  else if (acq->online == false)
    ret = -ENODATA;
  else
    ret = ssf_mems_protocol_read_setting(data->serdev, setting, value,
                                        ssf_mems_acquisition_read_timeout(data));
  mutex_unlock(&acq->lock);
  return ret;
}

/* 写入一个传感器配置项。
 * 返回：0 成功，负数为参数、设备状态或通信错误。
 * 设备上线后才发送请求；确认成功的采样率保留为本次运行的重连目标。
 */
int ssf_mems_acquisition_write_setting(struct ssf_mems_data *data,
                                      enum ssf_mems_sensor_setting setting,
                                      u16 value) {
  struct ssf_mems_acquisition_state *acq;
  int ret;

  if (data == NULL)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping) == true) {
    ret = -ENODEV;
  } else if (acq->firmware_blocked == true) {
    ret = -EBUSY;
  } else if (acq->online == false) {
    ret = -ENODATA;
  } else {
    ret = ssf_mems_protocol_write_setting(data->serdev, setting, value,
                                         ssf_mems_acquisition_read_timeout(data));
    /* 失败的写操作也可能已生效，等待下次完整读取刷新缓存。 */
    if (ret != -EINVAL) {
      ssf_mems_protocol_invalidate_features(data);
      mutex_lock(&acq->status_lock);
      acq->have_sample = false;
      mutex_unlock(&acq->status_lock);
    }
    if (ret == 0) {
      if (setting == SSF_MEMS_SETTING_SAMPLING_RATE)
        acq->target_sampling_rate_index = value;
      ssf_mems_acquisition_policy_restart(&acq->policy, jiffies);
    }
  }
  mutex_unlock(&acq->lock);
  return ret;
}

/* 复制当前采集状态。
 * 返回：无。
 * 只读主机状态，不发送串口请求。
 */
void ssf_mems_acquisition_get_status(
    struct ssf_mems_data *data, struct ssf_mems_acquisition_status *status) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;

  mutex_lock(&acq->status_lock);
  status->online = acq->online;
  status->have_sample = acq->have_sample;
  status->interval_ms = acq->policy.interval_ms;
  status->sample_age_ms = acq->have_sample ?
                            jiffies_to_msecs(jiffies - acq->last_sample) : 0;
  mutex_unlock(&acq->status_lock);
}
