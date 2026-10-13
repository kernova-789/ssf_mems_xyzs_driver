// SPDX-License-Identifier: GPL-2.0-only
#include "ssf_mems_acquisition.h"

#include "core.h"
#include "modbus_types.h"
#include "ssf_mems_iio.h"

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/property.h>

#define SSF_MEMS_ACQ_STARTUP_MS 1000U
#define SSF_MEMS_ACQ_TIMEOUT_MS 200U
#define SSF_MEMS_ACQ_SWITCH_SETTLE_MS 20U
#define SSF_MEMS_ACQ_RETRY_INITIAL_MS 1000U
#define SSF_MEMS_ACQ_RETRY_MAX_MS 30000U
/* 至少保留一段帧间空闲时间；不在超时后立即背靠背发送。 */
#define SSF_MEMS_ACQ_GAP_MS 2U
/* 5333.4 Hz：原始批次已降到 19000 点下限，配置带宽约 2.67 kHz。
 * 适合当前敲击/普通振动测试；完整 1~5.3 kHz 高频段需更高配置。 */
#define SSF_MEMS_ACQ_TARGET_SAMPLING_RATE 6U
#define SSF_MEMS_ACQ_WINDOW_MS 5000U /* 默认原始/特征各采集 5 秒。 */

static bool ssf_mems_acquisition_stopping(struct ssf_mems_acquisition_state *acq) {
  return READ_ONCE(acq->stopping) || kthread_should_stop();
}

static void ssf_mems_acquisition_wait(struct ssf_mems_acquisition_state *acq,
                                     unsigned int delay_ms, bool reconfigure) {
  wait_event_interruptible_timeout(
      acq->waitq, ssf_mems_acquisition_stopping(acq) ||
                      (reconfigure && (READ_ONCE(acq->reconnect) ||
                                       READ_ONCE(acq->config_changed))),
      msecs_to_jiffies(delay_ms));
}

/* 调用者持有 acq->lock；链路离线时不再把旧缓存作为有效数据提供。 */
static void ssf_mems_acquisition_offline(struct ssf_mems_xyzs_data *data) {
  data->acquisition.online = false;
  ssf_mems_protocol_invalidate_features(data);
}

static unsigned int ssf_mems_acquisition_gap(struct ssf_mems_xyzs_data *data) {
  unsigned int baudrate = READ_ONCE(data->protocol.host_baudrate);
  unsigned int bits = data->protocol.parity == SSF_MEMS_PARITY_NONE ? 35U : 39U;

  if (baudrate == 0)
    baudrate = 9600;
  return max(SSF_MEMS_ACQ_GAP_MS, DIV_ROUND_UP(bits * 1000U, baudrate));
}

/* sysfs 可以选择较低速率；响应超时须容纳最长寄存器响应的线上传输时间。 */
static unsigned int ssf_mems_acquisition_read_timeout(
    struct ssf_mems_xyzs_data *data) {
  unsigned int baudrate = READ_ONCE(data->protocol.host_baudrate);
  unsigned int bits = data->protocol.parity == SSF_MEMS_PARITY_NONE ? 10U : 11U;
  unsigned int bytes = 5U + SSF_MEMS_MODBUS_READ_MAX_REGS * 2U;

  if (baudrate == 0)
    baudrate = 9600;
  return SSF_MEMS_ACQ_TIMEOUT_MS + DIV_ROUND_UP(bytes * bits * 1000U, baudrate);
}

static int ssf_mems_acquisition_sample(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  struct ssf_mems_sensor_data features;
  unsigned long started = jiffies;
  unsigned long now;
  int ret;

  ret = ssf_mems_protocol_read_features(data->serdev,
                                       ssf_mems_acquisition_read_timeout(data));
  if (ret != 0)
    return ret;
  if (ssf_mems_acquisition_stopping(acq) == true)
    return -ENODEV;
  ret = ssf_mems_protocol_get_features(data->serdev, &features);
  if (ret != 0)
    return ret;

  now = jiffies;
  acq->last_sample = now;
  acq->have_sample = true;
  acq->online = true;
  ssf_mems_acquisition_policy_sample(&acq->policy, &features,
                                    jiffies_to_msecs(now - started), now);
  ret = ssf_mems_iio_publish_features(data, &features, 0);
  /* buffer 消费错误不等于串口通信错误，缓存仍是一份完整的有效结果。 */
  if (ret != 0)
    dev_warn_ratelimited(&data->serdev->dev,
                        "failed to push feature sample to IIO buffer: %d\n", ret);
  return 0;
}

static int ssf_mems_acquisition_connect(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  enum ssf_mems_baudrate found;
  int ret;

  acq->phase = SSF_MEMS_ACQ_MATCH;
  ret = ssf_mems_protocol_get_baudrate(data->serdev, &found,
                                      SSF_MEMS_ACQ_TIMEOUT_MS);
  if (ret != 0 || ssf_mems_acquisition_stopping(acq) == true)
    return ret ? ret : -ENODEV;

  dev_info(&data->serdev->dev,
           "sensor matched: baud %d, target %d, sampling target index %u (5333.4 Hz)\n",
           ssf_mems_baudrate_to_value(found),
           ssf_mems_baudrate_to_value(acq->target_baudrate),
           SSF_MEMS_ACQ_TARGET_SAMPLING_RATE);
  if (ssf_mems_baudrate_to_value(found) !=
      ssf_mems_baudrate_to_value(acq->target_baudrate)) {
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

  /* 在波特率保存/重启完成后配置采样率，避免重启打断固件应用新采样参数。
   * 已持 acq->lock，直接调用协议接口，不能递归调用采集层写配置接口。
   * 协议层相同值不重复写；40052 由固件自行保存，不发送 40110。 */
  acq->phase = SSF_MEMS_ACQ_CONFIGURE;
  ret = ssf_mems_protocol_write_setting(
      data->serdev, SSF_MEMS_SETTING_SAMPLING_RATE,
      SSF_MEMS_ACQ_TARGET_SAMPLING_RATE,
      ssf_mems_acquisition_read_timeout(data));
  if (ret != 0)
    dev_err_ratelimited(&data->serdev->dev,
                        "failed to configure sampling rate index %u: %d\n",
                        SSF_MEMS_ACQ_TARGET_SAMPLING_RATE, ret);
  if (ret != 0 || ssf_mems_acquisition_stopping(acq) == true)
    return ret ? ret : -ENODEV;

  dev_info(&data->serdev->dev,
           "sampling configuration verified: index %u (5333.4 Hz); firmware applies after current batch\n",
           SSF_MEMS_ACQ_TARGET_SAMPLING_RATE);

  acq->phase = SSF_MEMS_ACQ_VERIFY;
  ssf_mems_acquisition_policy_init(&acq->policy, jiffies);
  if (acq->mode == SSF_MEMS_ACQ_MODE_RAW) {
    /* 速率和采样配置已读回；纯原始模式不额外读取或发布特征。 */
    acq->online = true;
    acq->have_sample = false;
    ssf_mems_protocol_invalidate_features(data);
  } else {
    ret = ssf_mems_acquisition_sample(data);
    if (ret != 0)
      return ret;
  }

  acq->phase = SSF_MEMS_ACQ_POLL;
  acq->feature_until = jiffies + msecs_to_jiffies(acq->feature_duration_ms);
  acq->retry_ms = SSF_MEMS_ACQ_RETRY_INITIAL_MS;
  /* 采样率寄存器已确认；固件在当前计算批次结束后实际应用。 */
  dev_info(&data->serdev->dev,
           "sensor online, baud rate %d, sampling rate index %u (5333.4 Hz), poll period %u ms\n",
           ssf_mems_baudrate_to_value(acq->target_baudrate),
           SSF_MEMS_ACQ_TARGET_SAMPLING_RATE,
           acq->policy.interval_ms);
  return 0;
}

static int ssf_mems_acquisition_thread(void *context) {
  struct ssf_mems_xyzs_data *data = context;
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  unsigned int delay_ms;
  unsigned long started;
  int ret;

  ssf_mems_acquisition_wait(acq, SSF_MEMS_ACQ_STARTUP_MS, false);
  while (!ssf_mems_acquisition_stopping(acq)) {
    mutex_lock(&acq->lock);
    if (ssf_mems_acquisition_stopping(acq) == true) {
      mutex_unlock(&acq->lock);
      break;
    }
    if (acq->config_changed == true) {
      WRITE_ONCE(acq->config_changed, false);
      /* 交替模式修改后从完整特征窗口开始；纯原始模式下不使用此截止时间。 */
      acq->feature_until = jiffies + msecs_to_jiffies(acq->feature_duration_ms);
    }
    started = jiffies;
    if (acq->phase != SSF_MEMS_ACQ_POLL || acq->reconnect == true) {
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
                       ssf_mems_acquisition_gap(data) : acq->policy.interval_ms;
        if (acq->mode == SSF_MEMS_ACQ_MODE_ALTERNATING)
          delay_ms = min(delay_ms, acq->feature_duration_ms);
      }
    } else {
      if (acq->mode == SSF_MEMS_ACQ_MODE_RAW ||
          (acq->mode == SSF_MEMS_ACQ_MODE_ALTERNATING &&
           time_after_eq(jiffies, acq->feature_until))) {
        acq->phase = SSF_MEMS_ACQ_RAW;
        acq->have_sample = false;
        ret = ssf_mems_protocol_collect_raw(data, acq->raw_duration_ms);
        if (ret == -EOPNOTSUPP) {
          dev_warn_ratelimited(&data->serdev->dev,
                   "finite raw mode requires no parity and sampling index 0..8\n");
          /* 不把用户选择的纯原始模式偷偷变成特征模式。 */
          ssf_mems_acquisition_offline(data);
          acq->phase = SSF_MEMS_ACQ_BACKOFF;
        }
        if (READ_ONCE(data->protocol.mode) != SSF_MEMS_LINK_MODBUS) {
          /* 定长上传未确认结束时不能轮询特征；connect 先完成只读恢复。 */
          ssf_mems_acquisition_offline(data);
          acq->phase = SSF_MEMS_ACQ_BACKOFF;
        } else if (acq->phase != SSF_MEMS_ACQ_BACKOFF) {
          acq->phase = SSF_MEMS_ACQ_POLL;
        }
        acq->feature_until = jiffies + msecs_to_jiffies(acq->feature_duration_ms);
        if (ret == 0 || ret == -ENOSPC || ret == -EILSEQ)
          ssf_mems_acquisition_policy_init(&acq->policy, jiffies);
        if (ret == 0 && acq->mode != SSF_MEMS_ACQ_MODE_RAW &&
            ssf_mems_acquisition_stopping(acq) == false)
          ret = ssf_mems_acquisition_sample(data);
      } else {
        ret = ssf_mems_acquisition_sample(data);
      }
      if (ret != 0) {
        if (acq->have_sample == true &&
            time_after_eq(jiffies, acq->last_sample +
                                      msecs_to_jiffies(SSF_MEMS_LINK_FAILURE_MS)))
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
      if (acq->phase == SSF_MEMS_ACQ_BACKOFF) {
        delay_ms = SSF_MEMS_ACQ_RETRY_INITIAL_MS;
      } else {
        unsigned int elapsed_ms = jiffies_to_msecs(jiffies - started);
        unsigned int gap_ms = ssf_mems_acquisition_gap(data);

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

/* 采集时段只读启动属性；缺失/非法时保留默认值，0 可关闭原始采集。 */
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

int ssf_mems_acquisition_init(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_acquisition_state *acq;

  if (data == NULL || data->serdev == NULL)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_init(&acq->lock);
  init_waitqueue_head(&acq->waitq);
  acq->thread = NULL;
  acq->stopping = false;
  acq->reconnect = false;
  acq->online = false;
  acq->have_sample = false;
  acq->last_sample = 0;
  acq->phase = SSF_MEMS_ACQ_STARTUP;
  /* 设备树仅提供初次匹配速率；正式的原始数据和特征通信统一为 128000。 */
  acq->target_baudrate = SSF_MEMS_BAUDRATE_128000;
  acq->retry_ms = SSF_MEMS_ACQ_RETRY_INITIAL_MS;
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
  acq->feature_until = 0;
  ssf_mems_acquisition_policy_init(&acq->policy, jiffies);
  dev_info(&data->serdev->dev,
           "acquisition configuration: target baud %d, sampling index %u (5333.4 Hz)\n",
           ssf_mems_baudrate_to_value(acq->target_baudrate),
           SSF_MEMS_ACQ_TARGET_SAMPLING_RATE);
  return 0;
}

/* 一次锁内复制配置，避免模式和窗口来自不同时刻。 */
int ssf_mems_acquisition_get_config(
    struct ssf_mems_xyzs_data *data, struct ssf_mems_acquisition_config *config) {
  struct ssf_mems_acquisition_state *acq;
  int ret = 0;

  if (data == NULL || config == NULL)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping) == true) {
    ret = -ENODEV;
  } else {
    config->mode = acq->mode;
    config->raw_duration_ms = acq->raw_duration_ms;
    config->feature_duration_ms = acq->feature_duration_ms;
  }
  mutex_unlock(&acq->lock);
  return ret;
}

/* 模式只影响主机调度，不请求重新匹配，也不写传感器配置。 */
int ssf_mems_acquisition_set_mode(struct ssf_mems_xyzs_data *data,
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
    acq->mode = mode;
    WRITE_ONCE(acq->config_changed, true);
    if (mode == SSF_MEMS_ACQ_MODE_RAW) {
      acq->have_sample = false;
      ssf_mems_protocol_invalidate_features(data);
    }
    wake_up_interruptible(&acq->waitq);
  }
  mutex_unlock(&acq->lock);
  return ret;
}

/* 单独修改一个窗口，避免两个 sysfs 写者读改写整个配置时互相覆盖。 */
int ssf_mems_acquisition_set_duration(struct ssf_mems_xyzs_data *data,
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
    *window = duration_ms;
    WRITE_ONCE(acq->config_changed, true);
    wake_up_interruptible(&acq->waitq);
  }
  mutex_unlock(&acq->lock);
  return ret;
}

int ssf_mems_acquisition_start(struct ssf_mems_xyzs_data *data) {
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

void ssf_mems_acquisition_stop(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_acquisition_state *acq;
  int ret;

  if (data == NULL)
    return;
  acq = &data->acquisition;
  WRITE_ONCE(acq->stopping, true);
  wake_up_interruptible(&acq->waitq);
  wake_up_interruptible(&data->raw.waitq);
  if (acq->thread != NULL) {
    kthread_stop(acq->thread);
    acq->thread = NULL;
  }
  ret = ssf_mems_protocol_recover_raw(data);
  if (ret != 0)
    dev_warn(&data->serdev->dev,
             "sensor raw mode remains unconfirmed during removal: %d\n", ret);
}

int ssf_mems_acquisition_get_baudrate(struct ssf_mems_xyzs_data *data,
                                    enum ssf_mems_baudrate *baudrate) {
  struct ssf_mems_acquisition_state *acq;
  int ret;

  if (data == NULL || baudrate == 0)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping) == true) {
    ret = -ENODEV;
    goto out;
  }
  ret = ssf_mems_protocol_get_baudrate(data->serdev, baudrate,
                                      SSF_MEMS_ACQ_TIMEOUT_MS);
  if (ret != 0 || ssf_mems_baudrate_to_value(*baudrate) !=
                 ssf_mems_baudrate_to_value(acq->target_baudrate)) {
    ssf_mems_acquisition_offline(data);
    WRITE_ONCE(acq->reconnect, true);
    wake_up_interruptible(&acq->waitq);
  }
out:
  mutex_unlock(&acq->lock);
  return ret;
}

int ssf_mems_acquisition_set_baudrate(struct ssf_mems_xyzs_data *data,
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

/* 配置请求与自动轮询/重连串行；离线时不在未知 UART 速率上发送请求。 */
int ssf_mems_acquisition_read_setting(struct ssf_mems_xyzs_data *data,
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
  else if (acq->online == false)
    ret = -ENODATA;
  else
    ret = ssf_mems_protocol_read_setting(data->serdev, setting, value,
                                        ssf_mems_acquisition_read_timeout(data));
  mutex_unlock(&acq->lock);
  return ret;
}

int ssf_mems_acquisition_write_setting(struct ssf_mems_xyzs_data *data,
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
  } else if (acq->online == false) {
    ret = -ENODATA;
  } else {
    ret = ssf_mems_protocol_write_setting(data->serdev, setting, value,
                                         ssf_mems_acquisition_read_timeout(data));
    /* 失败的写操作也可能已生效，等待下次完整读取刷新缓存。 */
    if (ret != -EINVAL) {
      ssf_mems_protocol_invalidate_features(data);
      acq->have_sample = false;
    }
    if (ret == 0)
      ssf_mems_acquisition_policy_init(&acq->policy, jiffies);
  }
  mutex_unlock(&acq->lock);
  return ret;
}

void ssf_mems_acquisition_get_status(
    struct ssf_mems_xyzs_data *data, struct ssf_mems_acquisition_status *status) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;

  mutex_lock(&acq->lock);
  status->online = acq->online;
  status->have_sample = acq->have_sample;
  status->interval_ms = acq->policy.interval_ms;
  status->sample_age_ms = acq->have_sample ?
                            jiffies_to_msecs(jiffies - acq->last_sample) : 0;
  mutex_unlock(&acq->lock);
}
