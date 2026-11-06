// SPDX-License-Identifier: GPL-2.0-only
#include "ssf_mems_acquisition.h"

#include "core.h"
#include "modbus/modbus_types.h"
#include "transport/serial_transport.h"
#include "ssf_mems_iio.h"

#include <linux/completion.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/property.h>
#include <linux/string.h>

#define SSF_MEMS_ACQ_STARTUP_MS 1000U /* 加载后首次通信前的等待，ms。 */
#define SSF_MEMS_ACQ_TIMEOUT_MS 200U /* 普通请求的基础应答等待，ms。 */
#define SSF_MEMS_ACQ_SWITCH_SETTLE_MS 20U /* 协议层重启验证后的额外间隔，ms。 */
#define SSF_MEMS_ACQ_RETRY_INITIAL_MS 1000U /* 首次连接失败的退避时长，ms。 */
#define SSF_MEMS_ACQ_RETRY_MAX_MS 30000U /* 指数退避的最大时长，ms。 */
#define SSF_MEMS_ACQ_WINDOW_MS 5000U /* 默认三条路线各 5 秒。 */

/* 每个调用者拥有一个有界请求；完成前调用者一直保留该对象。 */
struct ssf_mems_capture_request {
  struct completion done;
  struct ssf_mems_acquisition_config config;
  unsigned long until;
  bool started, cancelled;
  int result;
};

static void ssf_mems_acquisition_phase(struct ssf_mems_acquisition_state *acq,
                                      enum ssf_mems_acquisition_phase phase) {
  mutex_lock(&acq->status_lock);
  acq->phase = phase;
  mutex_unlock(&acq->status_lock);
}

static void ssf_mems_acquisition_activity(struct ssf_mems_acquisition_state *acq,
    enum ssf_mems_acquisition_route route, bool busy, bool manual) {
  mutex_lock(&acq->status_lock);
  acq->visible_route = route;
  acq->busy = busy;
  acq->manual_active = manual;
  mutex_unlock(&acq->status_lock);
}

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
                                              READ_ONCE(acq->poll_changed) == true ||
                                              READ_ONCE(acq->requests_changed) == true)),
      msecs_to_jiffies(delay_ms));
}

/* 标记设备离线并使特征缓存失效。
 * 返回：无。
 * 调用者须持采集锁。
 */
static void ssf_mems_acquisition_offline(struct ssf_mems_data *data) {
  mutex_lock(&data->acquisition.status_lock);
  data->acquisition.online = false;
  data->acquisition.visible_route = SSF_MEMS_ACQ_ROUTE_NONE;
  data->acquisition.busy = false;
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

  const struct ssf_mems_acquisition_route_desc *desc =
      ssf_mems_acquisition_route_desc(SSF_MEMS_ACQ_ROUTE_FEATURE);

  ret = desc->collect(data, &acq->active_config,
                       ssf_mems_acquisition_read_timeout(data), false);
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

/* 调度状态与连接状态分开；每个就绪阶段都允许批次边界的手动操作。 */
static bool ssf_mems_acquisition_ready(struct ssf_mems_acquisition_state *acq) {
  return acq->phase == SSF_MEMS_ACQ_READY;
}

static bool ssf_mems_route_runnable(struct ssf_mems_data *data,
                                   enum ssf_mems_acquisition_route route) {
  const struct ssf_mems_acquisition_route_desc *desc = ssf_mems_acquisition_route_desc(route);
  const struct ssf_mems_acquisition_config *config = &data->acquisition.active_config;

  return desc && desc->supported(data) &&
      ssf_mems_acquisition_route_selected(config, route) &&
      !(desc->manual_window && config->mode == desc->single_mode && config->raw_manual) &&
      desc->ready(data);
}

/* 从 start 起按 FEATURE -> RAW -> SOUND 环形选择；无可用数据流时休眠。 */
static void ssf_mems_acquisition_select(struct ssf_mems_data *data,
                                       unsigned int start) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  unsigned int i;

  for (i = 0; i < SSF_MEMS_ACQ_ROUTE_COUNT; i++) {
    enum ssf_mems_acquisition_route route = (start + i) % SSF_MEMS_ACQ_ROUTE_COUNT;

    if (!ssf_mems_route_runnable(data, route))
      continue;
    acq->route = route;
    acq->window_start[route] = true;
    acq->phase_until = jiffies +
        msecs_to_jiffies(*ssf_mems_acquisition_duration(&acq->active_config, route));
    return;
  }
  acq->route = SSF_MEMS_ACQ_ROUTE_NONE;
}

/* 仅在线程/手动事务持 lock 时调用；复制和清标志在同一短临界区内完成。 */
static void ssf_mems_acquisition_apply_config(struct ssf_mems_data *data) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  bool changed;

  mutex_lock(&acq->status_lock);
  acq->active_config = acq->config;
  ssf_mems_acquisition_policy_configure(&acq->policy, &acq->config.poll, jiffies);
  acq->applied_version = acq->config_version;
  changed = acq->config_changed;
  WRITE_ONCE(acq->config_changed, false);
  WRITE_ONCE(acq->poll_changed, false);
  WRITE_ONCE(acq->requests_changed, false);
  mutex_unlock(&acq->status_lock);
  if (changed && ssf_mems_acquisition_ready(acq))
    ssf_mems_acquisition_select(data, SSF_MEMS_ACQ_ROUTE_FEATURE);
  if (!ssf_mems_acquisition_route_selected(&acq->active_config, SSF_MEMS_ACQ_ROUTE_FEATURE)) {
    mutex_lock(&acq->status_lock);
    acq->have_sample = false;
    mutex_unlock(&acq->status_lock);
    ssf_mems_protocol_invalidate_features(data);
  }
}

/* 匹配通信速率并配置采样参数。
 * 返回：0 连接成功，负数为停止或通信错误。
 */
static int ssf_mems_acquisition_connect(struct ssf_mems_data *data) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  enum ssf_mems_baudrate found;
  unsigned int interval_ms;
  int ret;

  ssf_mems_acquisition_phase(acq, SSF_MEMS_ACQ_MATCH);
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
    ssf_mems_acquisition_phase(acq, SSF_MEMS_ACQ_SWITCH);
    ret = ssf_mems_protocol_set_baudrate(data->serdev, acq->target_baudrate,
                                        SSF_MEMS_ACQ_TIMEOUT_MS);
    if (ret != 0)
      return ret;
  }

  /* 协议层已完成保存、重启等待和波特率验证；这里另读完整特征确认可采集。
   * 不修改校验位，20 ms 只作为后续采集前的额外间隔。 */
  ssf_mems_acquisition_phase(acq, SSF_MEMS_ACQ_VERIFY);
  ssf_mems_acquisition_wait(acq, SSF_MEMS_ACQ_SWITCH_SETTLE_MS, false);
  if (ssf_mems_acquisition_stopping(acq) == true)
    return -ENODEV;

  /* 波特率确认后配置采样率，不额外发送保存/重启命令。
   * 已持 acq->lock，直接调用协议接口，避免重复获取采集锁。 */
  ssf_mems_acquisition_phase(acq, SSF_MEMS_ACQ_CONFIGURE);
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

  ssf_mems_acquisition_phase(acq, SSF_MEMS_ACQ_VERIFY);
  mutex_lock(&acq->status_lock);
  ssf_mems_acquisition_policy_restart(&acq->policy, jiffies);
  mutex_unlock(&acq->status_lock);
  if (!ssf_mems_acquisition_route_selected(&acq->active_config, SSF_MEMS_ACQ_ROUTE_FEATURE)) {
    /* 速率和采样配置已读回；未选择特征路线时不额外读取或发布特征。 */
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

  ssf_mems_acquisition_phase(acq, SSF_MEMS_ACQ_READY);
  ssf_mems_acquisition_select(data, SSF_MEMS_ACQ_ROUTE_FEATURE);
  ssf_mems_acquisition_activity(acq, acq->route, false, false);
  acq->retry_ms = SSF_MEMS_ACQ_RETRY_INITIAL_MS;
  /* 采样率配置已读回，实际测量生效时间由设备决定。 */
  mutex_lock(&acq->status_lock);
  interval_ms = acq->policy.interval_ms;
  mutex_unlock(&acq->status_lock);
  dev_info(&data->serdev->dev,
           "sensor online, baud rate %d, sampling rate index %u, poll period %u ms\n",
           ssf_mems_baudrate_to_value(data->sensor, acq->target_baudrate),
           acq->target_sampling_rate_index,
           interval_ms);
  return 0;
}

/* 自动/手动路径共用一次完整批事务及恢复门禁；不改变调度窗口。 */
static int ssf_mems_acquisition_batch(
    struct ssf_mems_data *data, enum ssf_mems_acquisition_route route,
    const struct ssf_mems_acquisition_config *config, bool reset_stats, bool manual) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  const struct ssf_mems_acquisition_route_desc *desc = ssf_mems_acquisition_route_desc(route);
  int ret;

  if (READ_ONCE(acq->stopping))
    return -ENODEV;
  if (!desc || !desc->supported(data))
    return -EOPNOTSUPP;
  if (!desc->ready(data))
    return -ENODATA;
  if (desc->begin) {
    ret = desc->begin(data);
    if (ret)
      return ret;
  }
  ssf_mems_acquisition_activity(acq, route, true, manual);
  mutex_lock(&acq->status_lock);
  acq->have_sample = false;
  mutex_unlock(&acq->status_lock);
  ssf_mems_protocol_invalidate_features(data);
  ret = desc->collect(data, config, ssf_mems_acquisition_read_timeout(data), reset_stats);
  if (READ_ONCE(data->protocol.mode) != SSF_MEMS_LINK_MODBUS) {
    ssf_mems_acquisition_offline(data);
    ssf_mems_acquisition_phase(acq, SSF_MEMS_ACQ_BACKOFF);
  }
  if (ret == 0 || ret == -ENOSPC || ret == -EILSEQ) {
    mutex_lock(&acq->status_lock);
    ssf_mems_acquisition_policy_contact(&acq->policy, jiffies);
    mutex_unlock(&acq->status_lock);
  }
  ssf_mems_acquisition_activity(acq, acq->online ? acq->route : SSF_MEMS_ACQ_ROUTE_NONE,
                               false, false);
  return ret;
}

/* 每轮只执行一次特征读取或完整批次。 */
static int ssf_mems_acquisition_collect(struct ssf_mems_data *data) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  const struct ssf_mems_acquisition_route_desc *desc;
  int ret;

  if (acq->route == SSF_MEMS_ACQ_ROUTE_NONE)
    ssf_mems_acquisition_select(data, SSF_MEMS_ACQ_ROUTE_FEATURE);
  else if (!ssf_mems_route_runnable(data, acq->route) ||
           time_after_eq(jiffies, acq->phase_until))
    ssf_mems_acquisition_select(data, acq->route + 1);
  ssf_mems_acquisition_activity(acq, acq->route, false, false);
  desc = ssf_mems_acquisition_route_desc(acq->route);
  if (!desc)
    return 0;
  if (desc->transfer == SSF_MEMS_ACQ_SNAPSHOT) {
    ssf_mems_acquisition_activity(acq, acq->route, true, false);
    ret = ssf_mems_acquisition_sample(data);
    ssf_mems_acquisition_activity(acq, acq->route, false, false);
    return ret;
  }
  ret = ssf_mems_acquisition_batch(data, acq->route, &acq->active_config,
                                  acq->window_start[acq->route], false);
  if (ret != -ENODATA)
    acq->window_start[acq->route] = false;
  return ret;
}

/* 记录采集错误并在持续通信失败时转入重连。
 * 返回：无。
 * 调用者持采集锁；缓冲区错误不直接判定链路离线。
 */
static void ssf_mems_acquisition_error(struct ssf_mems_data *data, int ret) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  bool stale, lost;

  if (ret == -EOPNOTSUPP || ret == -ENODATA)
    return;
  mutex_lock(&acq->status_lock);
  stale = acq->have_sample &&
      time_after_eq(jiffies, acq->last_sample + msecs_to_jiffies(SSF_MEMS_LINK_FAILURE_MS));
  lost = ssf_mems_acquisition_policy_error(&acq->policy, ret, jiffies);
  mutex_unlock(&acq->status_lock);
  if (stale)
    ssf_mems_protocol_invalidate_features(data);
  if (lost) {
    ssf_mems_acquisition_offline(data);
    ssf_mems_acquisition_phase(acq, SSF_MEMS_ACQ_BACKOFF);
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
  unsigned int elapsed_ms = jiffies_to_msecs(jiffies - started);
  unsigned int gap_ms = ssf_mems_serial_gap_ms(data);
  unsigned int delay_ms, interval_ms;

  if (acq->phase == SSF_MEMS_ACQ_BACKOFF || acq->route == SSF_MEMS_ACQ_ROUTE_NONE)
    return SSF_MEMS_ACQ_RETRY_INITIAL_MS;
  mutex_lock(&acq->status_lock);
  interval_ms = acq->policy.interval_ms;
  mutex_unlock(&acq->status_lock);
  delay_ms = acq->route == SSF_MEMS_ACQ_ROUTE_FEATURE && interval_ms > elapsed_ms ?
      interval_ms - elapsed_ms : gap_ms;
  if (acq->active_config.mode == SSF_MEMS_ACQ_MODE_ALTERNATING) {
    if (time_after_eq(jiffies, acq->phase_until))
      return gap_ms;
    delay_ms = min(delay_ms,
        (unsigned int)jiffies_to_msecs(acq->phase_until - jiffies));
  }
  return max(gap_ms, delay_ms);
}

/* 完成前先移除槽位；调用者取短锁后读取结果，保证请求不悬空。 */
static void ssf_mems_manual_complete_locked(struct ssf_mems_acquisition_state *acq,
    unsigned int route, int result) {
  struct ssf_mems_capture_request *request = acq->requests[route];

  if (!request)
    return;
  acq->requests[route] = NULL;
  request->result = result;
  complete(&request->done);
}

static void ssf_mems_manual_flush(struct ssf_mems_acquisition_state *acq, int result) {
  unsigned int route;

  mutex_lock(&acq->status_lock);
  for (route = 0; route < SSF_MEMS_ACQ_ROUTE_COUNT; route++)
    ssf_mems_manual_complete_locked(acq, route, result);
  mutex_unlock(&acq->status_lock);
}

/* 已开始的显式振动窗口沿用快照；新配置等窗口完成后应用。 */
static bool ssf_mems_manual_window_running(struct ssf_mems_acquisition_state *acq) {
  struct ssf_mems_capture_request *request;
  unsigned int route;
  bool running = false;

  mutex_lock(&acq->status_lock);
  for (route = 0; route < SSF_MEMS_ACQ_ROUTE_COUNT; route++) {
    const struct ssf_mems_acquisition_route_desc *desc = ssf_mems_acquisition_route_desc(route);

    request = acq->requests[route];
    if (request && request->started && desc->manual_window)
      running = true;
  }
  WRITE_ONCE(acq->requests_changed, false);
  mutex_unlock(&acq->status_lock);
  return running;
}

/* 在工作线程中公平地执行一批手动请求；持续窗口不跨批持锁。 */
static bool ssf_mems_manual_step(struct ssf_mems_data *data) {
  struct ssf_mems_acquisition_state *acq = &data->acquisition;
  struct ssf_mems_capture_request *request = NULL;
  const struct ssf_mems_acquisition_route_desc *desc;
  unsigned int i, route = 0;
  bool first;
  int ret;

  mutex_lock(&acq->status_lock);
  for (i = 0; i < SSF_MEMS_ACQ_ROUTE_COUNT; i++) {
    route = (acq->manual_cursor + i) % SSF_MEMS_ACQ_ROUTE_COUNT;
    request = acq->requests[route];
    if (request)
      break;
  }
  if (!request) {
    mutex_unlock(&acq->status_lock);
    return false;
  }
  if (request->cancelled) {
    ssf_mems_manual_complete_locked(acq, route, -EINTR);
    mutex_unlock(&acq->status_lock);
    return true;
  }
  desc = ssf_mems_acquisition_route_desc(route);
  if (request->started && desc->manual_window && time_after_eq(jiffies, request->until)) {
    ssf_mems_manual_complete_locked(acq, route, 0);
    mutex_unlock(&acq->status_lock);
    return true;
  }
  first = !request->started;
  if (first) {
    request->started = true;
    request->until = jiffies + msecs_to_jiffies(
        *ssf_mems_acquisition_duration(&request->config, route));
  }
  mutex_unlock(&acq->status_lock);
  acq->manual_cursor = (route + 1) % SSF_MEMS_ACQ_ROUTE_COUNT;
  ret = ssf_mems_acquisition_batch(data, route, &request->config, first, true);
  if (ret)
    ssf_mems_acquisition_error(data, ret);
  mutex_lock(&acq->status_lock);
  if (READ_ONCE(acq->stopping))
    ret = -ENODEV;
  else if (request->cancelled)
    ret = -EINTR;
  if (ret || !desc->manual_window || time_after_eq(jiffies, request->until))
    ssf_mems_manual_complete_locked(acq, route, ret);
  mutex_unlock(&acq->status_lock);
  if (!acq->online)
    ssf_mems_manual_flush(acq, -ENODATA);
  return true;
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
    /* 与串口配置和手动事务互斥；主机配置不取此锁。 */
    mutex_lock(&acq->lock);
    if (ssf_mems_acquisition_stopping(acq) == true) {
      mutex_unlock(&acq->lock);
      break;
    }
    if (acq->firmware_blocked == true) {
      /* 末包结果不确定时只等待人工处理，不能用自动重连继续写配置。 */
      ssf_mems_manual_flush(acq, -EBUSY);
      mutex_unlock(&acq->lock);
      ssf_mems_acquisition_wait(acq, SSF_MEMS_ACQ_RETRY_INITIAL_MS, false);
      continue;
    }
    if (!ssf_mems_manual_window_running(acq))
      ssf_mems_acquisition_apply_config(data);
    started = jiffies;
    if (!acq->online || !ssf_mems_acquisition_ready(acq) || acq->reconnect) {
      ssf_mems_manual_flush(acq, -ENODATA);
      WRITE_ONCE(acq->reconnect, false);
      ret = ssf_mems_acquisition_connect(data);
      if (ret != 0) {
        ssf_mems_acquisition_offline(data);
        ssf_mems_acquisition_phase(acq, SSF_MEMS_ACQ_BACKOFF);
        delay_ms = acq->retry_ms;
        acq->retry_ms = min(SSF_MEMS_ACQ_RETRY_MAX_MS, acq->retry_ms * 2U);
        if (!ssf_mems_acquisition_stopping(acq))
          dev_warn_ratelimited(&data->serdev->dev,
                              "sensor connection failed: %d, retry in %u ms\n",
                              ret, delay_ms);
      } else {
        delay_ms = ssf_mems_acquisition_next_delay(data, started);
      }
    } else if (ssf_mems_manual_step(data)) {
      delay_ms = ssf_mems_serial_gap_ms(data);
    } else {
      ret = ssf_mems_acquisition_collect(data);
      if (ret != 0)
        ssf_mems_acquisition_error(data, ret);
      delay_ms = ssf_mems_acquisition_next_delay(data, started);
      /* 不支持的采样组合限速重试，避免反复触发。 */
      if (ret == -EOPNOTSUPP)
        delay_ms = max(delay_ms, SSF_MEMS_ACQ_RETRY_INITIAL_MS);
    }
    mutex_unlock(&acq->lock);
    ssf_mems_acquisition_wait(acq, delay_ms, true);
  }
  mutex_lock(&acq->lock);
  ssf_mems_manual_flush(acq, -ENODEV);
  ssf_mems_acquisition_offline(data);
  ssf_mems_acquisition_phase(acq, SSF_MEMS_ACQ_STOPPED);
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
  ssf_mems_acquisition_phase(acq, SSF_MEMS_ACQ_STARTUP);
  /* 设备树仅提供初次匹配提示；原始和特征通信共用型号表中的工作目标。 */
  acq->target_baudrate = data->sensor->profile.default_work_baudrate;
  acq->target_sampling_rate_index =
      data->sensor->profile.default_sampling_rate_index;
  acq->retry_ms = SSF_MEMS_ACQ_RETRY_INITIAL_MS;
  acq->config.raw_batch_samples = SSF_MEMS_RAW_BATCH_SAMPLES;
  acq->config.raw_manual = false;
  acq->config.raw_duration_ms = ssf_mems_acquisition_window(
      &data->serdev->dev, "sange-cbm,raw-duration-ms", true);
  acq->config.feature_duration_ms = ssf_mems_acquisition_window(
      &data->serdev->dev, "sange-cbm,feature-duration-ms", false);
  /* 兼容旧设备树 raw-duration-ms=0 的“仅特征”语义；保存合法原始窗口供以后切换。 */
  acq->config.mode = acq->config.raw_duration_ms == 0 ? SSF_MEMS_ACQ_MODE_FEATURE :
                                        SSF_MEMS_ACQ_MODE_ALTERNATING;
  if (acq->config.raw_duration_ms == 0)
    acq->config.raw_duration_ms = SSF_MEMS_ACQ_WINDOW_MS;
  if (data->protocol.parity != SSF_MEMS_PARITY_NONE) {
    acq->config.mode = SSF_MEMS_ACQ_MODE_FEATURE;
    dev_info(&data->serdev->dev,
             "finite raw mode requires no parity; using feature mode\n");
  }
  acq->config.sound_duration_ms = SSF_MEMS_ACQ_WINDOW_MS;
  {
    unsigned int route;

    for (route = 0; route < SSF_MEMS_ACQ_ROUTE_COUNT; route++)
      acq->config.alternating_en[route] = ssf_mems_acquisition_route_desc(route)->supported(data);
  }
  acq->config_changed = false;
  acq->poll_changed = false;
  acq->phase_until = 0;
  acq->route = SSF_MEMS_ACQ_ROUTE_NONE;
  acq->visible_route = SSF_MEMS_ACQ_ROUTE_NONE;
  acq->config_version = 1;
  acq->applied_version = 0;
  acq->manual_cursor = SSF_MEMS_ACQ_ROUTE_SOUND;
  memset(acq->window_start, 0, sizeof(acq->window_start));
  memset(acq->requests, 0, sizeof(acq->requests));
  acq->requests_changed = false;
  acq->busy = acq->manual_active = false;
  ssf_mems_acquisition_policy_init(&acq->policy, data->sensor, jiffies);
  acq->config.poll = acq->policy.config;
  acq->active_config = acq->config;
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
    *config = acq->config;
  }
  mutex_unlock(&acq->status_lock);
  return ret;
}

/* 仅短锁修改用户配置；绝不在 sysfs 配置路径等待串口事务。 */
static void ssf_mems_acquisition_config_changed(struct ssf_mems_acquisition_state *acq) {
  acq->config_version++;
  WRITE_ONCE(acq->config_changed, true);
  wake_up_interruptible(&acq->waitq);
}

int ssf_mems_acquisition_set_mode(struct ssf_mems_data *data,
                                  enum ssf_mems_acquisition_mode mode) {
  struct ssf_mems_acquisition_state *acq;
  int ret = 0;

  if (!data || !data->sensor_ops || (unsigned int)mode >= SSF_MEMS_ACQ_MODE_COUNT)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->status_lock);
  if (READ_ONCE(acq->stopping))
    ret = -ENODEV;
  else if (!ssf_mems_acquisition_mode_supported(data, mode) ||
           (mode != SSF_MEMS_ACQ_MODE_FEATURE &&
            data->protocol.parity != SSF_MEMS_PARITY_NONE))
    ret = -EOPNOTSUPP;
  else if (acq->config.mode != mode) {
    acq->config.mode = mode;
    ssf_mems_acquisition_config_changed(acq);
  }
  mutex_unlock(&acq->status_lock);
  return ret;
}

int ssf_mems_acquisition_set_duration(
    struct ssf_mems_data *data, enum ssf_mems_acquisition_route route,
    unsigned int duration_ms) {
  struct ssf_mems_acquisition_state *acq;
  unsigned int *window;
  int ret = 0;

  if (!data || !data->sensor_ops || (unsigned int)route >= SSF_MEMS_ACQ_ROUTE_COUNT ||
      duration_ms < SSF_MEMS_ACQ_MIN_DURATION_MS || duration_ms > SSF_MEMS_ACQ_MAX_DURATION_MS)
    return -EINVAL;
  if (!ssf_mems_acquisition_route_desc(route)->supported(data))
    return -EOPNOTSUPP;
  acq = &data->acquisition;
  mutex_lock(&acq->status_lock);
  window = ssf_mems_acquisition_duration(&acq->config, route);
  if (READ_ONCE(acq->stopping))
    ret = -ENODEV;
  else if (*window != duration_ms) {
    *window = duration_ms;
    ssf_mems_acquisition_config_changed(acq);
  }
  mutex_unlock(&acq->status_lock);
  return ret;
}

int ssf_mems_acquisition_set_alternating_en(
    struct ssf_mems_data *data, enum ssf_mems_acquisition_route route, bool enable) {
  struct ssf_mems_acquisition_state *acq;
  unsigned int i, selected = 0;
  int ret = 0;

  if (!data || !data->sensor_ops || (unsigned int)route >= SSF_MEMS_ACQ_ROUTE_COUNT)
    return -EINVAL;
  if (!ssf_mems_acquisition_route_desc(route)->supported(data))
    return -EOPNOTSUPP;
  acq = &data->acquisition;
  mutex_lock(&acq->status_lock);
  for (i = 0; i < SSF_MEMS_ACQ_ROUTE_COUNT; i++)
    selected += acq->config.alternating_en[i];
  if (READ_ONCE(acq->stopping))
    ret = -ENODEV;
  else if (!enable && acq->config.alternating_en[route] && selected == 1)
    ret = -EINVAL;
  else if (acq->config.alternating_en[route] != enable) {
    acq->config.alternating_en[route] = enable;
    ssf_mems_acquisition_config_changed(acq);
  }
  mutex_unlock(&acq->status_lock);
  return ret;
}

/* 新长度在下一批使用，不重启当前路线窗口。 */
int ssf_mems_acquisition_set_raw_samples(struct ssf_mems_data *data, u32 samples) {
  struct ssf_mems_acquisition_state *acq;
  int ret = 0;

  if (!data || !ssf_mems_sensor_raw_samples_valid(data->sensor, samples))
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->status_lock);
  if (READ_ONCE(acq->stopping))
    ret = -ENODEV;
  else if (acq->config.raw_batch_samples != samples) {
    acq->config.raw_batch_samples = samples;
    acq->config_version++;
    WRITE_ONCE(acq->poll_changed, true);
    wake_up_interruptible(&acq->waitq);
  }
  mutex_unlock(&acq->status_lock);
  return ret;
}

int ssf_mems_acquisition_set_raw_manual(struct ssf_mems_data *data, bool manual) {
  struct ssf_mems_acquisition_state *acq;
  int ret = 0;

  if (!data)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->status_lock);
  if (READ_ONCE(acq->stopping))
    ret = -ENODEV;
  else if (acq->config.raw_manual != manual) {
    acq->config.raw_manual = manual;
    ssf_mems_acquisition_config_changed(acq);
  }
  mutex_unlock(&acq->status_lock);
  return ret;
}

/* sysfs 调用只提交请求；由同一工作线程在批次边界执行，保持同步 ABI。 */
static int ssf_mems_acquisition_manual(
    struct ssf_mems_data *data, enum ssf_mems_acquisition_route route) {
  struct ssf_mems_acquisition_state *acq;
  const struct ssf_mems_acquisition_route_desc *desc = ssf_mems_acquisition_route_desc(route);
  struct ssf_mems_capture_request request = {0};
  int ret;

  if (!data || !data->sensor_ops || !desc)
    return -EINVAL;
  if (!desc->supported(data))
    return -EOPNOTSUPP;
  acq = &data->acquisition;
  init_completion(&request.done);
  mutex_lock(&acq->status_lock);
  if (READ_ONCE(acq->stopping))
    ret = -ENODEV;
  else if (acq->firmware_blocked)
    ret = -EBUSY;
  else if (desc->manual_window &&
           (!acq->config.raw_manual || acq->config.mode != desc->single_mode))
    ret = -EINVAL;
  else if (!acq->online || !ssf_mems_acquisition_ready(acq) || acq->reconnect)
    ret = -ENODATA;
  else if (!READ_ONCE(acq->thread))
    ret = -ENODEV;
  else if (acq->requests[route])
    ret = -EBUSY;
  else {
    request.config = acq->config;
    acq->requests[route] = &request;
    WRITE_ONCE(acq->requests_changed, true);
    ret = 0;
  }
  mutex_unlock(&acq->status_lock);
  if (ret)
    return ret;
  wake_up_interruptible(&acq->waitq);
  ret = wait_for_completion_interruptible(&request.done);
  if (ret) {
    mutex_lock(&acq->status_lock);
    request.cancelled = true;
    WRITE_ONCE(acq->requests_changed, true);
    mutex_unlock(&acq->status_lock);
    wake_up_interruptible(&acq->waitq);
    /* 不能在工作线程仍使用请求/接收缓存时返回，或自动重放副作用命令。 */
    wait_for_completion(&request.done);
  }
  mutex_lock(&acq->status_lock);
  ret = ret ? -EINTR : request.result;
  mutex_unlock(&acq->status_lock);
  return ret;
}

int ssf_mems_acquisition_capture_window(struct ssf_mems_data *data) {
  return ssf_mems_acquisition_manual(data, SSF_MEMS_ACQ_ROUTE_RAW);
}
int ssf_mems_acquisition_capture_sound(struct ssf_mems_data *data) {
  return ssf_mems_acquisition_manual(data, SSF_MEMS_ACQ_ROUTE_SOUND);
}

bool ssf_mems_acquisition_manual_pending(struct ssf_mems_data *data) {
  struct ssf_mems_acquisition_state *acq;
  unsigned int route;
  bool pending = false;

  if (!data)
    return false;
  acq = &data->acquisition;
  mutex_lock(&acq->status_lock);
  for (route = 0; route < SSF_MEMS_ACQ_ROUTE_COUNT; route++)
    pending |= acq->requests[route] != NULL;
  mutex_unlock(&acq->status_lock);
  return pending;
}

/* 修改主机特征轮询配置；不截断在途原始批次，不写传感器配置。 */
int ssf_mems_acquisition_set_poll_setting(struct ssf_mems_data *data,
                                          enum ssf_mems_poll_setting setting,
                                          unsigned int value) {
  struct ssf_mems_acquisition_state *acq;
  struct ssf_mems_poll_config config;
  struct ssf_mems_acquisition_policy validation;
  unsigned int previous_value;
  int ret = 0;

  if (data == NULL)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->status_lock);
  if (READ_ONCE(acq->stopping) == true) {
    ret = -ENODEV;
    goto out;
  }
  config = acq->config.poll;
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
  validation = acq->policy;
  ret = ssf_mems_acquisition_policy_configure(&validation, &config, jiffies);
  if (ret == 0) {
    acq->config.poll = config;
    acq->config_version++;
    if (setting == SSF_MEMS_POLL_SETTING_MODE ||
        (config.mode == SSF_MEMS_POLL_FIXED && setting == SSF_MEMS_POLL_SETTING_FIXED_INTERVAL) ||
        (config.mode == SSF_MEMS_POLL_ADAPTIVE && setting != SSF_MEMS_POLL_SETTING_FIXED_INTERVAL)) {
      WRITE_ONCE(acq->poll_changed, true);
      wake_up_interruptible(&acq->waitq);
    }
  }
out:
  mutex_unlock(&acq->status_lock);
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
  WRITE_ONCE(data->acquisition.thread, thread);
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
    WRITE_ONCE(acq->thread, NULL);
  }
  ret = ssf_mems_protocol_recover_capture(data);
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
  if (ssf_mems_acquisition_manual_pending(data)) {
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
  if (ssf_mems_acquisition_manual_pending(data)) {
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
  } else if (ssf_mems_acquisition_manual_pending(data)) {
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
      mutex_lock(&acq->status_lock);
      ssf_mems_acquisition_policy_restart(&acq->policy, jiffies);
      mutex_unlock(&acq->status_lock);
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

  unsigned int route;

  mutex_lock(&acq->status_lock);

  status->connection = acq->phase;
  status->route = acq->visible_route;
  status->requested_mode = acq->config.mode;
  status->applied_mode = acq->active_config.mode;
  status->config_version = acq->config_version;
  status->applied_version = acq->applied_version;
  status->busy = acq->busy;
  status->manual_active = acq->manual_active;
  status->firmware_blocked = READ_ONCE(acq->firmware_blocked);
  status->manual_wait = acq->active_config.mode == SSF_MEMS_ACQ_MODE_RAW &&
                        acq->active_config.raw_manual;
  status->manual_pending = false;
  for (route = 0; route < SSF_MEMS_ACQ_ROUTE_COUNT; route++)
    status->manual_pending |= acq->requests[route] != NULL;
  status->online = acq->online;
  status->have_sample = acq->have_sample;
  status->interval_ms = acq->policy.interval_ms;
  status->sample_age_ms = acq->have_sample ?
                            jiffies_to_msecs(jiffies - acq->last_sample) : 0;
  mutex_unlock(&acq->status_lock);
}
