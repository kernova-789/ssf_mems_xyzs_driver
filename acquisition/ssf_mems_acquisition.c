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

#define SSF_MEMS_ACQ_STARTUP_MS 1000U
#define SSF_MEMS_ACQ_TIMEOUT_MS 200U
#define SSF_MEMS_ACQ_SWITCH_SETTLE_MS 20U
#define SSF_MEMS_ACQ_RETRY_INITIAL_MS 1000U
#define SSF_MEMS_ACQ_RETRY_MAX_MS 30000U
/* 至少保留一段帧间空闲时间；不在超时后立即背靠背发送。 */
#define SSF_MEMS_ACQ_GAP_MS 2U
#define SSF_MEMS_ACQ_TARGET_BAUDRATE SSF_MEMS_BAUDRATE_115200

static bool ssf_mems_acquisition_stopping(struct ssf_mems_acquisition_state *acq) {
  return READ_ONCE(acq->stopping) || kthread_should_stop();
}

static void ssf_mems_acquisition_wait(struct ssf_mems_acquisition_state *acq,
                                     unsigned int delay_ms, bool reconfigure) {
  wait_event_interruptible_timeout(
      acq->waitq, ssf_mems_acquisition_stopping(acq) ||
                      (reconfigure && READ_ONCE(acq->reconnect)),
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

  if (!baudrate)
    baudrate = 9600;
  return max(SSF_MEMS_ACQ_GAP_MS, DIV_ROUND_UP(bits * 1000U, baudrate));
}

/* sysfs 可以选择较低速率；响应超时须容纳最长寄存器响应的线上传输时间。 */
static unsigned int ssf_mems_acquisition_read_timeout(
    struct ssf_mems_xyzs_data *data) {
  unsigned int baudrate = READ_ONCE(data->protocol.host_baudrate);
  unsigned int bits = data->protocol.parity == SSF_MEMS_PARITY_NONE ? 10U : 11U;
  unsigned int bytes = 5U + SSF_MEMS_MODBUS_READ_MAX_REGS * 2U;

  if (!baudrate)
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
  if (ret)
    return ret;
  if (ssf_mems_acquisition_stopping(acq))
    return -ENODEV;
  ret = ssf_mems_protocol_get_features(data->serdev, &features);
  if (ret)
    return ret;

  now = jiffies;
  acq->last_sample = now;
  acq->have_sample = true;
  acq->online = true;
  ssf_mems_acquisition_policy_sample(&acq->policy, &features,
                                    jiffies_to_msecs(now - started), now);
  ret = ssf_mems_iio_publish_features(data, &features, 0);
  /* buffer 消费错误不等于串口通信错误，缓存仍是一份完整的有效结果。 */
  if (ret)
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
  if (ret || ssf_mems_acquisition_stopping(acq))
    return ret ? ret : -ENODEV;

  if (ssf_mems_baudrate_to_value(found) !=
      ssf_mems_baudrate_to_value(acq->target_baudrate)) {
    acq->phase = SSF_MEMS_ACQ_SWITCH;
    ret = ssf_mems_protocol_set_baudrate(data->serdev, acq->target_baudrate,
                                        SSF_MEMS_ACQ_TIMEOUT_MS);
    if (ret)
      return ret;
  }

  /* 假设传感器先用旧速率发出应答，随后切换；此处给切换留出余量。
   * 不写校验位，不发送保存或重启命令。 */
  acq->phase = SSF_MEMS_ACQ_VERIFY;
  ssf_mems_acquisition_wait(acq, SSF_MEMS_ACQ_SWITCH_SETTLE_MS, false);
  if (ssf_mems_acquisition_stopping(acq))
    return -ENODEV;
  ssf_mems_acquisition_policy_init(&acq->policy, jiffies);
  ret = ssf_mems_acquisition_sample(data);
  if (ret)
    return ret;

  acq->phase = SSF_MEMS_ACQ_POLL;
  acq->retry_ms = SSF_MEMS_ACQ_RETRY_INITIAL_MS;
  dev_info(&data->serdev->dev, "sensor online, baud rate %d, poll period %u ms\n",
           ssf_mems_baudrate_to_value(acq->target_baudrate),
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
    if (ssf_mems_acquisition_stopping(acq)) {
      mutex_unlock(&acq->lock);
      break;
    }
    started = jiffies;
    if (acq->phase != SSF_MEMS_ACQ_POLL || acq->reconnect) {
      WRITE_ONCE(acq->reconnect, false);
      ret = ssf_mems_acquisition_connect(data);
      if (ret) {
        ssf_mems_acquisition_offline(data);
        acq->phase = SSF_MEMS_ACQ_BACKOFF;
        delay_ms = acq->retry_ms;
        acq->retry_ms = min(SSF_MEMS_ACQ_RETRY_MAX_MS, acq->retry_ms * 2U);
        if (!ssf_mems_acquisition_stopping(acq))
          dev_warn_ratelimited(&data->serdev->dev,
                              "sensor connection failed: %d, retry in %u ms\n",
                              ret, delay_ms);
      } else {
        /* 首次验证也是一份样本，下一轮从本次完成后计算间隔。 */
        delay_ms = acq->policy.interval_ms;
      }
    } else {
      ret = ssf_mems_acquisition_sample(data);
      if (ret) {
        if (acq->have_sample &&
            time_after_eq(jiffies, acq->last_sample +
                                      msecs_to_jiffies(SSF_MEMS_LINK_FAILURE_MS)))
          ssf_mems_protocol_invalidate_features(data);
        if (ssf_mems_acquisition_policy_error(&acq->policy, ret, jiffies)) {
          ssf_mems_acquisition_offline(data);
          acq->phase = SSF_MEMS_ACQ_BACKOFF;
          dev_warn_ratelimited(&data->serdev->dev,
                              "sensor communication lost, restarting baud discovery\n");
        } else if (ret != -ENODEV) {
          dev_warn_ratelimited(&data->serdev->dev,
                              "failed to read sensor features: %d\n", ret);
        }
      }
      if (acq->phase == SSF_MEMS_ACQ_BACKOFF) {
        delay_ms = SSF_MEMS_ACQ_RETRY_INITIAL_MS;
      } else {
        unsigned int elapsed_ms = jiffies_to_msecs(jiffies - started);
        unsigned int gap_ms = ssf_mems_acquisition_gap(data);

        delay_ms = acq->policy.interval_ms > elapsed_ms ?
                       acq->policy.interval_ms - elapsed_ms : gap_ms;
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

int ssf_mems_acquisition_init(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_acquisition_state *acq;

  if (!data || !data->serdev)
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
  acq->target_baudrate = SSF_MEMS_ACQ_TARGET_BAUDRATE;
  acq->retry_ms = SSF_MEMS_ACQ_RETRY_INITIAL_MS;
  ssf_mems_acquisition_policy_init(&acq->policy, jiffies);
  return 0;
}

int ssf_mems_acquisition_start(struct ssf_mems_xyzs_data *data) {
  struct task_struct *thread;

  if (!data || !data->indio_dev)
    return -EINVAL;
  if (READ_ONCE(data->acquisition.stopping))
    return -ENODEV;
  if (data->acquisition.thread)
    return -EBUSY;
  thread = kthread_run(ssf_mems_acquisition_thread, data, "ssf-mems-%s",
                       dev_name(&data->serdev->dev));
  if (IS_ERR(thread))
    return PTR_ERR(thread);
  data->acquisition.thread = thread;
  return 0;
}

void ssf_mems_acquisition_stop(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_acquisition_state *acq;

  if (!data)
    return;
  acq = &data->acquisition;
  WRITE_ONCE(acq->stopping, true);
  wake_up_interruptible(&acq->waitq);
  if (acq->thread) {
    kthread_stop(acq->thread);
    acq->thread = NULL;
  }
}

int ssf_mems_acquisition_get_baudrate(struct ssf_mems_xyzs_data *data,
                                    enum ssf_mems_baudrate *baudrate) {
  struct ssf_mems_acquisition_state *acq;
  int ret;

  if (!data || !baudrate)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping)) {
    ret = -ENODEV;
    goto out;
  }
  ret = ssf_mems_protocol_get_baudrate(data->serdev, baudrate,
                                      SSF_MEMS_ACQ_TIMEOUT_MS);
  if (ret || ssf_mems_baudrate_to_value(*baudrate) !=
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

  if (!data || (unsigned int)baudrate >= SSF_MEMS_BAUDRATE_MAX)
    return -EINVAL;
  acq = &data->acquisition;
  mutex_lock(&acq->lock);
  if (READ_ONCE(acq->stopping)) {
    ret = -ENODEV;
    goto out;
  }
  ret = ssf_mems_protocol_set_baudrate(data->serdev, baudrate,
                                      SSF_MEMS_ACQ_TIMEOUT_MS);
  if (!ret)
    acq->target_baudrate = baudrate;
  /* 即使写入失败，传感器也可能已经切换，只是应答丢失；重新匹配实际速率。 */
  ssf_mems_acquisition_offline(data);
  WRITE_ONCE(acq->reconnect, true);
  wake_up_interruptible(&acq->waitq);
out:
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
