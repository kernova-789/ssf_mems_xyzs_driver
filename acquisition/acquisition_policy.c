// SPDX-License-Identifier: GPL-2.0-only
#include "acquisition_policy.h"

#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/string.h>

/* 比较两份单轴特征。
 * 返回：true 相同，false 不同。
 */
static bool ssf_mems_axis_equal(const struct ssf_mems_axis_features *a,
                                const struct ssf_mems_axis_features *b) {
  return a->high_freq_acc_rms_x100 == b->high_freq_acc_rms_x100 &&
         a->low_freq_velocity_rms_x100 == b->low_freq_velocity_rms_x100 &&
         a->acc_peak_to_peak_x100 == b->acc_peak_to_peak_x100 &&
         a->acc_peak_x100 == b->acc_peak_x100 &&
         a->acc_rms_x100 == b->acc_rms_x100 &&
         a->kurtosis_x100 == b->kurtosis_x100 &&
         a->velocity_rms_x100 == b->velocity_rms_x100;
}

/* 比较两份完整特征。
 * 返回：true 相同，false 不同。
 * 只比较字段，不比较结构体填充字节。
 */
static bool ssf_mems_sample_equal(const struct ssf_mems_sensor_data *a,
                                  const struct ssf_mems_sensor_data *b) {
  return ssf_mems_axis_equal(&a->x, &b->x) &&
         ssf_mems_axis_equal(&a->y, &b->y) &&
         ssf_mems_axis_equal(&a->z, &b->z) &&
         a->temperature_x100 == b->temperature_x100 &&
         a->sound.rms_db_x100 == b->sound.rms_db_x100 &&
         a->sound.peak_db_x100 == b->sound.peak_db_x100 &&
         a->sound.peak_to_peak_db_x100 == b->sound.peak_to_peak_db_x100 &&
         a->zero_crossing_rate_percent == b->zero_crossing_rate_percent &&
         a->spectral_centroid_hz_x10 == b->spectral_centroid_hz_x10 &&
         a->spectral_flux_x100 == b->spectral_flux_x100 &&
         a->startup_flags == b->startup_flags;
}

/* 清空本轮轮询统计。
 * 返回：无。
 */
static void ssf_mems_reset_window(struct ssf_mems_acquisition_policy *policy) {
  policy->comparisons = 0;
  policy->repeats = 0;
  policy->max_read_ms = 0;
}

/* 小幅缩短轮询周期。
 * 返回：无。
 * 周期不低于设定下限。
 */
static void ssf_mems_poll_faster(struct ssf_mems_acquisition_policy *policy) {
  unsigned int step = max(1U, policy->interval_ms / 20U);

  policy->interval_ms = max(policy->config.min_interval_ms,
                            policy->interval_ms - step);
}

/* 初始化轮询策略。
 * 返回：无。
 */
void ssf_mems_acquisition_policy_init(
    struct ssf_mems_acquisition_policy *policy, unsigned long now) {
  memset(policy, 0, sizeof(*policy));
  policy->config.mode = SSF_MEMS_POLL_FIXED;
  policy->config.fixed_interval_ms = SSF_MEMS_POLL_DEFAULT_INTERVAL_MS;
  policy->config.min_interval_ms = SSF_MEMS_POLL_DEFAULT_MIN_MS;
  policy->config.max_interval_ms = SSF_MEMS_POLL_DEFAULT_MAX_MS;
  policy->interval_ms = policy->config.fixed_interval_ms;
  ssf_mems_acquisition_policy_restart(policy, now);
}

/* 清空历史统计并保留用户配置和当前周期。
 * 返回：无。
 */
void ssf_mems_acquisition_policy_restart(
    struct ssf_mems_acquisition_policy *policy, unsigned long now) {
  policy->have_previous = false;
  ssf_mems_reset_window(policy);
  ssf_mems_acquisition_policy_contact(policy, now);
  policy->next_probe = now + msecs_to_jiffies(SSF_MEMS_POLL_PROBE_MS);
}

/* 更新轮询配置并使当前周期满足新的限制。
 * 返回：0 成功，-EINVAL 模式、范围或上下限无效。
 */
int ssf_mems_acquisition_policy_configure(
    struct ssf_mems_acquisition_policy *policy,
    const struct ssf_mems_poll_config *config, unsigned long now) {
  bool active_changed;

  if (policy == NULL || config == NULL ||
      (unsigned int)config->mode >= SSF_MEMS_POLL_MODE_COUNT ||
      config->fixed_interval_ms < SSF_MEMS_POLL_MIN_INTERVAL_MS ||
      config->fixed_interval_ms > SSF_MEMS_POLL_MAX_INTERVAL_MS ||
      config->min_interval_ms < SSF_MEMS_POLL_MIN_INTERVAL_MS ||
      config->max_interval_ms > SSF_MEMS_POLL_MAX_INTERVAL_MS ||
      config->min_interval_ms > config->max_interval_ms)
    return -EINVAL;

  active_changed = policy->config.mode != config->mode ||
      (config->mode == SSF_MEMS_POLL_FIXED &&
       policy->config.fixed_interval_ms != config->fixed_interval_ms) ||
      (config->mode == SSF_MEMS_POLL_ADAPTIVE &&
       (policy->config.min_interval_ms != config->min_interval_ms ||
        policy->config.max_interval_ms != config->max_interval_ms));
  policy->config = *config;
  if (active_changed == false)
    return 0;
  policy->interval_ms = config->mode == SSF_MEMS_POLL_FIXED ?
      config->fixed_interval_ms :
      min(config->max_interval_ms, max(config->min_interval_ms, policy->interval_ms));
  policy->have_previous = false;
  ssf_mems_reset_window(policy);
  policy->next_probe = now + msecs_to_jiffies(SSF_MEMS_POLL_PROBE_MS);
  return 0;
}

/* 记录一次有效通信，不清空自适应统计。
 * 返回：无。
 */
void ssf_mems_acquisition_policy_contact(
    struct ssf_mems_acquisition_policy *policy, unsigned long now) {
  policy->link_failures = 0;
  policy->last_contact = now;
}

/* 记录成功样本并调整轮询周期。
 * 返回：无。
 */
void ssf_mems_acquisition_policy_sample(
    struct ssf_mems_acquisition_policy *policy,
    const struct ssf_mems_sensor_data *sample, unsigned int read_ms,
    unsigned long now) {
  bool probe_due = time_after_eq(now, policy->next_probe);

  ssf_mems_acquisition_policy_contact(policy, now);
  if (policy->config.mode == SSF_MEMS_POLL_FIXED)
    return;
  policy->max_read_ms = max(policy->max_read_ms, read_ms);
  if (policy->have_previous == true) {
    policy->comparisons++;
    if (ssf_mems_sample_equal(&policy->previous, sample) == true)
      policy->repeats++;
  }
  policy->previous = *sample;
  policy->have_previous = true;

  /* 完整统计窗口优先，避免同一轮先减速又试探提速。 */
  if (policy->comparisons >= SSF_MEMS_POLL_WINDOW) {
    if (policy->repeats * 100U > policy->comparisons * 50U)
      policy->interval_ms = min(policy->config.max_interval_ms,
                               policy->interval_ms +
                                   max(1U, policy->interval_ms / 5U));
    else if (policy->repeats * 100U < policy->comparisons * 10U &&
             policy->max_read_ms <= policy->interval_ms / 2U)
      ssf_mems_poll_faster(policy);
    ssf_mems_reset_window(policy);
    if (probe_due == true)
      policy->next_probe = now + msecs_to_jiffies(SSF_MEMS_POLL_PROBE_MS);
    return;
  }
  /* 定期试探不清空未完成的统计，长周期和交替采集也能积满窗口。 */
  if (probe_due == true) {
    if (policy->max_read_ms <= policy->interval_ms / 2U)
      ssf_mems_poll_faster(policy);
    policy->next_probe = now + msecs_to_jiffies(SSF_MEMS_POLL_PROBE_MS);
  }
}

/* 记录通信错误并判断是否需要重连。
 * 返回：true 需要重连，false 不需要。
 * 收到合法从机异常仍视为链路可达。
 */
bool ssf_mems_acquisition_policy_error(
    struct ssf_mems_acquisition_policy *policy, int error, unsigned long now) {
  policy->have_previous = false;
  ssf_mems_reset_window(policy);

  if (error == -EREMOTEIO) {
    /* 收到合法异常应答仍说明线路可达，不能据此启动波特率扫描。 */
    policy->link_failures = 0;
    policy->last_contact = now;
    return false;
  }
  if (error != -ETIMEDOUT && error != -EIO && error != -EBADMSG)
    return false;
  /* 同时满足失败次数与无通信时间，避免单次丢包触发重连。 */
  if (policy->link_failures < SSF_MEMS_LINK_FAILURE_LIMIT)
    policy->link_failures++;
  return policy->link_failures >= SSF_MEMS_LINK_FAILURE_LIMIT &&
         time_after_eq(now, policy->last_contact +
                               msecs_to_jiffies(SSF_MEMS_LINK_FAILURE_MS));
}
