// SPDX-License-Identifier: GPL-2.0-only
#include "acquisition_policy.h"

#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/string.h>

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

/* 按字段比较，避免结构体尾部填充字节影响重复判定。 */
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

static void ssf_mems_reset_window(struct ssf_mems_acquisition_policy *policy) {
  policy->comparisons = 0;
  policy->repeats = 0;
  policy->max_read_ms = 0;
}

static void ssf_mems_poll_faster(struct ssf_mems_acquisition_policy *policy) {
  unsigned int step = max(1U, policy->interval_ms / 20U);

  policy->interval_ms = max(SSF_MEMS_POLL_MIN_MS, policy->interval_ms - step);
}

void ssf_mems_acquisition_policy_init(
    struct ssf_mems_acquisition_policy *policy, unsigned long now) {
  memset(policy, 0, sizeof(*policy));
  policy->interval_ms = SSF_MEMS_POLL_INITIAL_MS;
  policy->last_contact = now;
  policy->next_probe = now + msecs_to_jiffies(SSF_MEMS_POLL_PROBE_MS);
}

void ssf_mems_acquisition_policy_sample(
    struct ssf_mems_acquisition_policy *policy,
    const struct ssf_mems_sensor_data *sample, unsigned int read_ms,
    unsigned long now) {
  policy->link_failures = 0;
  policy->last_contact = now;
  policy->max_read_ms = max(policy->max_read_ms, read_ms);
  if (policy->have_previous) {
    policy->comparisons++;
    if (ssf_mems_sample_equal(&policy->previous, sample))
      policy->repeats++;
  }
  policy->previous = *sample;
  policy->have_previous = true;

  /* 静止场景定期小幅提速试探，避免长期停留在最低轮询频率。 */
  if (time_after_eq(now, policy->next_probe)) {
    if (policy->max_read_ms <= policy->interval_ms / 2U)
      ssf_mems_poll_faster(policy);
    policy->next_probe = now + msecs_to_jiffies(SSF_MEMS_POLL_PROBE_MS);
    ssf_mems_reset_window(policy);
    return;
  }
  if (policy->comparisons < SSF_MEMS_POLL_WINDOW)
    return;

  if (policy->repeats * 100U > policy->comparisons * 50U)
    policy->interval_ms = min(SSF_MEMS_POLL_MAX_MS,
                             policy->interval_ms +
                                 max(1U, policy->interval_ms / 5U));
  else if (policy->repeats * 100U < policy->comparisons * 10U &&
           policy->max_read_ms <= policy->interval_ms / 2U)
    ssf_mems_poll_faster(policy);
  ssf_mems_reset_window(policy);
}

bool ssf_mems_acquisition_policy_error(
    struct ssf_mems_acquisition_policy *policy, int error, unsigned long now) {
  policy->have_previous = false;
  ssf_mems_reset_window(policy);

  if (error == -EREMOTEIO) {
    policy->link_failures = 0;
    policy->last_contact = now;
    return false;
  }
  if (error != -ETIMEDOUT && error != -EIO && error != -EBADMSG)
    return false;
  if (policy->link_failures < SSF_MEMS_LINK_FAILURE_LIMIT)
    policy->link_failures++;
  return policy->link_failures >= SSF_MEMS_LINK_FAILURE_LIMIT &&
         time_after_eq(now, policy->last_contact +
                               msecs_to_jiffies(SSF_MEMS_LINK_FAILURE_MS));
}
