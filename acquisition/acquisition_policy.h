/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/types.h>
#include "sensor_data.h"

/* 传感器约每 4.4~4.8 s 更新一批，固定每 1 s 读一次。
 * 周期指相邻两轮读取的开始时间间隔；相同上下限使自适应调速不改变周期。
 * 调速算法保留，后续需要自适应时可重新设置范围。 */
#define SSF_MEMS_POLL_INITIAL_MS 1000U
#define SSF_MEMS_POLL_MIN_MS 1000U
#define SSF_MEMS_POLL_MAX_MS 1000U
#define SSF_MEMS_POLL_WINDOW 32U
#define SSF_MEMS_POLL_PROBE_MS 30000U
#define SSF_MEMS_LINK_FAILURE_LIMIT 5U
#define SSF_MEMS_LINK_FAILURE_MS 3000U

struct ssf_mems_acquisition_policy {
  struct ssf_mems_sensor_data previous;
  bool have_previous;
  unsigned int interval_ms;
  unsigned int comparisons;
  unsigned int repeats;
  unsigned int max_read_ms;
  unsigned int link_failures;
  unsigned long last_contact;
  unsigned long next_probe;
};

void ssf_mems_acquisition_policy_init(
    struct ssf_mems_acquisition_policy *policy, unsigned long now);
void ssf_mems_acquisition_policy_sample(
    struct ssf_mems_acquisition_policy *policy,
    const struct ssf_mems_sensor_data *sample, unsigned int read_ms,
    unsigned long now);
/* 只有通信类错误累计到次数和时间阈值才返回 true；合法异常帧证明链路畅通。 */
bool ssf_mems_acquisition_policy_error(
    struct ssf_mems_acquisition_policy *policy, int error, unsigned long now);
