/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/types.h>
#include "sensor.h"

#define SSF_MEMS_POLL_DEFAULT_INTERVAL_MS 1000U /* 默认固定轮询周期。 */
#define SSF_MEMS_POLL_DEFAULT_MIN_MS 500U /* 默认自适应周期下限。 */
#define SSF_MEMS_POLL_DEFAULT_MAX_MS 5000U /* 默认自适应周期上限。 */
#define SSF_MEMS_POLL_MIN_INTERVAL_MS 100U /* 用户可设置的最短周期。 */
#define SSF_MEMS_POLL_MAX_INTERVAL_MS 60000U /* 用户可设置的最长周期。 */
#define SSF_MEMS_POLL_WINDOW 32U /* 每个调速窗口的相邻样本比较次数。 */
#define SSF_MEMS_POLL_PROBE_MS 30000U /* 定期试探更快轮询的间隔，ms。 */
#define SSF_MEMS_LINK_FAILURE_LIMIT 5U /* 判定断线所需的通信失败次数。 */
#define SSF_MEMS_LINK_FAILURE_MS 3000U /* 判定断线所需的无有效通信时长，ms。 */

enum ssf_mems_poll_mode {
  SSF_MEMS_POLL_FIXED, /* 使用用户指定的固定周期。 */
  SSF_MEMS_POLL_ADAPTIVE, /* 根据重复率在上下限内调整周期。 */
  SSF_MEMS_POLL_MODE_COUNT,
};

/* 主机特征轮询配置，单位均为 ms，不改变传感器采样率。 */
struct ssf_mems_poll_config {
  enum ssf_mems_poll_mode mode;
  unsigned int fixed_interval_ms;
  unsigned int min_interval_ms;
  unsigned int max_interval_ms;
};

/* 只记录轮询策略；采集锁保护统计，配置和生效周期另由短状态锁保护读写。 */
struct ssf_mems_acquisition_policy {
  const struct ssf_mems_sensor *sensor;
  struct ssf_mems_poll_config config;
  struct ssf_mems_sensor_data previous; /* 上一份完整特征，用于比较重复值。 */
  bool have_previous; /* previous 是否有效。 */
  unsigned int interval_ms; /* 当前轮询开始时间间隔，ms。 */
  unsigned int comparisons; /* 本窗口已完成的样本比较次数。 */
  unsigned int repeats; /* 本窗口完全相同的样本次数。 */
  unsigned int max_read_ms; /* 本窗口最长完整读取耗时，ms。 */
  unsigned int link_failures; /* 最近有效通信后累计的通信错误数。 */
  unsigned long last_contact; /* 最近成功/合法异常应答的 jiffies。 */
  unsigned long next_probe; /* 下次允许试探提速的 jiffies。 */
};

/* 初始化轮询策略。
 * 返回：无。
 */
void ssf_mems_acquisition_policy_init(
    struct ssf_mems_acquisition_policy *policy,
    const struct ssf_mems_sensor *sensor, unsigned long now);
/* 清空历史统计并保留用户配置和当前周期。
 * 返回：无。
 */
void ssf_mems_acquisition_policy_restart(
    struct ssf_mems_acquisition_policy *policy, unsigned long now);
/* 更新轮询配置并使当前周期满足新的限制。
 * 返回：0 成功，-EINVAL 模式、范围或上下限无效。
 */
int ssf_mems_acquisition_policy_configure(
    struct ssf_mems_acquisition_policy *policy,
    const struct ssf_mems_poll_config *config, unsigned long now);
/* 记录一次有效通信，不清空自适应统计。
 * 返回：无。
 */
void ssf_mems_acquisition_policy_contact(
    struct ssf_mems_acquisition_policy *policy, unsigned long now);
/* 记录成功样本并调整轮询周期。
 * 返回：无。
 */
void ssf_mems_acquisition_policy_sample(
    struct ssf_mems_acquisition_policy *policy,
    const struct ssf_mems_sensor_data *sample, unsigned int read_ms,
    unsigned long now);
/* 记录通信错误并判断是否需要重连。
 * 返回：true 需要重连，false 不需要。
 * 收到合法从机异常仍视为链路可达。
 */
bool ssf_mems_acquisition_policy_error(
    struct ssf_mems_acquisition_policy *policy, int error, unsigned long now);
