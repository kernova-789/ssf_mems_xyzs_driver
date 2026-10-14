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
#define SSF_MEMS_POLL_WINDOW 32U /* 每个调速窗口的相邻样本比较次数。 */
#define SSF_MEMS_POLL_PROBE_MS 30000U /* 定期试探更快轮询的间隔，ms。 */
#define SSF_MEMS_LINK_FAILURE_LIMIT 5U /* 判定断线所需的通信失败次数。 */
#define SSF_MEMS_LINK_FAILURE_MS 3000U /* 判定断线所需的无有效通信时长，ms。 */

/* 只记录轮询策略，不直接操作 UART；由采集锁保护。 */
struct ssf_mems_acquisition_policy {
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

/* 重置策略并设置初始周期；无返回值，policy 必须有效。 */
void ssf_mems_acquisition_policy_init(
    struct ssf_mems_acquisition_policy *policy, unsigned long now);
/* 记录一次成功样本并按窗口调整周期；无返回值，指针必须有效。 */
void ssf_mems_acquisition_policy_sample(
    struct ssf_mems_acquisition_policy *policy,
    const struct ssf_mems_sensor_data *sample, unsigned int read_ms,
    unsigned long now);
/* 记录错误；返回 true 需重新连接，false 尚未达到次数/时间阈值或非通信错误。
 * -EREMOTEIO 说明收到合法从机异常，清除断线计数。 */
bool ssf_mems_acquisition_policy_error(
    struct ssf_mems_acquisition_policy *policy, int error, unsigned long now);
