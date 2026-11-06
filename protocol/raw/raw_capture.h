/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once
#include <linux/types.h>

struct ssf_mems_data;

/* 收尾原始批次并确认普通通信恢复。
 * 返回：0 已恢复，负数为静默、表项或读回错误。
 * 调用者持 bus_lock；确认失败时禁止开始新业务。
 */
int ssf_mems_raw_finish_locked(struct ssf_mems_data *data);

/* 单批事务；length 为固件 U32 编码，0 兼容 1024 点。
 * reset_stats 只重置观测统计，不负责时间窗口或重复调度。
 */
int ssf_mems_raw_collect_batch(struct ssf_mems_data *data,
                                u32 length, bool reset_stats);
