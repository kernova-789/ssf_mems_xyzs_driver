/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

struct ssf_mems_xyzs_data;

/* 收尾原始批次并确认普通通信恢复。
 * 返回：0 已恢复，负数为静默、表项或读回错误。
 * 调用者持 bus_lock；确认失败时禁止开始新业务。
 */
int ssf_mems_raw_finish_locked(struct ssf_mems_xyzs_data *data);
