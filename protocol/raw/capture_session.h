/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once
#include <linux/types.h>
struct ssf_mems_data;
/* 调用者持 bus_lock；有限批次结束后只读确认普通通信。 */
int ssf_mems_capture_finish_locked(struct ssf_mems_data *data);
bool ssf_mems_capture_wait_quiet(struct ssf_mems_data *data);
