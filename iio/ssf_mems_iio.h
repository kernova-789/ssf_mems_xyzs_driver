/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/types.h>

struct ssf_mems_sensor_data;
struct ssf_mems_xyzs_data;

/* 创建并注册 IIO 设备；返回 0 或负 errno。 */
int ssf_mems_iio_register(struct ssf_mems_xyzs_data *data);

/* 先从 IIO 子系统注销再释放设备；未注册时直接返回。 */
void ssf_mems_iio_unregister(struct ssf_mems_xyzs_data *data);

/*
 * 发布一份完整特征快照：先替换 read_raw() 使用的缓存，再在
 * buffer 已启用时推送同一份数据。timestamp_ns 为 0 时使用 IIO 当前时间。
 * 调用者必须在注销 IIO 设备前停止数据生产者。
 */
int ssf_mems_iio_publish_features(
    struct ssf_mems_xyzs_data *data,
    const struct ssf_mems_sensor_data *features, s64 timestamp_ns);
