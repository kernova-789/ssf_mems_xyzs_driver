/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/types.h>

struct ssf_mems_sensor_data;
struct ssf_mems_raw_sample;
struct ssf_mems_xyzs_data;

/* 创建特征/原始两个 IIO 设备；返回 0 全部成功，-EINVAL 空设备，-EBUSY 已注册，
 * -ENOMEM 对象/buffer 分配失败；其他负 errno 来自 IIO 注册（如 -EINVAL 配置非法）。 */
int ssf_mems_iio_register(struct ssf_mems_xyzs_data *data);

/* 注销 IIO 并释放 buffer；无返回值，空设备/未注册时直接返回，对象由 devm 回收。 */
void ssf_mems_iio_unregister(struct ssf_mems_xyzs_data *data);

/*
 * 发布一份完整特征快照：先替换 read_raw() 使用的缓存，再在
 * buffer 已启用时推送同一份数据。timestamp_ns 为 0 时使用 IIO 当前时间。
 * 调用者必须在注销 IIO 设备前停止数据生产者。
 * 返回 0 推送成功或 buffer 未启用，-EINVAL 空参数/扫描掩码无效，
 * -ENODEV 未注册特征 IIO，-EOVERFLOW 扫描布局超出容量；其他为 IIO 推送错误。
 * 当前 kfifo 满时返回 -EBUSY；其他 buffer 实现的负 errno 原样返回。
 */
int ssf_mems_iio_publish_features(
    struct ssf_mems_xyzs_data *data,
    const struct ssf_mems_sensor_data *features, s64 timestamp_ns);

/* 向独立原始 IIO buffer 发布 XYZ、包号及包内序号；时间戳为主机接收时间。
 * 未启用 buffer 时不推送；调用者须在注销 IIO 前停止接收生产者。
 * 返回 0 成功/未启用 buffer，-EINVAL 空参数/扫描布局非法，-ENODEV 未注册原始 IIO；
 * 其他为 IIO 推送错误（当前 kfifo 满返回 -EBUSY）。 */
int ssf_mems_iio_publish_raw(struct ssf_mems_xyzs_data *data,
                            const struct ssf_mems_raw_sample *sample);
