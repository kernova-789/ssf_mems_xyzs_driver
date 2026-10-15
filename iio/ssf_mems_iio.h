/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/types.h>

struct ssf_mems_sensor_data;
struct ssf_mems_raw_sample;
struct ssf_mems_xyzs_data;

/* 注册特征和原始数据两个 IIO 设备。
 * 返回：0 全部成功，负数为初始化错误。
 * 任一设备注册失败时撤销已注册设备。
 */
int ssf_mems_iio_register(struct ssf_mems_xyzs_data *data);

/* 注销两个 IIO 设备并释放缓冲区。
 * 返回：无。
 * 调用前须停止所有数据生产者。
 */
void ssf_mems_iio_unregister(struct ssf_mems_xyzs_data *data);

/* 把一份特征推入 IIO 缓冲区。
 * 返回：0 成功或未启用缓冲区，负数为参数、设备或推送错误。
 * 协议层维护缓存；这里只持 mlock 打包和推送。
 */
int ssf_mems_iio_publish_features(
    struct ssf_mems_xyzs_data *data,
    const struct ssf_mems_sensor_data *features, s64 timestamp_ns);

/* 把一组 XYZ 原始样本推入 IIO 缓冲区。
 * 返回：0 成功或未启用缓冲区，负数为参数、设备或推送错误。
 * 注销 IIO 前须停止生产者；持 mlock 打包和推送。
 */
int ssf_mems_iio_publish_raw(struct ssf_mems_xyzs_data *data,
                            const struct ssf_mems_raw_sample *sample);
