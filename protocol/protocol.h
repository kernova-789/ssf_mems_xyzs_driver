#pragma once

#include <linux/mutex.h>
#include <linux/types.h>

#include "sensor_data.h"

struct serdev_device;
struct ssf_mems_xyzs_data;

/* 传感器协议业务状态；锁只保护缓存，传感器结果保持为可复制的纯数据。 */
struct ssf_mems_protocol_state {
  struct mutex lock;                    // 保护 features 和 valid
  struct ssf_mems_sensor_data features;  // 最近一次完整读取的特征缓存
  bool valid;                           // 是否已有有效缓存
};

/* 初始化传感器协议缓存及其锁；返回 0 成功，-EINVAL 驱动数据为空；仅用于尚未投入使用的状态。 */
int ssf_mems_protocol_init(struct ssf_mems_xyzs_data *data);

/* 同步处理完整帧；返回 0 已认领或已丢弃，-EINVAL 空上下文/帧；未认领帧不额外解析、不缓存，buf 仅回调期间有效。 */
int ssf_mems_protocol_handle_frame(void *context, const u8 *buf, size_t len);

/* 按块表解码完整连续特征块；返回 0 成功，-ENOENT 块命令缺失，-EINVAL 参数/数量错误；删除表项对应字段置零。 */
int ssf_mems_protocol_decode_features(const u16 *registers, size_t count,
                                      struct ssf_mems_sensor_data *result);

/* 按保留表项分段读取特征；返回 0 成功（无特征时缓存全零），-EINVAL 参数/范围错误，-ENODEV 无驱动数据，-ENOENT 块命令缺失，-EOPNOTSUPP 块格式不支持；其他负值来自读请求或解码。 */
int ssf_mems_protocol_read_features(struct serdev_device *serdev,
                                    unsigned int timeout_ms);

/* 复制特征缓存；返回 0 成功，-EINVAL 设备/输出指针为空，-ENODEV 未绑定驱动数据，-ENODATA 缓存尚无有效数据。 */
int ssf_mems_protocol_get_features(struct serdev_device *serdev,
                                   struct ssf_mems_sensor_data *result);

/* 按地址递增写入块表规定的 40061～40070，values_count 至少为 10；返回 0 成功，-ENOENT 块/地址缺失；其他负值同 ssf_mems_modbus_write_block()。 */
int ssf_mems_protocol_write_work_parameters(struct serdev_device *serdev,
                                            const u16 *values,
                                            size_t values_count,
                                            unsigned int timeout_ms);
