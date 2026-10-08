#pragma once

#include <linux/mutex.h>
#include <linux/types.h>

#include "sensor_data.h"

enum ssf_mems_baudrate {
  SSF_MEMS_BAUDRATE_DEFAULT = 0,
  SSF_MEMS_BAUDRATE_2400 = 1,
  SSF_MEMS_BAUDRATE_4800 = 2,
  SSF_MEMS_BAUDRATE_9600 = 3,
  SSF_MEMS_BAUDRATE_19200 = 4,
  SSF_MEMS_BAUDRATE_38400 = 5,
  SSF_MEMS_BAUDRATE_57600 = 6,
  SSF_MEMS_BAUDRATE_115200 = 7,
  SSF_MEMS_BAUDRATE_128000 = 8,
  SSF_MEMS_BAUDRATE_230400 = 9,
  SSF_MEMS_BAUDRATE_256000 = 10,
  SSF_MEMS_BAUDRATE_460800 = 11,
  SSF_MEMS_BAUDRATE_500000 = 12,
  SSF_MEMS_BAUDRATE_512000 = 13,
  SSF_MEMS_BAUDRATE_600000 = 14,
  SSF_MEMS_BAUDRATE_750000 = 15,
  SSF_MEMS_BAUDRATE_921600 = 16,
  SSF_MEMS_BAUDRATE_1000000 = 17,
  SSF_MEMS_BAUDRATE_MAX,
};

/* 与固件 40103 寄存器的数值一致。 */
enum ssf_mems_parity {
  SSF_MEMS_PARITY_NONE = 0,
  SSF_MEMS_PARITY_ODD = 1,
  SSF_MEMS_PARITY_EVEN = 2,
  SSF_MEMS_PARITY_MAX,
};

struct serdev_device;
struct ssf_mems_xyzs_data;

/* 传感器协议业务状态；lock 保护缓存，bus_lock 串行化所有请求和串口配置。 */
struct ssf_mems_protocol_state {
  struct mutex lock;                    // 保护 features 和 valid
  struct ssf_mems_sensor_data features; // 最近一次完整读取的特征缓存
  bool valid;                           // 是否已有有效缓存

  struct mutex bus_lock;                // 公共 Modbus/serdev 总线锁
  enum ssf_mems_baudrate baudrate;      // 与传感器同步的波特率寄存器值
  enum ssf_mems_parity parity;          // 当前主机校验位，数值与 40103 一致
  unsigned int host_baudrate;           // 控制器实际波特率，用于计算发送超时
};

/* 初始化协议状态和手册默认参数，再用启动属性覆盖；缺失或读取失败时
 * 记录日志并保留默认值，可读取但取值非法时返回 -EINVAL。
 * data/serdev 必须有效；仅用于尚未投入使用的状态，不发送 Modbus。 */
int ssf_mems_protocol_init(struct ssf_mems_xyzs_data *data);

/* 在 serdev 打开后、业务接口注册前配置主机 UART；使用 bus_lock。
 * 更新 host_baudrate 为控制器实际速率，不写 40102/40103 或保存寄存器。 */
int ssf_mems_protocol_configure_serial(struct ssf_mems_xyzs_data *data);

/* 同步处理完整帧；返回 0 已认领或已丢弃，-EINVAL
 * 空上下文/帧；未认领帧不额外解析、不缓存，buf 仅回调期间有效。 */
int ssf_mems_protocol_handle_frame(void *context, const u8 *buf, size_t len);

/* 复制特征缓存；返回 0 成功，-EINVAL 设备/输出指针为空，-ENODEV
 * 未绑定驱动数据，-ENODATA 缓存尚无有效数据。 */
int ssf_mems_protocol_get_features(struct serdev_device *serdev,
                                   struct ssf_mems_sensor_data *result);

/* 原子替换最新特征快照；返回 0 或 -EINVAL。 */
int ssf_mems_protocol_store_features(
    struct ssf_mems_xyzs_data *data,
    const struct ssf_mems_sensor_data *features);

/* 断线或数据过期时使缓存失效；保留内容，下次完整读取后再次有效。 */
void ssf_mems_protocol_invalidate_features(struct ssf_mems_xyzs_data *data);

/* 按块表解码完整连续特征块；返回 0 成功，-ENOENT 块命令缺失，-EINVAL
 * 参数/数量错误；删除表项对应字段置零。 */
int ssf_mems_protocol_decode_features(const u16 *registers, size_t count,
                                      struct ssf_mems_sensor_data *result);

/* 按保留表项分段读取特征；返回 0 成功（无特征时缓存全零），-EINVAL
 * 参数/范围错误，-ENODEV 无驱动数据，-ENOENT 块命令缺失，-EOPNOTSUPP
 * 块格式不支持；其他负值来自读请求或解码。 */
int ssf_mems_protocol_read_features(struct serdev_device *serdev,
                                    unsigned int timeout_ms);

/* 按地址递增写入块表规定的 40061～40070，values_count 至少为 10；返回 0
 * 成功，-ENOENT 块/地址缺失；其他负值同 ssf_mems_modbus_write_block()。 */
int ssf_mems_protocol_write_work_parameters(struct serdev_device *serdev,
                                            const u16 *values,
                                            size_t values_count,
                                            unsigned int timeout_ms);

/* 将寄存器枚举转换为串口波特率；返回正整数，非法枚举返回 -EINVAL。 */
int ssf_mems_baudrate_to_value(enum ssf_mems_baudrate baudrate);

/* 将数值波特率转换为寄存器枚举；9600 选显式枚举，返回 0 或 -EINVAL。 */
int ssf_mems_baudrate_from_value(unsigned int value,
                                 enum ssf_mems_baudrate *baudrate);

/* 写 40102，收到正确应答后同步修改 serdev 波特率；返回 0 或负 errno。 */
int ssf_mems_protocol_set_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate baudrate,
                                   unsigned int timeout_ms);

/*
 * 读 40102 获取传感器波特率。当前 serdev 速率超时时，按枚举切换
 * serdev 并在每个速率只尝试一次；成功时保留匹配速率，全部失败时
 * 恢复入口速率。返回 0，或请求、数据校验及 serdev 配置的负 errno。
 */
int ssf_mems_protocol_get_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate *baudrate,
                                   unsigned int timeout_ms);
