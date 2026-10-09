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

/* 特征计算配置；FEATURE_ENABLE 是 40070 高六位的归一化掩码。 */
enum ssf_mems_sensor_setting {
  SSF_MEMS_SETTING_SAMPLING_RATE,
  SSF_MEMS_SETTING_SAMPLING_LENGTH,
  SSF_MEMS_SETTING_PARAMETER_SWITCH,
  SSF_MEMS_SETTING_FEATURE_ENABLE,
  SSF_MEMS_SETTING_FIRMWARE_VERSION,
  SSF_MEMS_SETTING_MAX,
};

/* MEMS2.0 的 40052 索引 9 对应最高配置采样率 26667 Hz。 */
#define SSF_MEMS_SAMPLING_RATE_MAX_INDEX 9U

int ssf_mems_protocol_read_setting(struct serdev_device *serdev,
                                  enum ssf_mems_sensor_setting setting,
                                  u16 *value, unsigned int timeout_ms);
/* 仅允许采样率索引 0..9 和特征掩码 0..63；不改报警位，不发送保存/重启。
 * 固件自行将采样率变化保存至 Flash。相同值不重复写，写后读回验证。 */
int ssf_mems_protocol_write_setting(struct serdev_device *serdev,
                                   enum ssf_mems_sensor_setting setting,
                                   u16 value, unsigned int timeout_ms);

/* 保存应答后禁止发送的重启等待时间；按实机最长启动时间调整。 */
#define SSF_MEMS_SENSOR_REBOOT_DELAY_MS 1000U
/* 保存已确认后，新速率下仅重试读取验证，不重发 Flash 保存命令。
 * 两次请求仍受请求层的超时隔离约束；仅 -ETIMEDOUT 允许重试。 */
#define SSF_MEMS_BAUDRATE_VERIFY_ATTEMPTS 8U
#define SSF_MEMS_BAUDRATE_VERIFY_RETRY_MS 100U

/* 固件先结束特征计算再切速；首包预算不计入有效采集时长。 */
#define SSF_MEMS_RAW_START_TIMEOUT_MS 15000U
#define SSF_MEMS_RAW_STOP_ATTEMPTS 6U /* 每轮恢复最多发送六次停止命令。 */
#define SSF_MEMS_RAW_QUIET_MS 300U /* 覆盖最低采样率约 120 ms 的包间隔。 */
#define SSF_MEMS_RAW_STOP_GAP_WAIT_MS 300U /* RS485 停止前等待下一完整包的上限。 */

/* 总线业务模式与接收格式分开：恢复未确认时不能发普通业务请求。 */
enum ssf_mems_link_mode {
  SSF_MEMS_LINK_MODBUS, /* 已确认普通 Modbus 通信，可以读写配置。 */
  SSF_MEMS_LINK_RAW_STARTING, /* 启动已发送，固件可能仍在完成当前计算。 */
  SSF_MEMS_LINK_RAW_STREAMING, /* 已收到通过 CRC 校验的原始包。 */
  SSF_MEMS_LINK_RAW_RECOVERING, /* 已停止发布，尚未确认恢复普通通信。 */
};

/* 传感器协议业务状态；lock 保护缓存，bus_lock 串行化所有请求和串口配置。 */
struct ssf_mems_protocol_state {
  struct mutex lock;                    // 保护 features 和 valid
  struct ssf_mems_sensor_data features; // 最近一次完整读取的特征缓存
  bool valid;                           // 是否已有有效缓存

  struct mutex bus_lock;                // 公共 Modbus/serdev 总线锁
  enum ssf_mems_baudrate baudrate;      // 普通通信速率；不随私有流临时切到 1 Mbaud
  enum ssf_mems_parity parity;          // 当前主机校验位，数值与 40103 一致
  unsigned int host_baudrate;           // 控制器实际波特率，用于计算发送超时
  enum ssf_mems_link_mode mode; /* bus_lock 保护的业务模式，不随解析格式切换。 */
  unsigned long raw_start_until; /* 启动可能尚未生效的期限，防止过早确认停止。 */
  bool raw_baudrate_unknown; /* 加载时发现遗留私有流，尚不知道退出后的普通速率。 */
};

/* 初始化协议状态和手册默认参数，再用启动属性覆盖；缺失或读取失败时
 * 记录日志并保留默认值；不支持的 current-speed 同样回退到 9600，
 * 从站地址或校验位取值非法时仍返回 -EINVAL。
 * data/serdev 必须有效；仅用于尚未投入使用的状态，不发送 Modbus。 */
int ssf_mems_protocol_init(struct ssf_mems_xyzs_data *data);

/* 在 serdev 打开后、业务接口注册前配置主机 UART；使用 bus_lock。
 * 更新 host_baudrate 为控制器实际速率；启动非默认速率被控制器拒绝时
 * 回退到 9600。不写 40102/40103 或保存寄存器。 */
int ssf_mems_protocol_configure_serial(struct ssf_mems_xyzs_data *data);

/* 独占总线采集一段连续 XYZ：40058 启动，临时 1 Mbaud，40059 停止，
 * 恢复配置速率并验证。调用者持采集锁；退出/停止时仍执行停止和串口恢复。
 * duration_ms 从收到首包开始计时；奇偶校验或采样索引 >8 时不支持。 */
int ssf_mems_protocol_collect_raw(struct ssf_mems_xyzs_data *data,
                                 unsigned int duration_ms);
/* 仅在恢复未确认时重试停止；用于后台已退避时的卸载清理。
 * 调用前停止采集线程，接收状态仍须存活；普通模式下不发送任何帧。 */
int ssf_mems_protocol_recover_raw(struct ssf_mems_xyzs_data *data);

/* 同步处理完整帧；返回 0 已认领或已丢弃，-EINVAL
 * 空上下文/帧；未认领帧不额外解析、不缓存，buf 仅回调期间有效。 */
int ssf_mems_protocol_handle_frame(void *context, const u8 *buf, size_t len);

/* 复制特征缓存；返回 0 成功，-EINVAL 设备/输出指针为空，-ENODEV
 * 未绑定驱动数据，-ENODATA 缓存尚无有效数据。 */
int ssf_mems_protocol_get_features(struct serdev_device *serdev,
                                   struct ssf_mems_sensor_data *result);

/* pr_info 输出最新一批 XYZ 振动特征（g、mm/s）及声音（dB）。
 * features 在调用期间必须保持不变；NULL 无输出。仅用于调试。 */
void ssf_mems_protocol_log_features(
    const struct ssf_mems_sensor_data *features);

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

/* 旧速率写 40102，再写 40110=1；分别确认应答后等待重启，切换主机
 * 速率并读取 40102 验证。整个流程持 bus_lock，重启期间不发新请求。
 * 返回 0 或负 errno；保存后失败不盲目回退主机速率或重写 Flash。 */
int ssf_mems_protocol_set_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate baudrate,
                                   unsigned int timeout_ms);

/*
 * 用 40102 读应答确认实际通信速率；寄存器暂存值不等于已生效速率。
 * 当前 serdev 速率超时时，按枚举切换
 * serdev 并在每个速率只尝试一次；成功时保留匹配速率，全部失败时
 * 恢复入口速率。返回 0，或请求、数据校验及 serdev 配置的负 errno。
 */
int ssf_mems_protocol_get_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate *baudrate,
                                   unsigned int timeout_ms);
