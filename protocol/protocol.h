#pragma once

#include <linux/mutex.h>
#include <linux/types.h>

#include "sensor_data.h"

/* 40102 的波特率编码，枚举值不是每秒位数。 */
enum ssf_mems_baudrate {
  SSF_MEMS_BAUDRATE_DEFAULT = 0, /* 固件默认速率，实际为 9600 bit/s。 */
  SSF_MEMS_BAUDRATE_2400 = 1, /* 2400 bit/s。 */
  SSF_MEMS_BAUDRATE_4800 = 2, /* 4800 bit/s。 */
  SSF_MEMS_BAUDRATE_9600 = 3, /* 9600 bit/s，显式编码。 */
  SSF_MEMS_BAUDRATE_19200 = 4, /* 19200 bit/s。 */
  SSF_MEMS_BAUDRATE_38400 = 5, /* 38400 bit/s。 */
  SSF_MEMS_BAUDRATE_57600 = 6, /* 57600 bit/s。 */
  SSF_MEMS_BAUDRATE_115200 = 7, /* 115200 bit/s。 */
  SSF_MEMS_BAUDRATE_128000 = 8, /* 128000 bit/s，当前正式工作速率。 */
  SSF_MEMS_BAUDRATE_230400 = 9, /* 230400 bit/s。 */
  SSF_MEMS_BAUDRATE_256000 = 10, /* 256000 bit/s。 */
  SSF_MEMS_BAUDRATE_460800 = 11, /* 460800 bit/s。 */
  SSF_MEMS_BAUDRATE_500000 = 12, /* 500000 bit/s。 */
  SSF_MEMS_BAUDRATE_512000 = 13, /* 512000 bit/s。 */
  SSF_MEMS_BAUDRATE_600000 = 14, /* 600000 bit/s。 */
  SSF_MEMS_BAUDRATE_750000 = 15, /* 750000 bit/s。 */
  SSF_MEMS_BAUDRATE_921600 = 16, /* 921600 bit/s。 */
  SSF_MEMS_BAUDRATE_1000000 = 17, /* 1000000 bit/s，固件最高编码速率。 */
  SSF_MEMS_BAUDRATE_MAX, /* 编码数量/非法值边界，不能写入传感器。 */
};

/* 与固件 40103 寄存器的数值一致。 */
enum ssf_mems_parity {
  SSF_MEMS_PARITY_NONE = 0, /* 8N1，无奇偶校验。 */
  SSF_MEMS_PARITY_ODD = 1, /* 8O1，奇校验。 */
  SSF_MEMS_PARITY_EVEN = 2, /* 8E1，偶校验。 */
  SSF_MEMS_PARITY_MAX, /* 编码数量，不是合法校验格式。 */
};

struct serdev_device;
struct ssf_mems_xyzs_data;

/* 特征计算配置；FEATURE_ENABLE 是 40070 高六位的归一化掩码。 */
enum ssf_mems_sensor_setting {
  SSF_MEMS_SETTING_SAMPLING_RATE, /* 40052，配置采样率索引。 */
  SSF_MEMS_SETTING_SAMPLING_LENGTH, /* 40053，采样长度索引。 */
  SSF_MEMS_SETTING_PARAMETER_SWITCH, /* 40070，完整开关字。 */
  SSF_MEMS_SETTING_FEATURE_ENABLE, /* 40070 高六位，归一化为 0..63。 */
  SSF_MEMS_SETTING_FIRMWARE_VERSION, /* 40121，固件版本编码。 */
  SSF_MEMS_SETTING_MAX, /* 配置项数量，不是寄存器地址。 */
};

/* MEMS2.0 的 40052 索引 9 对应最高配置采样率 26667 Hz。 */
#define SSF_MEMS_SAMPLING_RATE_MAX_INDEX 9U
/* 当前定长原始采集已验证的上限；索引 9 合法，但该采集路径尚未验证。 */
#define SSF_MEMS_RAW_SAMPLING_RATE_MAX_INDEX 8U

/* 读取指定配置；返回 0 成功，-EINVAL 参数/配置项非法，-ENODEV 无驱动数据；
 * 其余错误及含义见 modbus_request.h 的请求返回值约定。 */
int ssf_mems_protocol_read_setting(struct serdev_device *serdev,
                                  enum ssf_mems_sensor_setting setting,
                                  u16 *value, unsigned int timeout_ms);
/* 仅允许采样率索引 0..9 和特征掩码 0..63；不改报警位，不发送保存/重启。
 * 固件自行将采样率变化保存至 Flash。相同值不重复写，写后读回验证。
 * 返回 0 成功/无需写入，-EINVAL 参数/范围非法，-ENODEV 无驱动数据，
 * -EIO 读回值不符；其余错误见 modbus_request.h 的请求返回值约定。 */
int ssf_mems_protocol_write_setting(struct serdev_device *serdev,
                                   enum ssf_mems_sensor_setting setting,
                                   u16 value, unsigned int timeout_ms);

/* 保存应答后禁止发送的重启等待时间；按实机最长启动时间调整。 */
#define SSF_MEMS_SENSOR_REBOOT_DELAY_MS 8000U
/* 保存已确认后，新速率下仅重试读取验证，不重发 Flash 保存命令。
 * 两次请求仍受请求层的超时隔离约束；仅 -ETIMEDOUT 允许重试。 */
#define SSF_MEMS_BAUDRATE_VERIFY_ATTEMPTS 8U
#define SSF_MEMS_BAUDRATE_VERIFY_RETRY_MS 100U

/* 固件先完成当前特征计算，再进行定长采集和上传。 */
#define SSF_MEMS_RAW_START_TIMEOUT_MS 15000U
#define SSF_MEMS_RAW_QUIET_MS 300U /* 覆盖最低采样率约 120 ms 的包间隔。 */

/* 总线业务模式与接收格式分开：恢复未确认时不能发普通业务请求。 */
enum ssf_mems_link_mode {
  SSF_MEMS_LINK_MODBUS, /* 已确认普通 Modbus 通信，可以读写配置。 */
  SSF_MEMS_LINK_RAW_STARTING, /* 启动已发送，固件可能仍在完成当前计算。 */
  SSF_MEMS_LINK_RAW_STREAMING, /* 已收到通过 CRC 校验的原始包。 */
  SSF_MEMS_LINK_RAW_RECOVERING, /* 已停止发布，尚未确认恢复普通通信。 */
  SSF_MEMS_LINK_OTA, /* bus_lock 内的固件传输；只解析私有 6 字节应答。 */
};

/* 传感器协议业务状态；lock 保护缓存，bus_lock 串行化所有请求和串口配置。 */
struct ssf_mems_protocol_state {
  struct mutex lock;                    // 保护 features 和 valid
  struct ssf_mems_sensor_data features; // 最近一次完整读取的特征缓存
  bool valid;                           // 是否已有有效缓存

  struct mutex bus_lock;                // 公共 Modbus/serdev 总线锁
  enum ssf_mems_baudrate baudrate;      // 当前通信速率；定长数据与 Modbus 共用
  enum ssf_mems_parity parity;          // 当前主机校验位，数值与 40103 一致
  unsigned int host_baudrate;           // UART 配置前为启动值，配置后为实际速率
  enum ssf_mems_link_mode mode; /* bus_lock 保护的业务模式，不随解析格式切换。 */
  unsigned long raw_start_until; /* 当前批次的最晚收尾时间，含启动和线上传输预算。 */
};

/* 初始化协议状态和手册默认参数，再用启动属性覆盖；缺失或读取失败时
 * 记录日志并保留默认值；不支持的 current-speed 同样回退到 9600，
 * 返回 0 初始化成功（含使用默认值），-EINVAL 设备为空或从站地址/校验位非法。
 * 仅用于尚未投入使用的状态，不发送 Modbus。 */
int ssf_mems_protocol_init(struct ssf_mems_xyzs_data *data);

/* 在 serdev 打开后、业务接口注册前配置主机 UART；使用 bus_lock。
 * 更新 host_baudrate 为控制器实际速率；启动非默认速率被控制器拒绝时
 * 回退到 9600。不写 40102/40103 或保存寄存器。
 * 返回 0 成功，-EINVAL 设备/枚举非法，-EIO 控制器无法设置速率；
 * 其他负 errno 来自 serdev 校验位设置（如 -EOPNOTSUPP 控制器不支持）。 */
int ssf_mems_protocol_configure_serial(struct ssf_mems_xyzs_data *data);

/* 独占总线，在 duration_ms 窗口内逐批读取 1024 组 XYZ；至少完成一批。
 * 40060 启动，保持工作波特率；窗口到期/卸载时等待在途批次自然结束。
 * 调用者持采集锁；当前支持无校验、采样索引 0..8。
 * 返回 0 完整收完窗口，-EINVAL 参数非法，-EOPNOTSUPP 校验位/采样率不支持，
 * -ENOENT 必要表项缺失，-ENODEV 停止中，-ETIMEDOUT 启动/收尾超时，
 * -EILSEQ 批次不完整/包损坏，-ENOSPC IIO 丢样，-EPROTO 收尾寄存器异常，
 * -EIO 短写/配置读回不符，-ERESTARTSYS 等待被信号打断；其余为请求层错误。 */
int ssf_mems_protocol_collect_raw(struct ssf_mems_xyzs_data *data,
                                 unsigned int duration_ms);
/* 仅在恢复未确认时等待定长上传结束并只读验证；不切速、不写 Flash。
 * 调用前停止采集线程，接收状态仍须存活；普通模式下不发送任何帧。
 * 返回 0 已确认/本来就是普通模式，-EINVAL 空设备，-ETIMEDOUT 无法确认静默，
 * -EPROTO 波特率寄存器异常；其他负值来自收尾的只读请求。 */
int ssf_mems_protocol_recover_raw(struct ssf_mems_xyzs_data *data);

/* 同步处理完整帧；返回 0 已认领或已丢弃，-EINVAL
 * 空上下文/帧；未认领帧不额外解析、不缓存，buf 仅回调期间有效。 */
int ssf_mems_protocol_handle_frame(void *context, const u8 *buf, size_t len);

/* 复制特征缓存；返回 0 成功，-EINVAL 设备/输出指针为空，-ENODEV
 * 未绑定驱动数据，-ENODATA 缓存尚无有效数据。 */
int ssf_mems_protocol_get_features(struct serdev_device *serdev,
                                   struct ssf_mems_sensor_data *result);

/* pr_info 输出最新一批 XYZ 振动特征（g、mm/s）及声音（dB）。
 * features 在调用期间必须保持不变；NULL 无输出。无返回值，仅用于调试。 */
void ssf_mems_protocol_log_features(
    const struct ssf_mems_sensor_data *features);

/* 原子替换最新特征快照；返回 0 成功，-EINVAL 设备或输入快照为空。 */
int ssf_mems_protocol_store_features(
    struct ssf_mems_xyzs_data *data,
    const struct ssf_mems_sensor_data *features);

/* 断线或过期时使缓存失效；无返回值，空设备直接退出，保留缓存内容。 */
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

/* 将数值波特率转换为寄存器枚举；9600 选显式枚举，返回 0 成功，
 * -EINVAL 输出指针为空或数值波特率不受支持。 */
int ssf_mems_baudrate_from_value(unsigned int value,
                                 enum ssf_mems_baudrate *baudrate);

/* 旧速率写 40102，再写 40110=1；分别确认应答后等待重启，切换主机
 * 速率并读取 40102 验证。整个流程持 bus_lock，重启期间不发新请求。
 * 返回 0 已确认新速率，-EINVAL 参数非法，-ENODEV 无设备/停止中，
 * -EPROTO 读回速率异常/与目标不符，-EIO 控制器拒绝配置；
 * 其余错误见 modbus_request.h 的请求返回值约定。保存后失败不重复写 Flash。 */
int ssf_mems_protocol_set_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate baudrate,
                                   unsigned int timeout_ms);

/*
 * 用 40102 读应答确认实际通信速率；寄存器暂存值不等于已生效速率。
 * 当前 serdev 速率超时时，按枚举切换
 * serdev 并在每个速率只尝试一次；成功时保留匹配速率，全部失败时
 * 恢复入口速率。返回 0 匹配成功，-EINVAL 参数非法，-ENODEV 无设备/停止中，
 * -EPROTO 寄存器值异常，-EIO 主机调速失败，
 * -ETIMEDOUT 所有候选均无应答；其余错误见 modbus_request.h 的请求返回值约定。
 */
int ssf_mems_protocol_get_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate *baudrate,
                                   unsigned int timeout_ms);
