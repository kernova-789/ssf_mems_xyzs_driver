#pragma once

#include <linux/mutex.h>
#include <linux/types.h>

#include "model/sensor_data.h"
#include "model/modbus_table.h"

/* 与手册 40103 寄存器的编码一致。 */
enum ssf_mems_parity {
  SSF_MEMS_PARITY_NONE = 0, /* 8N1，无奇偶校验。 */
  SSF_MEMS_PARITY_ODD = 1, /* 8O1，奇校验。 */
  SSF_MEMS_PARITY_EVEN = 2, /* 8E1，偶校验。 */
  SSF_MEMS_PARITY_MAX, /* 编码数量，不是合法校验格式。 */
};

struct serdev_device;
struct ssf_mems_xyzs_data;

/* 特征计算配置；地址和使能位布局由型号表解释。 */
enum ssf_mems_sensor_setting {
  SSF_MEMS_SETTING_SAMPLING_RATE, /* 40052，配置采样率索引。 */
  SSF_MEMS_SETTING_SAMPLING_LENGTH, /* 40053，采样长度索引。 */
  SSF_MEMS_SETTING_PARAMETER_SWITCH, /* 40070，完整开关字。 */
  SSF_MEMS_SETTING_FEATURE_ENABLE, /* 表中使能位归一化后的掩码，当前 0..63。 */
  SSF_MEMS_SETTING_FIRMWARE_VERSION, /* 40121，固件版本编码。 */
  SSF_MEMS_SETTING_MAX, /* 配置项数量，不是寄存器地址。 */
};

/* 读取一个传感器配置项。
 * 返回：0 成功，负数为参数、缺失表项或通信错误。
 */
int ssf_mems_protocol_read_setting(struct serdev_device *serdev,
                                  enum ssf_mems_sensor_setting setting,
                                  u16 *value, unsigned int timeout_ms);
/* 修改采样率或特征使能配置。
 * 返回：0 成功或无需修改，负数为参数、通信或读回错误。
 * 同值不重复写入，写后读回并保留原有报警位。
 */
int ssf_mems_protocol_write_setting(struct serdev_device *serdev,
                                   enum ssf_mems_sensor_setting setting,
                                   u16 value, unsigned int timeout_ms);

/* 保存应答后禁止发送的重启等待时间；按实机最长启动时间调整。 */
#define SSF_MEMS_SENSOR_REBOOT_DELAY_MS 8000U
/* 保存已确认后，新速率下仅重试读取验证，不重发 Flash 保存命令。
 * 两次请求仍受请求层的超时隔离约束；仅 -ETIMEDOUT 允许重试。 */
#define SSF_MEMS_BAUDRATE_VERIFY_ATTEMPTS 8U
#define SSF_MEMS_BAUDRATE_VERIFY_RETRY_MS 100U

/* 首包等待预算，允许启动后延迟输出数据。 */
#define SSF_MEMS_RAW_START_TIMEOUT_MS 15000U
#define SSF_MEMS_RAW_QUIET_MS 300U /* 覆盖最低采样率约 120 ms 的包间隔。 */

/* 总线业务模式与接收格式分开：恢复未确认时不能发普通业务请求。 */
enum ssf_mems_link_mode {
  SSF_MEMS_LINK_MODBUS, /* 已确认普通 Modbus 通信，可以读写配置。 */
  SSF_MEMS_LINK_RAW_STARTING, /* 启动已发送，正在等待首个原始包。 */
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

/* 初始化协议状态并读取启动配置。
 * 返回：0 成功，-EINVAL 设备或启动配置无效。
 * 只初始化状态，不发送串口命令。
 */
int ssf_mems_protocol_init(struct ssf_mems_xyzs_data *data);

/* 按启动配置设置主机 UART。
 * 返回：0 成功，负数为参数或控制器错误。
 * 非默认启动速率被拒绝时回退到 9600。
 */
int ssf_mems_protocol_configure_serial(struct ssf_mems_xyzs_data *data);

/* 在指定窗口内逐批采集 1024 组 XYZ。
 * 返回：0 完成，负数为配置、通信、数据完整性或 IIO 错误。
 * 调用者持采集锁；全程独占总线，窗口到期仍收完在途批次。
 */
int ssf_mems_protocol_collect_raw(struct ssf_mems_xyzs_data *data,
                                 unsigned int duration_ms);
/* 恢复尚未确认结束的原始批次。
 * 返回：0 已恢复或无需恢复，负数为参数或收尾错误。
 * 接收模块须仍可用，只读确认，不写 Flash。
 */
int ssf_mems_protocol_recover_raw(struct ssf_mems_xyzs_data *data);

/* 把完整 Modbus 帧交给请求层匹配。
 * 返回：0 已处理或丢弃，-EINVAL 参数为空。
 * 帧数据只在本次调用期间有效。
 */
int ssf_mems_protocol_handle_frame(void *context, const u8 *buf, size_t len);

/* 复制最新的有效特征缓存。
 * 返回：0 成功，-ENODATA 无有效缓存，其他负数为参数或设备错误。
 */
int ssf_mems_protocol_get_features(struct serdev_device *serdev,
                                   struct ssf_mems_sensor_data *result);

/* 打印一份完整特征用于调试。
 * 返回：无。
 * 用定点数输出，避免内核浮点运算。
 */
void ssf_mems_protocol_log_features(
    const struct ssf_mems_sensor_data *features);

/* 替换最新的特征缓存。
 * 返回：0 成功，-EINVAL 参数为空。
 * 加锁更新，避免读到部分新数据。
 */
int ssf_mems_protocol_store_features(
    struct ssf_mems_xyzs_data *data,
    const struct ssf_mems_sensor_data *features);

/* 把特征缓存标记为无效。
 * 返回：无。
 */
void ssf_mems_protocol_invalidate_features(struct ssf_mems_xyzs_data *data);

/* 把完整特征块转换为特征结构体。
 * 返回：0 成功，-EINVAL 参数无效，-ENOENT 块描述缺失，-ERANGE 换算越界。
 * 删除的表项对应字段置零。
 */
int ssf_mems_protocol_decode_features(const u16 *registers, size_t count,
                                      struct ssf_mems_sensor_data *result);

/* 分段读取全部特征并更新缓存。
 * 返回：0 成功，负数为参数、表项、通信或解码错误。
 * 所有分段成功后才更新缓存。
 */
int ssf_mems_protocol_read_features(struct serdev_device *serdev,
                                    unsigned int timeout_ms);

/* 写入块表规定的十个工作参数。
 * 返回：0 成功，负数为参数、表项或通信错误。
 */
int ssf_mems_protocol_write_work_parameters(struct serdev_device *serdev,
                                            const u16 *values,
                                            size_t values_count,
                                            unsigned int timeout_ms);

/* 把波特率编码转换为 bit/s 数值。
 * 返回：正数为波特率，-EINVAL 编码无效。
 */
int ssf_mems_baudrate_to_value(enum ssf_mems_baudrate baudrate);

/* 把 bit/s 数值转换为波特率编码。
 * 返回：0 成功，-EINVAL 参数或速率无效。
 * 9600 对应显式编码 3。
 */
int ssf_mems_baudrate_from_value(unsigned int value,
                                 enum ssf_mems_baudrate *baudrate);

/* 修改并验证传感器通信波特率。
 * 返回：0 新速率已确认，负数为参数、通信或验证错误。
 * 旧速率下暂存并保存重启，再切换主机验证；不重发保存。
 */
int ssf_mems_protocol_set_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate baudrate,
                                   unsigned int timeout_ms);

/* 确认传感器当前可通信的波特率。
 * 返回：0 匹配成功，负数为参数、通信或编码错误。
 * 当前速率超时才扫描其他速率，失败时尝试恢复入口速率。
 */
int ssf_mems_protocol_get_baudrate(struct serdev_device *serdev,
                                   enum ssf_mems_baudrate *baudrate,
                                   unsigned int timeout_ms);
