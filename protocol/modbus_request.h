#pragma once

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "modbus_types.h"

struct serdev_device;
struct ssf_mems_xyzs_data;

/* 请求接口的共用返回值（具体接口只会返回实际路径上的部分错误）：
 * 0 成功；-EINVAL 参数/数量/地址布局非法；-ENOENT 必要表项缺失；
 * -EACCES 无读写权限；-EOPNOTSUPP 功能码/格式不支持；-EAGAIN 总线模式不允许；
 * -ENODEV 无设备或关闭中；-EBUSY 当前请求未释放；-ENOMEM 分配失败；
 * -EIO 串口未完整发送；-ENOSPC 组帧缓冲区不足；-EMSGSIZE 帧超限；
 * -EREMOTEIO 传感器返回合法 Modbus 异常；-EFAULT 输出缓冲区异常；
 * -ETIMEDOUT 等待应答/总线静默超时；-ERESTARTSYS 等待被信号打断。
 * 串口控制器的其他负 errno 原样传回；CRC 错帧和不匹配帧会丢弃，
 * 无正确应答时最终表现为 -ETIMEDOUT，不直接向等待者返回候选帧 CRC 错误。 */

/* 保存当前正在执行的 Modbus 请求状态。 */
struct ssf_mems_modbus_request_state {
  struct mutex lock;       // 保护请求状态
  wait_queue_head_t waitq; // 等待响应的等待队列
  bool busy;               // 保持占用直到等待者取走最终结果
  bool pending;            // 是否有请求正在等待
  bool shutting_down;      // 驱动是否正在关闭
  unsigned long quarantine_until; // 超时/部分发送后，在此时间前不发送新请求
  u8 slave_id;             // 当前请求的从机地址
  struct ssf_modbus_transfer transfer; // 保存当前实际请求的帧/块/地址描述
  u16 *values;             // 当前请求的结果缓冲区
  size_t values_count;     // 结果缓冲区容量
  u16 write_value;         // 当前 0x06 请求写入的值
  int status; // 当前请求的执行结果0 = 成功，负数 = 失败，负数对应不同 errno
};

#define SSF_MEMS_MODBUS_DEFAULT_TIMEOUT_MS 1000U /* 调用者传 0 时的应答等待，ms。 */
#define SSF_MEMS_MODBUS_MIN_WRITE_TIMEOUT_MS 100U /* UART 写入最短超时，ms。 */
#define SSF_MEMS_MODBUS_WRITE_MARGIN_MS 100U /* 在线传输预算之外的发送余量，ms。 */
/* 无事务编号的 RTU 只能在已知最大响应延迟内隔离旧应答，实测后可调整。 */
#define SSF_MEMS_MODBUS_RECOVERY_GUARD_MS 1000U
#define SSF_MEMS_MODBUS_IDLE_WAIT_MS 1000U /* 隔离后最多再等待总线静默的时间，ms。 */

/* 返回 0 请求描述已生成，-EINVAL 参数/范围错误，-ENOENT 地址未入表，-EACCES 权限不足，-EOPNOTSUPP 格式不支持。 */
int ssf_mems_modbus_plan_request(u8 function, u16 display_reg, u16 reg_count,
                                const struct ssf_block_cmd_desc *block,
                                struct ssf_modbus_transfer *transfer);

/* 初始化请求状态及其锁和等待队列；返回 0 成功，-EINVAL 驱动数据为空；仅用于尚未投入使用的状态。 */
int ssf_mems_modbus_request_init(struct ssf_mems_xyzs_data *data);
/* 清理请求状态并唤醒等待者；无返回值，等待中的请求以 -ENODEV 结束，空指针直接退出。 */
void ssf_mems_modbus_request_remove(struct ssf_mems_xyzs_data *data);
/* 连续读取 1～125 个普通寄存器，校验失败不发送；返回 0 成功，-EINVAL 参数/数量/地址范围错误，-ENOENT 地址未入表，-EACCES 不可读，-EOPNOTSUPP 特殊响应，-ENODEV 无设备/关闭中，-EBUSY 请求占用，-ENOMEM 分配失败，-EIO 发送不足，-EREMOTEIO 从机异常，-EFAULT 结果缓冲区异常，-ETIMEDOUT 超时，-ERESTARTSYS 信号打断；其他负值来自组帧或串口。 */
int ssf_mems_modbus_read(struct serdev_device *serdev, u16 display_reg,
                         u16 reg_count, u16 *values, size_t values_count,
                         unsigned int timeout_ms);
/* 读取块命令中的实际连续段；返回 0 成功，-EINVAL 参数/范围错误，-ENOENT 地址缺失，-EACCES 不可读，-EOPNOTSUPP 格式不支持；其他负值同通用读取。 */
int ssf_mems_modbus_read_block_range(struct serdev_device *serdev,
                                     const struct ssf_block_cmd_desc *block,
                                     u16 display_reg, u16 reg_count,
                                     u16 *values, size_t values_count,
                                     unsigned int timeout_ms);
/* 调用者已持 bus_lock 的块读取；返回值及含义同 read_block_range()。 */
int ssf_mems_modbus_read_block_range_locked(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    u16 display_reg, u16 reg_count, u16 *values, size_t values_count,
    unsigned int timeout_ms);
/* 读取单个寄存器；返回 0 成功，失败原因和负值同 ssf_mems_modbus_read()。 */
int ssf_mems_modbus_read_reg(struct serdev_device *serdev, u16 display_reg,
                             u16 *value, unsigned int timeout_ms);
/* 调用者已持 bus_lock 的单寄存器读取；返回值及含义同 read_reg()。 */
int ssf_mems_modbus_read_reg_locked(struct serdev_device *serdev,
                                    u16 display_reg, u16 *value,
                                    unsigned int timeout_ms);
/* 仅供协议层持 bus_lock 做只读恢复确认；允许恢复模式及请求关闭后的读取。
 * 不重新开放普通请求，接收模块必须仍存活；用于收流退出和卸载清理。
 * 返回 0 成功；其余错误同普通读取，OTA 模式仍返回 -EAGAIN。 */
int ssf_mems_modbus_read_reg_recovery_locked(struct serdev_device *serdev,
                                             u16 display_reg, u16 *value,
                                             unsigned int timeout_ms);
/* 以 0x06 写入 write_single 为真的可写寄存器；返回 0 成功，-EOPNOTSUPP 表项不支持 0x06；其余校验、发送、响应和等待错误同 ssf_mems_modbus_write_regs()。 */
int ssf_mems_modbus_write_reg(struct serdev_device *serdev, u16 display_reg,
                              u16 value, unsigned int timeout_ms);
/* 调用者已持 bus_lock 的单寄存器写入；返回值及含义同 write_reg()。 */
int ssf_mems_modbus_write_reg_locked(struct serdev_device *serdev,
                                     u16 display_reg, u16 value,
                                     unsigned int timeout_ms);
/* 按表项 name 读取一个原始寄存器值；空参数 -EINVAL，名称缺失 -ENOENT，
 * 其余返回值同 read_reg；不做单位换算，不触发高层配置流程。 */
int ssf_mems_modbus_read_reg_by_name(struct serdev_device *serdev,
                                    const char *name, u16 *value,
                                    unsigned int timeout_ms);
/* 已持 bus_lock 的按名读取；返回值及含义同 read_reg_by_name()。 */
int ssf_mems_modbus_read_reg_by_name_locked(struct serdev_device *serdev,
                                           const char *name, u16 *value,
                                           unsigned int timeout_ms);
/* 按 name 执行一次 0x06 写入，权限和功能码仍按表校验；写 baudrate 不等于
 * 完成波特率切换，保存/重启流程仍须调用 protocol 的业务接口。
 * 返回 0 成功，-EINVAL 名称/设备为空，-ENOENT 名称缺失；其余错误同 write_reg()。 */
int ssf_mems_modbus_write_reg_by_name(struct serdev_device *serdev,
                                     const char *name, u16 value,
                                     unsigned int timeout_ms);
/* 已持 bus_lock 的按名写入；返回值及含义同 write_reg_by_name()。 */
int ssf_mems_modbus_write_reg_by_name_locked(struct serdev_device *serdev,
                                            const char *name, u16 value,
                                            unsigned int timeout_ms);
/* 连续写入 1～123 个寄存器，校验失败不发送；返回 0 成功，-EINVAL 参数/数量/地址范围错误，-ENOENT 地址未入表，-EACCES 不可写，-EOPNOTSUPP 帧格式不支持，-ENODEV 无设备/关闭中，-EBUSY 请求占用，-ENOMEM 分配失败，-EIO 发送不足，-EREMOTEIO 从机异常，-ETIMEDOUT 超时，-ERESTARTSYS 信号打断；其他负值来自组帧或串口。 */
/* 此接口检查权限，设备是否接受任意子范围的 0x10 仍需实测；返回值含义见上方。 */
int ssf_mems_modbus_write_regs(struct serdev_device *serdev, u16 display_reg,
                               u16 reg_count, const u16 *values,
                               size_t values_count, unsigned int timeout_ms);
/* 已持 bus_lock 的连续写入；返回值及含义同 write_regs()，仅配置指定范围。 */
int ssf_mems_modbus_write_regs_locked(
    struct serdev_device *serdev, u16 display_reg, u16 reg_count,
    const u16 *values, size_t values_count, unsigned int timeout_ms);
/* 按块描述执行写入，保留功能码/响应格式/方向和固定范围校验；返回 0 成功，-EINVAL 参数错误，-EOPNOTSUPP 不是可写块或格式不支持；其余负值同 ssf_mems_modbus_write_regs()。 */
int ssf_mems_modbus_write_block(struct serdev_device *serdev,
                                const struct ssf_block_cmd_desc *block,
                                const u16 *values, size_t values_count,
                                unsigned int timeout_ms);
/* 已持 bus_lock 的块写入；返回值及含义同 write_block()。 */
int ssf_mems_modbus_write_block_locked(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    const u16 *values, size_t values_count, unsigned int timeout_ms);
/* 返回 true 帧已消费（成功、从机异常或 -EFAULT），false 无等待请求、响应不匹配或帧损坏；true 不代表请求成功，不持有 buf。 */
bool ssf_mems_modbus_claim_frame(struct ssf_mems_xyzs_data *data,
                                 const u8 *buf, size_t len);
