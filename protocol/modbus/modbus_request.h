#pragma once

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "modbus/modbus_types.h"

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

/* 查表校验请求并生成传输描述。
 * 返回：0 成功，负数为参数、缺失表项、权限或格式错误。
 */
int ssf_mems_modbus_plan_request(u8 function, u16 display_reg, u16 reg_count,
                                const struct ssf_block_cmd_desc *block,
                                struct ssf_modbus_transfer *transfer);

/* 初始化请求状态和等待队列。
 * 返回：0 成功，-EINVAL 设备无效。
 */
int ssf_mems_modbus_request_init(struct ssf_mems_xyzs_data *data);
/* 关闭请求模块并唤醒等待者。
 * 返回：无。
 * 在途请求以 -ENODEV 结束。
 */
void ssf_mems_modbus_request_remove(struct ssf_mems_xyzs_data *data);
/* 读取一段普通寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 私有数据响应不能通过此接口读取。
 */
int ssf_mems_modbus_read(struct serdev_device *serdev, u16 display_reg,
                         u16 reg_count, u16 *values, size_t values_count,
                         unsigned int timeout_ms);
/* 读取寄存器块中的一段数据。
 * 返回：0 成功，负数为校验或通信错误。
 */
int ssf_mems_modbus_read_block_range(struct serdev_device *serdev,
                                     const struct ssf_block_cmd_desc *block,
                                     u16 display_reg, u16 reg_count,
                                     u16 *values, size_t values_count,
                                     unsigned int timeout_ms);
/* 在已加锁的总线上读取块内数据。
 * 返回：0 成功，负数为校验或通信错误。
 * 调用者须持 bus_lock。
 */
int ssf_mems_modbus_read_block_range_locked(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    u16 display_reg, u16 reg_count, u16 *values, size_t values_count,
    unsigned int timeout_ms);
/* 读取一个普通寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 */
int ssf_mems_modbus_read_reg(struct serdev_device *serdev, u16 display_reg,
                             u16 *value, unsigned int timeout_ms);
/* 在已加锁的总线上读取一个寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 调用者须持 bus_lock。
 */
int ssf_mems_modbus_read_reg_locked(struct serdev_device *serdev,
                                    u16 display_reg, u16 *value,
                                    unsigned int timeout_ms);
/* 只读确认定长采集后的普通通信。
 * 返回：0 成功，负数为校验或通信错误。
 * 调用者须持 bus_lock，恢复和关闭期间也允许读取。
 */
int ssf_mems_modbus_read_reg_recovery_locked(struct serdev_device *serdev,
                                             u16 display_reg, u16 *value,
                                             unsigned int timeout_ms);
/* 用 0x06 写入一个寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 表项须允许单寄存器写入。
 */
int ssf_mems_modbus_write_reg(struct serdev_device *serdev, u16 display_reg,
                              u16 value, unsigned int timeout_ms);
/* 在已加锁的总线上写入一个寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 调用者须持 bus_lock。
 */
int ssf_mems_modbus_write_reg_locked(struct serdev_device *serdev,
                                     u16 display_reg, u16 value,
                                     unsigned int timeout_ms);
/* 按名称读取一个寄存器。
 * 返回：0 成功，负数为名称、校验或通信错误。
 */
int ssf_mems_modbus_read_reg_by_name(struct serdev_device *serdev,
                                    const char *name, u16 *value,
                                    unsigned int timeout_ms);
/* 在已加锁的总线上按名称读取寄存器。
 * 返回：0 成功，负数为名称、校验或通信错误。
 * 调用者须持 bus_lock。
 */
int ssf_mems_modbus_read_reg_by_name_locked(struct serdev_device *serdev,
                                           const char *name, u16 *value,
                                           unsigned int timeout_ms);
/* 按名称用 0x06 写入一个寄存器。
 * 返回：0 成功，负数为名称、校验或通信错误。
 * 仅写寄存器，不代替波特率保存和切换流程。
 */
int ssf_mems_modbus_write_reg_by_name(struct serdev_device *serdev,
                                     const char *name, u16 value,
                                     unsigned int timeout_ms);
/* 在已加锁的总线上按名称写入寄存器。
 * 返回：0 成功，负数为名称、校验或通信错误。
 * 调用者须持 bus_lock，不额外发送保存命令。
 */
int ssf_mems_modbus_write_reg_by_name_locked(struct serdev_device *serdev,
                                            const char *name, u16 value,
                                            unsigned int timeout_ms);
/* 用 0x10 写入一段寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 */
int ssf_mems_modbus_write_regs(struct serdev_device *serdev, u16 display_reg,
                               u16 reg_count, const u16 *values,
                               size_t values_count, unsigned int timeout_ms);
/* 在已加锁的总线上写入一段寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 调用者须持 bus_lock。
 */
int ssf_mems_modbus_write_regs_locked(
    struct serdev_device *serdev, u16 display_reg, u16 reg_count,
    const u16 *values, size_t values_count, unsigned int timeout_ms);
/* 按块命令描述写入寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 */
int ssf_mems_modbus_write_block(struct serdev_device *serdev,
                                const struct ssf_block_cmd_desc *block,
                                const u16 *values, size_t values_count,
                                unsigned int timeout_ms);
/* 在已加锁的总线上执行块写入。
 * 返回：0 成功，负数为校验或通信错误。
 * 调用者须持 bus_lock。
 */
int ssf_mems_modbus_write_block_locked(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    const u16 *values, size_t values_count, unsigned int timeout_ms);
/* 尝试把完整帧匹配给当前请求。
 * 返回：true 已认领，false 未认领。
 * 认领从机异常帧也返回 true，不表示请求成功。
 */
bool ssf_mems_modbus_claim_frame(struct ssf_mems_xyzs_data *data,
                                 const u8 *buf, size_t len);
