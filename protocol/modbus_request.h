#pragma once

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "modbus_types.h"

struct serdev_device;
struct ssf_mems_xyzs_data;

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

#define SSF_MEMS_MODBUS_DEFAULT_TIMEOUT_MS 1000U
#define SSF_MEMS_MODBUS_MIN_WRITE_TIMEOUT_MS 100U
#define SSF_MEMS_MODBUS_WRITE_MARGIN_MS 100U
/* 无事务编号的 RTU 只能在已知最大响应延迟内隔离旧应答，实测后可调整。 */
#define SSF_MEMS_MODBUS_RECOVERY_GUARD_MS 1000U
#define SSF_MEMS_MODBUS_IDLE_WAIT_MS 1000U

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
/* 与 read_block_range 相同，但调用者必须已持有 protocol.bus_lock。 */
int ssf_mems_modbus_read_block_range_locked(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    u16 display_reg, u16 reg_count, u16 *values, size_t values_count,
    unsigned int timeout_ms);
/* 读取单个寄存器；返回 0 成功，失败原因和负值同 ssf_mems_modbus_read()。 */
int ssf_mems_modbus_read_reg(struct serdev_device *serdev, u16 display_reg,
                             u16 *value, unsigned int timeout_ms);
/* 与 read_reg 相同，但调用者必须已持有 protocol.bus_lock。 */
int ssf_mems_modbus_read_reg_locked(struct serdev_device *serdev,
                                    u16 display_reg, u16 *value,
                                    unsigned int timeout_ms);
/* 仅供协议层持 bus_lock 做只读恢复确认；允许恢复模式及请求关闭后的读取。
 * 不重新开放普通请求，接收模块必须仍存活；用于收流退出和卸载清理。 */
int ssf_mems_modbus_read_reg_recovery_locked(struct serdev_device *serdev,
                                             u16 display_reg, u16 *value,
                                             unsigned int timeout_ms);
/* 以 0x06 写入 write_single 为真的可写寄存器；返回 0 成功，-EOPNOTSUPP 表项不支持 0x06；其余校验、发送、响应和等待错误同 ssf_mems_modbus_write_regs()。 */
int ssf_mems_modbus_write_reg(struct serdev_device *serdev, u16 display_reg,
                              u16 value, unsigned int timeout_ms);
/* 与 write_reg 相同，但调用者必须已持有 protocol.bus_lock。 */
int ssf_mems_modbus_write_reg_locked(struct serdev_device *serdev,
                                     u16 display_reg, u16 value,
                                     unsigned int timeout_ms);
/* 连续写入 1～123 个寄存器，校验失败不发送；返回 0 成功，-EINVAL 参数/数量/地址范围错误，-ENOENT 地址未入表，-EACCES 不可写，-EOPNOTSUPP 帧格式不支持，-ENODEV 无设备/关闭中，-EBUSY 请求占用，-ENOMEM 分配失败，-EIO 发送不足，-EREMOTEIO 从机异常，-ETIMEDOUT 超时，-ERESTARTSYS 信号打断；其他负值来自组帧或串口。 */
/* 此接口检查寄存器权限，设备是否接受任意子范围的 0x10 仍需实测。 */
int ssf_mems_modbus_write_regs(struct serdev_device *serdev, u16 display_reg,
                               u16 reg_count, const u16 *values,
                               size_t values_count, unsigned int timeout_ms);
/* 按块描述执行写入，保留功能码/响应格式/方向和固定范围校验；返回 0 成功，-EINVAL 参数错误，-EOPNOTSUPP 不是可写块或格式不支持；其余负值同 ssf_mems_modbus_write_regs()。 */
int ssf_mems_modbus_write_block(struct serdev_device *serdev,
                                const struct ssf_block_cmd_desc *block,
                                const u16 *values, size_t values_count,
                                unsigned int timeout_ms);
/* 与 write_block 相同，但调用者必须已持有 protocol.bus_lock。 */
int ssf_mems_modbus_write_block_locked(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    const u16 *values, size_t values_count, unsigned int timeout_ms);
/* 返回 true 帧已消费（成功、从机异常或 -EFAULT），false 无等待请求、响应不匹配或帧损坏；true 不代表请求成功，不持有 buf。 */
bool ssf_mems_modbus_claim_frame(struct ssf_mems_xyzs_data *data,
                                 const u8 *buf, size_t len);
