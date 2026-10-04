#pragma once

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "modbus_table.h"

struct serdev_device;
struct ssf_mems_xyzs_data;
struct ssf_mems_frame_slot;

/* 保存当前正在执行的 Modbus 请求状态。 */
struct ssf_mems_modbus_request_state {
  struct mutex lock;       // 保护请求状态
  wait_queue_head_t waitq; // 等待响应的等待队列
  bool busy;               // 保持占用直到等待者取走最终结果
  bool pending;            // 是否有请求正在等待
  bool shutting_down;      // 驱动是否正在关闭
  u8 slave_id;             // 当前请求的从机地址
  struct ssf_modbus_transfer transfer; // 保存当前实际请求的帧/块/地址描述
  u16 *values;             // 当前请求的结果缓冲区
  size_t values_count;     // 结果缓冲区容量
  u16 write_value;         // 当前 0x06 请求写入的值
  int status; // 当前请求的执行结果0 = 成功，负数 = 失败，负数对应不同 errno
};

#define SSF_MEMS_MODBUS_DEFAULT_TIMEOUT_MS 1000U
#define SSF_MEMS_MODBUS_WRITE_TIMEOUT_MS 100U

/* 初始化请求状态和工作队列；返回 0，当前实现没有失败分支。 */
int ssf_mems_modbus_request_init(struct ssf_mems_xyzs_data *data);
/* 清理请求状态并停止工作队列；无返回值，等待中的请求以 -ENODEV 结束。 */
void ssf_mems_modbus_request_remove(struct ssf_mems_xyzs_data *data);
/* 将接收帧解析任务加入工作队列；无返回值，空设备或未绑定驱动数据时直接退出。 */
void ssf_mems_modbus_queue_parse(struct serdev_device *serdev);
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
/* 读取单个寄存器；返回 0 成功，失败原因和负值同 ssf_mems_modbus_read()。 */
int ssf_mems_modbus_read_reg(struct serdev_device *serdev, u16 display_reg,
                             u16 *value, unsigned int timeout_ms);
/* 以 0x06 写入 write_single 为真的可写寄存器；返回 0 成功，-EOPNOTSUPP 表项不支持 0x06；其余校验、发送、响应和等待错误同 ssf_mems_modbus_write_regs()。 */
int ssf_mems_modbus_write_reg(struct serdev_device *serdev, u16 display_reg,
                              u16 value, unsigned int timeout_ms);
/* 连续写入 1～123 个寄存器，校验失败不发送；返回 0 成功，-EINVAL 参数/数量/地址范围错误，-ENOENT 地址未入表，-EACCES 不可写，-EOPNOTSUPP 帧格式不支持，-ENODEV 无设备/关闭中，-EBUSY 请求占用，-ENOMEM 分配失败，-EIO 发送不足，-EREMOTEIO 从机异常，-ETIMEDOUT 超时，-ERESTARTSYS 信号打断；其他负值来自组帧或串口。 */
/* 此接口检查寄存器权限，设备是否接受任意子范围的 0x10 仍需实测。 */
int ssf_mems_modbus_write_regs(struct serdev_device *serdev, u16 display_reg,
                               u16 reg_count, const u16 *values,
                               size_t values_count, unsigned int timeout_ms);
/* 按地址递增写入 40061～40070，values_count 至少为 10；返回 0 写入成功，-ENOENT 块命令缺失，-EOPNOTSUPP 块命令格式不支持；参数、权限、发送及等待错误同 ssf_mems_modbus_write_regs()。 */
int ssf_mems_modbus_write_work_parameters(struct serdev_device *serdev,
                                         const u16 *values,
                                         size_t values_count,
                                         unsigned int timeout_ms);
/* 返回 true 帧已消费（成功、从机异常或 -EFAULT），false 无等待请求、响应不匹配或帧损坏；true 不代表请求成功。 */
bool ssf_mems_modbus_claim_frame(struct ssf_mems_xyzs_data *data,
                                 struct ssf_mems_frame_slot *slot);
