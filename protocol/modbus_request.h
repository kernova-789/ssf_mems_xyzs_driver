#pragma once

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

struct serdev_device;
struct ssf_mems_xyzs_data;
struct ssf_mems_frame_slot;

/* 保存当前正在执行的 Modbus 请求状态。 */
struct ssf_mems_modbus_request_state {
  struct mutex lock;       // 保护请求状态
  wait_queue_head_t waitq; // 等待响应的等待队列
  bool pending;            // 是否有请求正在等待
  bool shutting_down;      // 驱动是否正在关闭
  u8 slave_id;             // 当前请求的从机地址
  u8 function;             // 当前请求的功能码
  u16 display_reg;         // 当前请求的显示寄存器号
  u16 protocol_addr;       // 当前请求的协议寄存器地址
  u16 reg_count;           // 当前请求的寄存器数量
  u16 *values;             // 当前请求的结果缓冲区
  size_t values_count;     // 结果缓冲区容量
  u16 write_value;         // 当前 0x06 请求写入的值
  int status; // 当前请求的执行结果0 = 成功，负数 = 失败，负数对应不同 errno
};

#define SSF_MEMS_MODBUS_DEFAULT_TIMEOUT_MS 1000U
#define SSF_MEMS_MODBUS_DEFAULT_SLAVE_ID 0x01U
#define SSF_MEMS_MODBUS_WRITE_TIMEOUT_MS 100U
#define SSF_MEMS_MODBUS_READ_MAX_REGS 125U
#define SSF_MEMS_MODBUS_WRITE_MAX_REGS 123U

/* 初始化 Modbus 请求状态和工作队列。 */
int ssf_mems_modbus_request_init(struct ssf_mems_xyzs_data *data);
/* 清理 Modbus 请求状态并停止工作队列。 */
void ssf_mems_modbus_request_remove(struct ssf_mems_xyzs_data *data);
/* 将接收帧解析任务加入工作队列。 */
void ssf_mems_modbus_queue_parse(struct serdev_device *serdev);
/*
 * 连续读取 1～125 个普通寄存器，以 0x03 请求等待响应。
 * 全范围逐项查表：缺失地址返回 -ENOENT，不可读返回 -EACCES，
 * 特殊数据响应返回 -EOPNOTSUPP。检查失败时不发送任何字节。
 */
int ssf_mems_modbus_read(struct serdev_device *serdev, u16 display_reg,
                         u16 reg_count, u16 *values, size_t values_count,
                         unsigned int timeout_ms);
/* 读取单个寄存器的简化接口。 */
int ssf_mems_modbus_read_reg(struct serdev_device *serdev, u16 display_reg,
                             u16 *value, unsigned int timeout_ms);
/* 发送 0x06 请求并等待响应，仅支持表中 write_single 为真的可写寄存器。 */
int ssf_mems_modbus_write_reg(struct serdev_device *serdev, u16 display_reg,
                              u16 value, unsigned int timeout_ms);
/*
 * 连续写入 1～123 个寄存器，以 0x10 请求等待响应。
 * 全范围逐项查表：缺失地址返回 -ENOENT，不可写返回 -EACCES。
 * 协议地址必须连续；检查失败时不发送任何字节。
 * 此接口检查寄存器权限，设备是否接受任意子范围的 0x10 仍需实测。
 */
int ssf_mems_modbus_write_regs(struct serdev_device *serdev, u16 display_reg,
                               u16 reg_count, const u16 *values,
                               size_t values_count, unsigned int timeout_ms);
/*
 * 查块命令表，固定写入 40061～40070 的十个工作参数。
 * values[0]～values[9] 按寄存器地址递增排列，values_count 至少为 10。
 */
int ssf_mems_modbus_write_work_parameters(struct serdev_device *serdev,
                                         const u16 *values,
                                         size_t values_count,
                                         unsigned int timeout_ms);
/* 判断完整响应帧是否属于当前请求。 */
bool ssf_mems_modbus_claim_frame(struct ssf_mems_xyzs_data *data,
                                 struct ssf_mems_frame_slot *slot);
