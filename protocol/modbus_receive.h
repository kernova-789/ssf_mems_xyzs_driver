#pragma once

#include <linux/atomic.h>
#include <linux/kfifo.h>
#include <linux/serdev.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct ssf_mems_xyzs_data;
struct ssf_modbus_frame_desc;

/* 帧仅在同步回调期间有效，context 必须存活到接收清理结束，不得保存缓冲区或等待需要同一接收工作项处理的响应；返回 0 已处理，负值处理失败。 */
typedef int (*ssf_mems_frame_handler_t)(void *context, const u8 *buf, size_t len);

#define SSF_MEMS_RX_FIFO_SIZE 1024U
#define SSF_MEMS_FRAME_SLOT_NUM 4
#define SSF_MEMS_FRAME_SLOT_FREE 0
#define SSF_MEMS_FRAME_SLOT_USED 1

enum ssf_mems_rx_state {
  SSF_MEMS_RX_IDLE,
  SSF_MEMS_RX_FUNC,
  SSF_MEMS_RX_DATA,
  SSF_MEMS_RX_CRC,
  SSF_MEMS_RX_DONE,
  SSF_MEMS_RX_ERROR,
};

/*
 * 一个 frame_slot 就是一个独立的“候选帧解析上下文”。
 *
 * 多个 slot 可以同时存在，分别对同一串输入数据
 * 做不同的帧边界假设。
 */
struct ssf_mems_frame_slot {
  u8 *data;         //实际分配的帧缓冲区

  size_t data_len;  //data 缓冲区的容量
  size_t data_pos;  //当前已经接收了多少字节
  size_t frame_len; //根据帧头判断出来的“这一帧应该有多少字节”

  enum ssf_mems_rx_state state;

  u8 slave_id;
  u8 function;
  const struct ssf_modbus_frame_desc *frame_desc; // 来自表的响应长度和布局描述

  atomic_t in_use;
};

/* 接收模块独立拥有字节 FIFO、FIFO 锁、解析工作项和候选帧槽位。 */
struct ssf_mems_modbus_receive_state {
  struct kfifo fifo;                 // 尚未解析的接收字节流
  spinlock_t fifo_lock;              // 保护 FIFO 读写和关闭状态
  struct work_struct work;           // 从 FIFO 中分离完整帧的工作项
  bool shutting_down;                // 关闭后禁止入队和调度新工作
  ssf_mems_frame_handler_t handler;  // 收到数据并组帧成功后自动调用这个函数指针并交出帧
  void *handler_context;             // 函数指针的上下文
  struct ssf_mems_frame_slot frame[SSF_MEMS_FRAME_SLOT_NUM]; // 候选帧上下文
};

/* 初始化接收状态并注册完整帧回调；返回 0 成功，-EINVAL 驱动数据/回调为空且不修改状态，其他负值来自 FIFO 分配；仅用于尚未投入使用的状态。 */
int ssf_mems_modbus_receive_init(struct ssf_mems_xyzs_data *data,
                               ssf_mems_frame_handler_t handler, void *context);

/* 清理已初始化的接收状态；无返回值，停止入队/工作项并释放 FIFO 和候选帧缓冲区，空指针直接退出。 */
void ssf_mems_modbus_receive_remove(struct ssf_mems_xyzs_data *data);

/* 将解析任务加入工作队列；无返回值，空设备、未绑定驱动数据或接收已关闭时直接退出。 */
void ssf_mems_modbus_queue_parse(struct serdev_device *serdev);

/* 返回 count 全部入队（count 为 0 返回 0），-EINVAL 空设备/数据，-ENODEV 无驱动数据或接收已关闭，-ENOSPC 部分入队或 FIFO 已满。 */
int ssf_mems_rx_push(struct serdev_device *serdev, const unsigned char *buf,
                     size_t count);
