#pragma once

#include <linux/atomic.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/serdev.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct ssf_mems_xyzs_data;
struct ssf_modbus_frame_desc;

/* 帧仅在同步回调期间有效，context 必须存活到接收清理结束，不得保存缓冲区或等待需要同一接收工作项处理的响应；返回 0 已处理，负值处理失败。 */
typedef int (*ssf_mems_frame_handler_t)(void *context, const u8 *buf, size_t len);

#define SSF_MEMS_RX_FIFO_SIZE 16384U /* 容纳定长批次上传时的接收调度抖动。 */
#define SSF_MEMS_FRAME_SLOT_NUM 4 /* 同时保留的帧边界候选数。 */
#define SSF_MEMS_FRAME_SLOT_FREE 0 /* 候选槽位可分配。 */
#define SSF_MEMS_FRAME_SLOT_USED 1 /* 候选槽位已占用。 */
#define SSF_MEMS_RX_EXPIRY_MARGIN_MS 100U /* 半帧寿命额外调度余量，ms。 */

/* 单个候选帧的解析阶段，与总线业务模式无关。 */
enum ssf_mems_rx_state {
  SSF_MEMS_RX_IDLE, /* 槽位未保存候选帧。 */
  SSF_MEMS_RX_FUNC, /* 已收到从站地址，等待功能码。 */
  SSF_MEMS_RX_DATA, /* 收集字节数或响应数据。 */
  SSF_MEMS_RX_CRC, /* 收集帧末尾的两个 CRC 字节。 */
  SSF_MEMS_RX_DONE, /* 已收齐帧，准备校验并交付。 */
  SSF_MEMS_RX_ERROR, /* 预留错误状态，异常候选通常直接释放。 */
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

  enum ssf_mems_rx_state state; /* 当前候选帧解析阶段。 */

  u8 slave_id; /* 本候选帧的从站地址。 */
  u8 function; /* 本候选帧的响应功能码。 */
  const struct ssf_modbus_frame_desc *frame_desc; // 来自表的响应长度和布局描述
  unsigned long started; // 候选帧创建时刻，寿命预算由当前波特率决定

  atomic_t in_use; /* FREE/USED 标记，防止同一槽位被重复认领。 */
};

/* 接收模块独立拥有字节 FIFO、FIFO 锁、解析工作项和候选帧槽位。 */
struct ssf_mems_modbus_receive_state {
  struct kfifo fifo;                 // 尚未解析的接收字节流
  spinlock_t fifo_lock;              // 保护 FIFO 读写和关闭状态
  struct work_struct work;           // 从 FIFO 中分离完整帧的工作项
  struct delayed_work expiry_work;   // 清理长期未完成的候选帧
  struct mutex parse_lock;           // 串行化解析、过期清理和 flush
  unsigned long last_rx;             // 最近接收字节的时间，由 fifo_lock 保护
  bool shutting_down;                // 关闭后禁止入队和调度新工作
  bool flushing;                     // 清理残留帧期间丢弃字节
  ssf_mems_frame_handler_t handler;  // 收到数据并组帧成功后自动调用这个函数指针并交出帧
  void *handler_context;             // 函数指针的上下文
  struct ssf_mems_frame_slot frame[SSF_MEMS_FRAME_SLOT_NUM]; // 候选帧上下文
};

/* 初始化接收状态并注册完整帧回调；返回 0 成功，-EINVAL 驱动数据/回调为空且不修改状态，其他负值来自 FIFO 分配；仅用于尚未投入使用的状态。 */
int ssf_mems_modbus_receive_init(struct ssf_mems_xyzs_data *data,
                               ssf_mems_frame_handler_t handler, void *context);

/* 清理已初始化的接收状态；无返回值，停止入队/工作项并释放 FIFO 和候选帧缓冲区，空指针直接退出。 */
void ssf_mems_modbus_receive_remove(struct ssf_mems_xyzs_data *data);

/* 清空 FIFO 和未完成候选帧；发送准备及波特率切换时调用，不得持请求锁。
 * 无返回值；空指针或已关闭时直接返回。 */
void ssf_mems_modbus_receive_flush(struct ssf_mems_xyzs_data *data);

/* 返回最近接收活动的 jiffies，用于发送前检测空闲；包含 flush 期间丢弃的字节。 */
unsigned long ssf_mems_modbus_receive_last_activity(struct ssf_mems_xyzs_data *data);

/* 将解析任务加入工作队列；无返回值，空设备、未绑定驱动数据或接收已关闭时直接退出。 */
void ssf_mems_modbus_queue_parse(struct serdev_device *serdev);

/* 返回实际入队字节数（0 表示未接收），-EINVAL 空设备/数据，-ENODEV
 * 无驱动数据或接收已关闭；FIFO 空间不足时返回部分入队数，便于 serdev 重送剩余数据。 */
ssize_t ssf_mems_rx_push(struct serdev_device *serdev,
                         const unsigned char *buf, size_t count);
