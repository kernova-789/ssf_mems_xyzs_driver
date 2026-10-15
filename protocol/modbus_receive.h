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

/* 处理接收器交出的完整帧。
 * 返回：0 成功，负数为处理错误。
 * 帧仅在回调期间有效；context 须保持有效，回调内不能等待接收应答。
 */
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

/* 初始化接收队列并注册完整帧回调。
 * 返回：0 成功，-EINVAL 参数无效，其他负数为分配错误。
 */
int ssf_mems_modbus_receive_init(struct ssf_mems_xyzs_data *data,
                               ssf_mems_frame_handler_t handler, void *context);

/* 关闭接收并释放队列和候选帧。
 * 返回：无。
 * 先停止接收工作项，再释放资源。
 */
void ssf_mems_modbus_receive_remove(struct ssf_mems_xyzs_data *data);

/* 清空接收队列和未完成的候选帧。
 * 返回：无。
 * 清理期间收到的字节会丢弃，调用时不能持请求锁。
 */
void ssf_mems_modbus_receive_flush(struct ssf_mems_xyzs_data *data);

/* 读取最近收到串口字节的时间。
 * 返回：接收时间的 jiffies 值。
 * 包含清理期间丢弃的字节。
 */
unsigned long ssf_mems_modbus_receive_last_activity(struct ssf_mems_xyzs_data *data);

/* 安排接收队列的解析任务。
 * 返回：无。
 */
void ssf_mems_modbus_queue_parse(struct serdev_device *serdev);

/* 把串口字节放入接收队列。
 * 返回：已接收的字节数，负数为参数或接收状态错误。
 * 队列不足时允许部分入队，清理期间直接丢弃。
 */
ssize_t ssf_mems_rx_push(struct serdev_device *serdev,
                         const unsigned char *buf, size_t count);
