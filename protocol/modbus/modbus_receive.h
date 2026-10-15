#pragma once

#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/serdev.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "modbus/modbus_types.h"

struct ssf_mems_xyzs_data;

/* 处理接收器交出的完整帧。
 * 返回：0 成功，负数为处理错误。
 * 帧仅在回调期间有效；context 须保持有效，回调内不能等待接收应答。
 */
typedef int (*ssf_mems_frame_handler_t)(void *context, const u8 *buf, size_t len);

#define SSF_MEMS_RX_FIFO_SIZE 16384U /* 容纳定长批次上传时的接收调度抖动。 */
#define SSF_MEMS_RX_EXPIRY_MARGIN_MS 100U /* 半帧寿命额外调度余量，ms。 */

/* FIFO 由 fifo_lock 保护，解析窗口及回调由 parse_lock 保护。 */
struct ssf_mems_modbus_receive_state {
  struct kfifo fifo;                 // 尚未解析的接收字节流
  spinlock_t fifo_lock;              // 保护 FIFO 读写和关闭状态
  struct work_struct work;           // 从 FIFO 中分离完整帧的工作项
  struct delayed_work expiry_work;   // 清理长期未完成的候选帧
  struct mutex parse_lock;           // 串行化解析、过期清理和 flush
  unsigned long last_rx;             // 最近接收字节的时间，由 fifo_lock 保护
  bool shutting_down;                // 关闭后禁止入队和调度新工作
  bool flushing;                     // 清理残留帧期间丢弃字节
  ssf_mems_frame_handler_t handler;   // 完整帧回调
  void *handler_context;             // 函数指针的上下文
  u8 frame[SSF_MEMS_MODBUS_MAX_FRAME_LEN]; /* 保留所有未完成帧头的有界窗口。 */
  size_t frame_used;                 /* 窗口内字节数。 */
  unsigned long frame_started[SSF_MEMS_MODBUS_MAX_FRAME_LEN]; /* 各字节作为帧头的创建时间。 */
};

/* 初始化接收队列并注册完整帧回调。
 * 返回：0 成功，-EINVAL 参数无效，其他负数为分配错误。
 */
int ssf_mems_modbus_receive_init(struct ssf_mems_xyzs_data *data,
                               ssf_mems_frame_handler_t handler, void *context);

/* 关闭接收、释放队列并清空解析窗口。
 * 返回：无。
 * 先停止接收工作项，再释放资源。
 */
void ssf_mems_modbus_receive_remove(struct ssf_mems_xyzs_data *data);

/* 清空接收队列和未完成的解析窗口。
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
