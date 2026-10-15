#include "modbus_receive.h"
#include "serial_transport.h"
#include "core.h"
#include "modbus.h"
#include "modbus_table.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include <linux/string.h>

/* 取出接收队列中的字节并逐个解析。
 * 返回：0 成功，-EINVAL 设备无效，-ENODEV 接收不可用。
 * 只由接收工作项调用。
 */
static int ssf_mems_modbus_receive_drain(struct serdev_device *serdev);
/* 释放候选帧缓冲区并归还槽位。
 * 返回：无。
 */
static void ssf_mems_rx_free_slot(struct ssf_mems_frame_slot *slot);

/* 计算半帧的超时长度。
 * 返回：超时 tick 数。
 * 按最长帧传输时间加余量计算。
 */
static unsigned long ssf_mems_rx_expiry_ticks(struct ssf_mems_xyzs_data *data) {
  return msecs_to_jiffies(ssf_mems_serial_wire_ms(data,
                         SSF_MEMS_MODBUS_MAX_FRAME_LEN) + SSF_MEMS_RX_EXPIRY_MARGIN_MS);
}

/* 清理长时间没有收完整的候选帧。
 * 返回：无。
 * 队列非空时暂不清理。
 */
static void ssf_mems_rx_expiry_workfn(struct work_struct *work) {
  struct ssf_mems_modbus_receive_state *rx =
      container_of(work, struct ssf_mems_modbus_receive_state, expiry_work.work);
  struct ssf_mems_xyzs_data *data =
      container_of(rx, struct ssf_mems_xyzs_data, modbus_rx);
  unsigned long flags;
  unsigned long expiry = ssf_mems_rx_expiry_ticks(data);
  unsigned long now = jiffies;
  bool expired;
  int i;

  mutex_lock(&rx->parse_lock);
  spin_lock_irqsave(&rx->fifo_lock, flags);
  if (rx->shutting_down == true || rx->flushing == true) {
    spin_unlock_irqrestore(&rx->fifo_lock, flags);
    mutex_unlock(&rx->parse_lock);
    return;
  }
  /* 已交付但尚未解析的字节应先被解析。push() 会重新启动空闲计时。 */
  expired = kfifo_is_empty(&rx->fifo) == true &&
            time_after_eq(now, rx->last_rx + expiry) == true;
  if (expired == false)
    mod_delayed_work(system_wq, &rx->expiry_work, expiry);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
  if (expired == true) {
    for (i = 0; i < SSF_MEMS_FRAME_SLOT_NUM; i++)
      ssf_mems_rx_free_slot(&rx->frame[i]);
  }
  mutex_unlock(&rx->parse_lock);
}

/* 执行接收队列的解析任务。
 * 返回：无。
 */
static void ssf_mems_modbus_rx_workfn(struct work_struct *work) {
  struct ssf_mems_modbus_receive_state *rx =
      container_of(work, struct ssf_mems_modbus_receive_state, work);
  struct ssf_mems_xyzs_data *data =
      container_of(rx, struct ssf_mems_xyzs_data, modbus_rx);

  ssf_mems_modbus_receive_drain(data->serdev);
}

/* 检查候选帧槽位是否被占用。
 * 返回：true 已占用，false 空闲。
 */
static bool ssf_mems_rx_slot_is_used(const struct ssf_mems_frame_slot *slot) {
  return atomic_read(&slot->in_use) == SSF_MEMS_FRAME_SLOT_USED;
}

/* 申请一个空闲候选帧槽位。
 * 返回：槽位指针，没有空位时为 NULL。
 */
static struct ssf_mems_frame_slot *
ssf_mems_rx_alloc_slot(struct ssf_mems_xyzs_data *data) {
  int i;

  for (i = 0; i < SSF_MEMS_FRAME_SLOT_NUM; i++) {
    struct ssf_mems_frame_slot *slot = &data->modbus_rx.frame[i];

    if (atomic_cmpxchg(&slot->in_use, SSF_MEMS_FRAME_SLOT_FREE,
                       SSF_MEMS_FRAME_SLOT_USED) == SSF_MEMS_FRAME_SLOT_FREE)
      return slot;
  }
  return NULL;
}

/* 释放候选帧缓冲区并归还槽位。
 * 返回：无。
 */
static void ssf_mems_rx_free_slot(struct ssf_mems_frame_slot *slot) {
  kfree(slot->data);
  slot->data = NULL;
  slot->data_len = 0;
  slot->data_pos = 0;
  slot->frame_len = 0;
  slot->state = SSF_MEMS_RX_IDLE;
  slot->slave_id = 0;
  slot->function = 0;
  slot->frame_desc = NULL;
  slot->started = 0;
  atomic_set(&slot->in_use, SSF_MEMS_FRAME_SLOT_FREE);
}

/* 初始化接收队列并注册完整帧回调。
 * 返回：0 成功，-EINVAL 参数无效，其他负数为分配错误。
 */
int ssf_mems_modbus_receive_init(struct ssf_mems_xyzs_data *data,
                               ssf_mems_frame_handler_t handler, void *context) {
  struct ssf_mems_modbus_receive_state *rx;
  int ret;
  int i;

  if (data == NULL || handler == NULL)
    return -EINVAL;
  rx = &data->modbus_rx;
  spin_lock_init(&rx->fifo_lock);
  mutex_init(&rx->parse_lock);
  rx->last_rx = jiffies;
  rx->shutting_down = true;
  rx->flushing = false;
  rx->handler = NULL;
  rx->handler_context = NULL;
  INIT_WORK(&rx->work, ssf_mems_modbus_rx_workfn);
  INIT_DELAYED_WORK(&rx->expiry_work, ssf_mems_rx_expiry_workfn);
  for (i = 0; i < SSF_MEMS_FRAME_SLOT_NUM; i++) {
    memset(&rx->frame[i], 0, sizeof(rx->frame[i]));
    rx->frame[i].state = SSF_MEMS_RX_IDLE;
    atomic_set(&rx->frame[i].in_use, SSF_MEMS_FRAME_SLOT_FREE);
  }

  ret = kfifo_alloc(&rx->fifo, SSF_MEMS_RX_FIFO_SIZE, GFP_KERNEL);
  if (ret != 0)
    return ret;
  rx->handler = handler;
  rx->handler_context = context;
  rx->shutting_down = false;
  return 0;
}

/* 清空接收队列和未完成的候选帧。
 * 返回：无。
 * 清理期间收到的字节会丢弃，调用时不能持请求锁。
 */
void ssf_mems_modbus_receive_flush(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_modbus_receive_state *rx;
  unsigned long flags;
  int i;

  if (data == NULL)
    return;
  rx = &data->modbus_rx;

  spin_lock_irqsave(&rx->fifo_lock, flags);
  if (rx->shutting_down == true) {
    spin_unlock_irqrestore(&rx->fifo_lock, flags);
    return;
  }
  WRITE_ONCE(rx->flushing, true);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);

  cancel_work_sync(&rx->work);
  cancel_delayed_work_sync(&rx->expiry_work);

  mutex_lock(&rx->parse_lock);
  spin_lock_irqsave(&rx->fifo_lock, flags);
  kfifo_reset(&rx->fifo);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
  for (i = 0; i < SSF_MEMS_FRAME_SLOT_NUM; i++)
    ssf_mems_rx_free_slot(&rx->frame[i]);
  mutex_unlock(&rx->parse_lock);

  spin_lock_irqsave(&rx->fifo_lock, flags);
  WRITE_ONCE(rx->flushing, false);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
}

/* 关闭接收并释放队列和候选帧。
 * 返回：无。
 * 先停止接收工作项，再释放资源。
 */
void ssf_mems_modbus_receive_remove(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_modbus_receive_state *rx;
  unsigned long flags;
  int i;

  if (data == NULL)
    return;
  rx = &data->modbus_rx;
  spin_lock_irqsave(&rx->fifo_lock, flags);
  WRITE_ONCE(rx->shutting_down, true);
  WRITE_ONCE(rx->flushing, false);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
  cancel_work_sync(&rx->work);
  cancel_delayed_work_sync(&rx->expiry_work);
  mutex_lock(&rx->parse_lock);
  rx->handler = NULL;
  rx->handler_context = NULL;
  for (i = 0; i < SSF_MEMS_FRAME_SLOT_NUM; i++)
    ssf_mems_rx_free_slot(&rx->frame[i]);
  kfifo_free(&rx->fifo);
  mutex_unlock(&rx->parse_lock);
}

/* 读取最近收到串口字节的时间。
 * 返回：接收时间的 jiffies 值。
 * 包含清理期间丢弃的字节。
 */
unsigned long ssf_mems_modbus_receive_last_activity(struct ssf_mems_xyzs_data *data) {
  unsigned long flags;
  unsigned long last_rx;

  spin_lock_irqsave(&data->modbus_rx.fifo_lock, flags);
  last_rx = data->modbus_rx.last_rx;
  spin_unlock_irqrestore(&data->modbus_rx.fifo_lock, flags);
  return last_rx;
}

/* 安排接收队列的解析任务。
 * 返回：无。
 */
void ssf_mems_modbus_queue_parse(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_receive_state *rx;
  unsigned long flags;

  if (serdev == NULL)
    return;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return;
  rx = &data->modbus_rx;
  spin_lock_irqsave(&rx->fifo_lock, flags);
  if (rx->shutting_down == false && rx->flushing == false)
    queue_work(system_wq, &rx->work);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
}

/* 初始化一个候选帧。
 * 返回：无。
 */
static void ssf_mems_rx_init_candidate(struct ssf_mems_frame_slot *slot, u8 slave_id) {
  slot->data = NULL;
  slot->data_len = 0;
  slot->data_pos = 0;
  slot->frame_len = 0;
  slot->state = SSF_MEMS_RX_FUNC;
  slot->slave_id = slave_id;
  slot->function = 0;
  slot->frame_desc = NULL;
  slot->started = jiffies;
}

/* 向候选帧追加一个字节。
 * 返回：0 成功，-EINVAL 无缓冲区，-ENOSPC 缓冲区已满。
 */
static int ssf_mems_rx_append_byte(struct ssf_mems_frame_slot *slot, u8 byte) {
  if (slot->data == NULL)
    return -EINVAL;
  if (slot->data_pos >= slot->data_len)
    return -ENOSPC;
  slot->data[slot->data_pos++] = byte;
  return 0;
}

/* 为候选帧分配缓冲区。
 * 返回：0 成功，-ENOMEM 分配失败。
 */
static int ssf_mems_rx_alloc_frame(struct ssf_mems_frame_slot *slot, size_t len) {
  slot->data = kzalloc(len, GFP_KERNEL);
  if (slot->data == NULL)
    return -ENOMEM;
  slot->data_len = len;
  slot->frame_len = len;
  slot->data_pos = 0;
  return 0;
}

/* 根据功能码准备候选帧的接收格式。
 * 返回：0 成功，负数为格式、分配或缓冲区错误。
 */
static int ssf_mems_rx_set_function(struct ssf_mems_frame_slot *slot, u8 function) {
  int ret;
  int len;

  slot->frame_desc = ssf_mems_modbus_find_rx_frame(function);
  if (slot->frame_desc == NULL)
    return -EOPNOTSUPP;
  slot->function = function;

  if (slot->frame_desc->rx_bytes_per_reg != 0) {
    if (slot->frame_desc->rx_byte_count_offset != 2)
      return -EOPNOTSUPP;
  } else {
    len = ssf_mems_modbus_rx_frame_len(slot->frame_desc, 0);
    if (len < 0)
      return len;
    ret = ssf_mems_rx_alloc_frame(slot, len);
    if (ret != 0)
      return ret;
    ret = ssf_mems_rx_append_byte(slot, slot->slave_id);
    if (ret != 0)
      return ret;
    ret = ssf_mems_rx_append_byte(slot, slot->function);
    if (ret != 0)
      return ret;
  }

  slot->state = SSF_MEMS_RX_DATA;
  return 0;
}

/* 根据字节数为读取应答准备缓冲区。
 * 返回：0 成功，负数为长度、分配或缓冲区错误。
 */
static int ssf_mems_rx_start_read_frame(struct ssf_mems_frame_slot *slot, u8 byte_count) {
  int len;
  int ret;

  len = ssf_mems_modbus_rx_frame_len(slot->frame_desc, byte_count);
  if (len < 0)
    return len;
  ret = ssf_mems_rx_alloc_frame(slot, len);
  if (ret != 0)
    return ret;
  ret = ssf_mems_rx_append_byte(slot, slot->slave_id);
  if (ret != 0)
    return ret;
  ret = ssf_mems_rx_append_byte(slot, slot->function);
  if (ret != 0)
    return ret;
  return ssf_mems_rx_append_byte(slot, byte_count);
}

/* 校验完整帧并交给回调处理。
 * 返回：无。
 * 回调结束后释放槽位，帧数据只在回调期间有效。
 */
static void ssf_mems_rx_complete(struct ssf_mems_xyzs_data *data,
                                struct ssf_mems_frame_slot *slot) {
  int ret;

  if (slot->data == NULL || slot->data_pos != slot->frame_len) {
    ssf_mems_rx_free_slot(slot);
    return;
  }
  ret = ssf_mems_modbus_check_crc(slot->data, slot->data_len);
  if (ret != 0) {
    dev_err_ratelimited(&data->serdev->dev, "invalid frame CRC/length: %d\n", ret);
    ssf_mems_rx_free_slot(slot);
    return;
  }
  ret = data->modbus_rx.handler(data->modbus_rx.handler_context,
                               slot->data, slot->data_len);
  if (ret < 0)
    dev_err(&data->serdev->dev, "frame handler failed: %d\n", ret);
  ssf_mems_rx_free_slot(slot);
}

/* 用一个字节推进候选帧的解析。
 * 返回：无。
 * 解析失败时释放槽位。
 */
static void ssf_mems_rx_process_candidate(struct ssf_mems_xyzs_data *data,
                                         struct ssf_mems_frame_slot *slot, u8 byte) {
  int ret;

  switch (slot->state) {
  case SSF_MEMS_RX_FUNC:
    ret = ssf_mems_rx_set_function(slot, byte);
    if (ret != 0)
      ssf_mems_rx_free_slot(slot);
    break;
  case SSF_MEMS_RX_DATA:
    if (slot->data == NULL)
      ret = ssf_mems_rx_start_read_frame(slot, byte);
    else
      ret = ssf_mems_rx_append_byte(slot, byte);
    if (ret != 0) {
      ssf_mems_rx_free_slot(slot);
      break;
    }
    if (slot->data_pos == slot->frame_len - SSF_MEMS_MODBUS_CRC_LEN)
      slot->state = SSF_MEMS_RX_CRC;
    break;
  case SSF_MEMS_RX_CRC:
    ret = ssf_mems_rx_append_byte(slot, byte);
    if (ret != 0) {
      ssf_mems_rx_free_slot(slot);
      break;
    }
    if (slot->data_pos == slot->frame_len) {
      slot->state = SSF_MEMS_RX_DONE;
      ssf_mems_rx_complete(data, slot);
    }
    break;
  default:
    ssf_mems_rx_free_slot(slot);
    break;
  }
}

/* 遇到本机地址时创建候选帧。
 * 返回：无。
 */
static void ssf_mems_rx_create_candidate(struct ssf_mems_xyzs_data *data, u8 byte) {
  struct ssf_mems_frame_slot *slot;

  if (byte != data->slave_id)
    return;
  slot = ssf_mems_rx_alloc_slot(data);
  if (slot == NULL) {
    dev_warn_ratelimited(&data->serdev->dev, "no free frame slot for new candidate\n");
    return;
  }
  ssf_mems_rx_init_candidate(slot, byte);
}

/* 解析一个接收字节。
 * 返回：无。
 * 原始数据和升级应答按当前模式分流。
 */
static void ssf_mems_rx_process_byte(struct ssf_mems_xyzs_data *data, u8 byte) {
  int i;

  if (READ_ONCE(data->protocol.mode) == SSF_MEMS_LINK_OTA) {
    ssf_mems_ota_receive_byte(data, byte);
    return;
  }

  if (READ_ONCE(data->raw.active) == true) {
    ssf_mems_raw_receive_byte(data, byte);
    return;
  }

  for (i = 0; i < SSF_MEMS_FRAME_SLOT_NUM; i++) {
    struct ssf_mems_frame_slot *slot = &data->modbus_rx.frame[i];

    if (ssf_mems_rx_slot_is_used(slot) == false)
      continue;
    if (time_after_eq(jiffies, slot->started + ssf_mems_rx_expiry_ticks(data)) == true)
      ssf_mems_rx_free_slot(slot);
    else
      ssf_mems_rx_process_candidate(data, slot, byte);
  }
  ssf_mems_rx_create_candidate(data, byte);
}

/* 把串口字节放入接收队列。
 * 返回：已接收的字节数，负数为参数或接收状态错误。
 * 队列不足时允许部分入队，清理期间直接丢弃。
 */
ssize_t ssf_mems_rx_push(struct serdev_device *serdev,
                         const unsigned char *buf, size_t count) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_receive_state *rx;
  unsigned long flags;
  unsigned int ret;

  if (count == 0)
    return 0;
  if (serdev == NULL || buf == NULL)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;
  rx = &data->modbus_rx;
  spin_lock_irqsave(&rx->fifo_lock, flags);
  if (rx->shutting_down == true) {
    spin_unlock_irqrestore(&rx->fifo_lock, flags);
    return -ENODEV;
  }
  rx->last_rx = jiffies;
  if (rx->flushing == true) {
    spin_unlock_irqrestore(&rx->fifo_lock, flags);
    return (ssize_t)count;
  }
  ret = kfifo_in(&rx->fifo, buf, count);
  mod_delayed_work(system_wq, &rx->expiry_work, ssf_mems_rx_expiry_ticks(data));
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
  ssf_mems_modbus_queue_parse(serdev);
  if (ret != count) {
    dev_err(&serdev->dev, "rx fifo overflow: received %zu bytes, stored %u bytes\n", count, ret);
  }
  return (ssize_t)ret;
}

/* 取出接收队列中的字节并逐个解析。
 * 返回：0 成功，-EINVAL 设备无效，-ENODEV 接收不可用。
 * 只由接收工作项调用。
 */
static int ssf_mems_modbus_receive_drain(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_receive_state *rx;
  u8 byte;

  if (serdev == NULL)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;
  rx = &data->modbus_rx;
  mutex_lock(&rx->parse_lock);
  if (READ_ONCE(rx->shutting_down) == true) {
    mutex_unlock(&rx->parse_lock);
    return -ENODEV;
  }
  while (READ_ONCE(rx->shutting_down) == false && READ_ONCE(rx->flushing) == false &&
         kfifo_out_spinlocked(&rx->fifo, &byte, sizeof(byte), &rx->fifo_lock) == sizeof(byte))
    ssf_mems_rx_process_byte(data, byte);
  mutex_unlock(&rx->parse_lock);
  return 0;
}
