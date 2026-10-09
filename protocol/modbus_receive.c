#include "modbus_receive.h"
#include "core.h"
#include "modbus.h"
#include "modbus_table.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include <linux/string.h>

/* 仅由接收工作项串行消费 FIFO 并拼帧；返回 0 已消费，-EINVAL 空设备，-ENODEV 无驱动数据或接收已关闭。 */
static int ssf_mems_modbus_receive_drain(struct serdev_device *serdev);
static void ssf_mems_rx_free_slot(struct ssf_mems_frame_slot *slot);

/* 串口驱动可能分批交付字节，不能拿交付时间直接当作线上的 t1.5。
 * 使用最长 RTU 帧传输时间加调度余量，避免因工作队列延迟误丢完整帧。 */
static unsigned long ssf_mems_rx_expiry_ticks(struct ssf_mems_xyzs_data *data) {
  unsigned int baudrate = READ_ONCE(data->protocol.host_baudrate);
  unsigned int bits = READ_ONCE(data->protocol.parity) == SSF_MEMS_PARITY_NONE ? 10U : 11U;
  unsigned int wire_ms;

  if (baudrate == 0)
    baudrate = 9600;
  wire_ms = DIV_ROUND_UP(SSF_MEMS_MODBUS_MAX_FRAME_LEN * bits * 1000U, baudrate);
  return msecs_to_jiffies(wire_ms + SSF_MEMS_RX_EXPIRY_MARGIN_MS);
}

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
  expired = kfifo_is_empty(&rx->fifo) && time_after_eq(now, rx->last_rx + expiry);
  if (expired == false)
    mod_delayed_work(system_wq, &rx->expiry_work, expiry);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
  if (expired == true) {
    for (i = 0; i < SSF_MEMS_FRAME_SLOT_NUM; i++)
      ssf_mems_rx_free_slot(&rx->frame[i]);
  }
  mutex_unlock(&rx->parse_lock);
}

/* 接收工作项串行消费 FIFO 并拼帧；无返回值。 */
static void ssf_mems_modbus_rx_workfn(struct work_struct *work) {
  struct ssf_mems_modbus_receive_state *rx =
      container_of(work, struct ssf_mems_modbus_receive_state, work);
  struct ssf_mems_xyzs_data *data =
      container_of(rx, struct ssf_mems_xyzs_data, modbus_rx);

  ssf_mems_modbus_receive_drain(data->serdev);
}

/* 返回 true 槽位已占用，false 槽位未占用。 */
static bool ssf_mems_rx_slot_is_used(const struct ssf_mems_frame_slot *slot) {
  return atomic_read(&slot->in_use) == SSF_MEMS_FRAME_SLOT_USED;
}

/* 返回已认领的槽位；没有空闲槽位返回 NULL。 */
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

/* 释放帧缓冲区并归还槽位；无返回值。 */
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

/* 初始化接收模块并注册完整帧回调；返回 0 成功，-EINVAL 驱动数据/回调为空且不修改状态，其他负值来自 FIFO 分配。 */
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

/* 清空旧串口参数下的字节和候选帧；清理期间到达的字节也会被丢弃。 */
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

/* 清理接收模块；无返回值，先禁止新入队和新工作，再停止工作并释放 FIFO/候选帧，空指针直接退出。 */
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

unsigned long ssf_mems_modbus_receive_last_activity(struct ssf_mems_xyzs_data *data) {
  unsigned long flags;
  unsigned long last_rx;

  spin_lock_irqsave(&data->modbus_rx.fifo_lock, flags);
  last_rx = data->modbus_rx.last_rx;
  spin_unlock_irqrestore(&data->modbus_rx.fifo_lock, flags);
  return last_rx;
}

/* 将接收解析加入工作队列；无返回值，空设备、未绑定驱动数据或接收已关闭时直接退出。 */
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

/* 初始化候选帧状态；无返回值。 */
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

/* 追加一个字节；返回 0 成功，-EINVAL 无缓冲区，-ENOSPC 缓冲区已满。 */
static int ssf_mems_rx_append_byte(struct ssf_mems_frame_slot *slot, u8 byte) {
  if (slot->data == NULL)
    return -EINVAL;
  if (slot->data_pos >= slot->data_len)
    return -ENOSPC;
  slot->data[slot->data_pos++] = byte;
  return 0;
}

/* 分配帧缓冲区；返回 0 成功，-ENOMEM 分配失败。 */
static int ssf_mems_rx_alloc_frame(struct ssf_mems_frame_slot *slot, size_t len) {
  slot->data = kzalloc(len, GFP_KERNEL);
  if (slot->data == NULL)
    return -ENOMEM;
  slot->data_len = len;
  slot->frame_len = len;
  slot->data_pos = 0;
  return 0;
}

/* 按表选择响应格式；返回 0 成功，-EOPNOTSUPP 功能码/布局不支持；其他负值来自长度计算、分配或追加字节。 */
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

/* 根据字节数准备读取响应；返回 0 成功，-EMSGSIZE 字节数异常；其他负值来自分配或追加字节。 */
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

/* 校验并通过同步回调交出完整帧；无返回值，校验/处理失败记录日志，所有路径均释放槽位。 */
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

/* 推进候选帧状态；无返回值，准备或追加失败时释放槽位。 */
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

/* 识别普通 Modbus 从机地址并创建候选帧；私有流在字节入口另行处理。 */
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

/* 先让已有候选帧消费字节，再尝试以该字节建立新帧；无返回值。 */
static void ssf_mems_rx_process_byte(struct ssf_mems_xyzs_data *data, u8 byte) {
  int i;

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

/* 返回实际入队字节数；FIFO 不足时保留已入队部分，由 serdev 根据返回值重送剩余数据。 */
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

/* 仅由接收工作项串行消费 FIFO 并拼帧；返回 0 已消费，-EINVAL 空设备，-ENODEV 无驱动数据或接收已关闭，单帧错误内部处理或记录日志。 */
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
  while (!READ_ONCE(rx->shutting_down) && !READ_ONCE(rx->flushing) &&
         kfifo_out_spinlocked(&rx->fifo, &byte, sizeof(byte), &rx->fifo_lock) == sizeof(byte))
    ssf_mems_rx_process_byte(data, byte);
  mutex_unlock(&rx->parse_lock);
  return 0;
}
