#include "modbus_receive.h"
#include "core.h"
#include "modbus.h"
#include "modbus_table.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include <linux/string.h>

/* 仅由接收工作项串行消费 FIFO 并拼帧；返回 0 已消费，-EINVAL 空设备，-ENODEV 无驱动数据或接收已关闭。 */
static int ssf_mems_modbus_receive_drain(struct serdev_device *serdev);

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
  atomic_set(&slot->in_use, SSF_MEMS_FRAME_SLOT_FREE);
}

/* 初始化接收模块并注册完整帧回调；返回 0 成功，-EINVAL 驱动数据/回调为空且不修改状态，其他负值来自 FIFO 分配。 */
int ssf_mems_modbus_receive_init(struct ssf_mems_xyzs_data *data,
                               ssf_mems_frame_handler_t handler, void *context) {
  struct ssf_mems_modbus_receive_state *rx;
  int ret;
  int i;

  if (!data || !handler)
    return -EINVAL;
  rx = &data->modbus_rx;
  spin_lock_init(&rx->fifo_lock);
  rx->shutting_down = true;
  rx->handler = NULL;
  rx->handler_context = NULL;
  INIT_WORK(&rx->work, ssf_mems_modbus_rx_workfn);
  for (i = 0; i < SSF_MEMS_FRAME_SLOT_NUM; i++) {
    memset(&rx->frame[i], 0, sizeof(rx->frame[i]));
    rx->frame[i].state = SSF_MEMS_RX_IDLE;
    atomic_set(&rx->frame[i].in_use, SSF_MEMS_FRAME_SLOT_FREE);
  }

  ret = kfifo_alloc(&rx->fifo, SSF_MEMS_RX_FIFO_SIZE, GFP_KERNEL);
  if (ret)
    return ret;
  rx->handler = handler;
  rx->handler_context = context;
  rx->shutting_down = false;
  return 0;
}

/* 清理接收模块；无返回值，先禁止新入队和新工作，再停止工作并释放 FIFO/候选帧，空指针直接退出。 */
void ssf_mems_modbus_receive_remove(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_modbus_receive_state *rx;
  unsigned long flags;
  int i;

  if (!data)
    return;
  rx = &data->modbus_rx;
  spin_lock_irqsave(&rx->fifo_lock, flags);
  WRITE_ONCE(rx->shutting_down, true);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
  cancel_work_sync(&rx->work);
  rx->handler = NULL;
  rx->handler_context = NULL;
  for (i = 0; i < SSF_MEMS_FRAME_SLOT_NUM; i++)
    ssf_mems_rx_free_slot(&rx->frame[i]);
  kfifo_free(&rx->fifo);
}

/* 将接收解析加入工作队列；无返回值，空设备、未绑定驱动数据或接收已关闭时直接退出。 */
void ssf_mems_modbus_queue_parse(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_receive_state *rx;
  unsigned long flags;

  if (!serdev)
    return;
  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return;
  rx = &data->modbus_rx;
  spin_lock_irqsave(&rx->fifo_lock, flags);
  if (!rx->shutting_down)
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
}

/* 追加一个字节；返回 0 成功，-EINVAL 无缓冲区，-ENOSPC 缓冲区已满。 */
static int ssf_mems_rx_append_byte(struct ssf_mems_frame_slot *slot, u8 byte) {
  if (!slot->data)
    return -EINVAL;
  if (slot->data_pos >= slot->data_len)
    return -ENOSPC;
  slot->data[slot->data_pos++] = byte;
  return 0;
}

/* 分配帧缓冲区；返回 0 成功，-ENOMEM 分配失败。 */
static int ssf_mems_rx_alloc_frame(struct ssf_mems_frame_slot *slot, size_t len) {
  slot->data = kzalloc(len, GFP_KERNEL);
  if (!slot->data)
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
  if (!slot->frame_desc)
    return -EOPNOTSUPP;
  slot->function = function;

  if (slot->frame_desc->rx_bytes_per_reg) {
    if (slot->frame_desc->rx_byte_count_offset != 2)
      return -EOPNOTSUPP;
  } else {
    len = ssf_mems_modbus_rx_frame_len(slot->frame_desc, 0);
    if (len < 0)
      return len;
    ret = ssf_mems_rx_alloc_frame(slot, len);
    if (ret)
      return ret;
    ret = ssf_mems_rx_append_byte(slot, slot->slave_id);
    if (ret)
      return ret;
    ret = ssf_mems_rx_append_byte(slot, slot->function);
    if (ret)
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
  if (ret)
    return ret;
  ret = ssf_mems_rx_append_byte(slot, slot->slave_id);
  if (ret)
    return ret;
  ret = ssf_mems_rx_append_byte(slot, slot->function);
  if (ret)
    return ret;
  return ssf_mems_rx_append_byte(slot, byte_count);
}

/* 校验并通过同步回调交出完整帧；无返回值，校验/处理失败记录日志，所有路径均释放槽位。 */
static void ssf_mems_rx_complete(struct ssf_mems_xyzs_data *data,
                                struct ssf_mems_frame_slot *slot) {
  int ret;

  if (!slot->data || slot->data_pos != slot->frame_len) {
    ssf_mems_rx_free_slot(slot);
    return;
  }
  ret = ssf_mems_modbus_check_crc(slot->data, slot->data_len);
  if (ret) {
    dev_err(&data->serdev->dev, "invalid frame CRC/length: %d\n", ret);
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
    if (ret)
      ssf_mems_rx_free_slot(slot);
    break;
  case SSF_MEMS_RX_DATA:
    if (!slot->data)
      ret = ssf_mems_rx_start_read_frame(slot, byte);
    else
      ret = ssf_mems_rx_append_byte(slot, byte);
    if (ret) {
      ssf_mems_rx_free_slot(slot);
      break;
    }
    if (slot->data_pos == slot->frame_len - SSF_MEMS_MODBUS_CRC_LEN)
      slot->state = SSF_MEMS_RX_CRC;
    break;
  case SSF_MEMS_RX_CRC:
    ret = ssf_mems_rx_append_byte(slot, byte);
    if (ret) {
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

/* 识别从机地址并创建候选帧；无返回值，非当前支持地址或无槽位时退出，私有 0x15 帧仍待实现。 */
static void ssf_mems_rx_create_candidate(struct ssf_mems_xyzs_data *data, u8 byte) {
  struct ssf_mems_frame_slot *slot;

  if (byte != SSF_MEMS_MODBUS_DEFAULT_SLAVE_ID)
    return;
  slot = ssf_mems_rx_alloc_slot(data);
  if (!slot) {
    dev_warn(&data->serdev->dev, "no free frame slot for new candidate\n");
    return;
  }
  ssf_mems_rx_init_candidate(slot, byte);
}

/* 先让已有候选帧消费字节，再尝试以该字节建立新帧；无返回值。 */
static void ssf_mems_rx_process_byte(struct ssf_mems_xyzs_data *data, u8 byte) {
  int i;

  for (i = 0; i < SSF_MEMS_FRAME_SLOT_NUM; i++) {
    if (ssf_mems_rx_slot_is_used(&data->modbus_rx.frame[i]))
      ssf_mems_rx_process_candidate(data, &data->modbus_rx.frame[i], byte);
  }
  ssf_mems_rx_create_candidate(data, byte);
}

/* 返回 count 全部入队（count 为 0 返回 0），-EINVAL 空设备/数据，-ENODEV 无驱动数据或接收已关闭，-ENOSPC 部分入队或 FIFO 已满。 */
int ssf_mems_rx_push(struct serdev_device *serdev, const unsigned char *buf, size_t count) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_receive_state *rx;
  unsigned long flags;
  unsigned int ret;

  if (!count)
    return 0;
  if (!serdev || !buf)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;
  rx = &data->modbus_rx;
  spin_lock_irqsave(&rx->fifo_lock, flags);
  if (rx->shutting_down) {
    spin_unlock_irqrestore(&rx->fifo_lock, flags);
    return -ENODEV;
  }
  ret = kfifo_in(&rx->fifo, buf, count);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
  ssf_mems_modbus_queue_parse(serdev);
  if (ret != count) {
    dev_err(&serdev->dev, "rx fifo overflow: received %zu bytes, stored %u bytes\n", count, ret);
    return -ENOSPC;
  }
  return ret;
}

/* 仅由接收工作项串行消费 FIFO 并拼帧；返回 0 已消费，-EINVAL 空设备，-ENODEV 无驱动数据或接收已关闭，单帧错误内部处理或记录日志。 */
static int ssf_mems_modbus_receive_drain(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_receive_state *rx;
  u8 byte;

  if (!serdev)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;
  rx = &data->modbus_rx;
  if (READ_ONCE(rx->shutting_down))
    return -ENODEV;
  while (!READ_ONCE(rx->shutting_down) &&
         kfifo_out_spinlocked(&rx->fifo, &byte, sizeof(byte), &rx->fifo_lock) == sizeof(byte))
    ssf_mems_rx_process_byte(data, byte);
  return 0;
}
