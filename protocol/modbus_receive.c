#include "modbus_receive.h"
#include "core.h"
#include "modbus.h"
#include "modbus_request.h"
#include "modbus_table.h"
#include "protocol.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/slab.h>

/* 返回 true 槽位已占用，false 槽位未占用。 */
static bool ssf_mems_rx_slot_is_used(const struct ssf_mems_frame_slot *slot) {
  return atomic_read(&slot->in_use) == SSF_MEMS_FRAME_SLOT_USED;
}

/* 返回已认领的槽位；没有空闲槽位返回 NULL。 */
static struct ssf_mems_frame_slot *
ssf_mems_rx_alloc_slot(struct ssf_mems_xyzs_data *data) {
  int i;

  for (i = 0; i < SSF_MEMS_FRAME_SLOT_NUM; i++) {
    struct ssf_mems_frame_slot *slot = &data->frame[i];

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

/* 校验并分发完整帧；无返回值，损坏帧释放槽位，未认领帧策略仍保留 TODO。 */
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
  if (ssf_mems_modbus_claim_frame(data, slot)) {
    ssf_mems_rx_free_slot(slot);
    return;
  }

  /* TODO：按此前要求保留未声明/未定义调用，未认领帧策略确定后再实现。 */
  ret = ssf_mems_protocol_process(data, slot);
  if (ret < 0)
    dev_err(&data->serdev->dev, "failed to ssf_mems_protocol_process: %d\n", ret);
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
    if (ssf_mems_rx_slot_is_used(&data->frame[i]))
      ssf_mems_rx_process_candidate(data, &data->frame[i], byte);
  }
  ssf_mems_rx_create_candidate(data, byte);
}

/* 返回 count 全部入队（count 为 0 返回 0），-EINVAL 空设备/数据，-ENODEV 无驱动数据，-ENOSPC 部分入队或 FIFO 已满。 */
int ssf_mems_rx_push(struct serdev_device *serdev, const unsigned char *buf, size_t count) {
  struct ssf_mems_xyzs_data *data;
  unsigned int ret;

  if (!count)
    return 0;
  if (!serdev || !buf)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;
  ret = kfifo_in_spinlocked(&data->rx_fifo, buf, count, &data->rx_fifo_lock);
  ssf_mems_modbus_queue_parse(serdev);
  if (ret != count) {
    dev_err(&serdev->dev, "rx fifo overflow: received %zu bytes, stored %u bytes\n", count, ret);
    return -ENOSPC;
  }
  return ret;
}

/* 返回 0 FIFO 消费完成，-EINVAL 空设备，-ENODEV 无驱动数据；单帧错误由内部处理或记录日志。 */
int ssf_mems_modbus_parse_frame(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data;
  u8 byte;

  if (!serdev)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;
  while (kfifo_out_spinlocked(&data->rx_fifo, &byte, sizeof(byte),
                               &data->rx_fifo_lock) == sizeof(byte))
    ssf_mems_rx_process_byte(data, byte);
  return 0;
}
