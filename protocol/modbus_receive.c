#include "modbus_receive.h"
#include "core.h"
#include "modbus_request.h"
#include "protocol.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/slab.h>

#define SSF_MEMS_MODBUS_SLAVE_ID_LEN 1
#define SSF_MEMS_MODBUS_FUNC_LEN 1
#define SSF_MEMS_MODBUS_BYTE_COUNT_LEN 1
#define SSF_MEMS_MODBUS_CRC_LEN 2

#define SSF_MEMS_MODBUS_READ_FIXED_LEN                                         \
  (SSF_MEMS_MODBUS_SLAVE_ID_LEN + SSF_MEMS_MODBUS_FUNC_LEN +                   \
   SSF_MEMS_MODBUS_BYTE_COUNT_LEN + SSF_MEMS_MODBUS_CRC_LEN)

/*
 * Modbus 0x10 响应：
 *
 * Slave ID
 * Function
 * Starting Address : 2 bytes
 * Register Quantity: 2 bytes
 * CRC              : 2 bytes
 *
 * 总长度 = 8 bytes
 */
#define SSF_MEMS_MODBUS_WRITE_FIXED_FRAME_LEN 8

static const struct ssf_mems_frame_desc ssf_mems_frames[] = {
    {
        .type = SSF_MEMS_FRAME_MODBUS,
        .slave_id = 0x01,
    },
    {
        .type = SSF_MEMS_FRAME_RAW_VIB,
        .slave_id = 0x15,
    },
};

static size_t ssf_mems_modbus_read_frame_len(u8 byte_count) {
  return SSF_MEMS_MODBUS_READ_FIXED_LEN + byte_count;
}

static bool ssf_mems_rx_slot_is_used(const struct ssf_mems_frame_slot *slot) {
  return atomic_read(&slot->in_use) == SSF_MEMS_FRAME_SLOT_USED;
}

static struct ssf_mems_frame_slot *
ssf_mems_rx_alloc_slot(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_frame_slot *slot;
  int index;

  for (index = 0; index < SSF_MEMS_FRAME_SLOT_NUM; index++) {

    slot = &data->frame[index];

    if (atomic_cmpxchg(&slot->in_use, SSF_MEMS_FRAME_SLOT_FREE,
                       SSF_MEMS_FRAME_SLOT_USED) == SSF_MEMS_FRAME_SLOT_FREE) {

      return slot;
    }
  }

  return NULL;
}

static void ssf_mems_rx_free_slot(struct ssf_mems_frame_slot *slot) {
  kfree(slot->data);

  slot->data = NULL;
  slot->data_len = 0;
  slot->data_pos = 0;
  slot->frame_len = 0;

  slot->state = SSF_MEMS_RX_IDLE;

  slot->slave_id = 0;
  slot->function = 0;

  atomic_set(&slot->in_use, SSF_MEMS_FRAME_SLOT_FREE);
}

static void ssf_mems_rx_init_candidate(struct ssf_mems_frame_slot *slot,
                                       u8 slave_id) {
  slot->data = NULL;
  slot->data_len = 0;
  slot->data_pos = 0;
  slot->frame_len = 0;

  slot->state = SSF_MEMS_RX_FUNC;

  slot->slave_id = slave_id;
  slot->function = 0;
}

static int ssf_mems_rx_append_byte(struct ssf_mems_frame_slot *slot, u8 byte) {
  if (!slot->data)
    return -EINVAL;

  if (slot->data_pos >= slot->data_len)
    return -ENOSPC;

  slot->data[slot->data_pos] = byte;
  slot->data_pos++;

  return 0;
}

static int ssf_mems_rx_alloc_read_frame(struct ssf_mems_frame_slot *slot,
                                        u8 byte_count) {
  size_t frame_len;

  frame_len = ssf_mems_modbus_read_frame_len(byte_count);

  slot->data = kzalloc(frame_len, GFP_KERNEL);
  if (!slot->data)
    return -ENOMEM;

  slot->data_len = frame_len;
  slot->frame_len = frame_len;
  slot->data_pos = 0;

  return 0;
}

static int
ssf_mems_rx_alloc_write_fixed_frame(struct ssf_mems_frame_slot *slot) {
  slot->data = kzalloc(SSF_MEMS_MODBUS_WRITE_FIXED_FRAME_LEN, GFP_KERNEL);

  if (!slot->data)
    return -ENOMEM;

  slot->data_len = SSF_MEMS_MODBUS_WRITE_FIXED_FRAME_LEN;

  slot->frame_len = SSF_MEMS_MODBUS_WRITE_FIXED_FRAME_LEN;

  slot->data_pos = 0;

  return 0;
}

static int ssf_mems_rx_set_function(struct ssf_mems_frame_slot *slot,
                                    u8 function) {
  int ret;

  if (function != SSF_MEMS_MODBUS_FUNC_READ &&
      function != SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE &&
      function != SSF_MEMS_MODBUS_FUNC_WRITE_MULTI) {
    return -EINVAL;
  }

  slot->function = function;

  if (function == SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE ||
      function == SSF_MEMS_MODBUS_FUNC_WRITE_MULTI) {
    ret = ssf_mems_rx_alloc_write_fixed_frame(slot);
    if (ret)
      return ret;

    /* 此时 Slave ID 和 Function 已经确定，直接写入 frame buffer。 */
    ret = ssf_mems_rx_append_byte(slot, slot->slave_id);
    if (ret)
      return ret;

    ret = ssf_mems_rx_append_byte(slot, slot->function);
    if (ret)
      return ret;
  }

  /* 0x03 此时还不知道 Byte Count，所以暂时不能创建完整 frame buffer。 */
  slot->state = SSF_MEMS_RX_DATA;

  return 0;
}

static int ssf_mems_rx_start_read_frame(struct ssf_mems_frame_slot *slot,
                                        u8 byte_count) {
  int ret;

  ret = ssf_mems_rx_alloc_read_frame(slot, byte_count);
  if (ret)
    return ret;

  ret = ssf_mems_rx_append_byte(slot, slot->slave_id);
  if (ret)
    return ret;

  ret = ssf_mems_rx_append_byte(slot, slot->function);
  if (ret)
    return ret;

  ret = ssf_mems_rx_append_byte(slot, byte_count);
  if (ret)
    return ret;

  return 0;
}

static u16 ssf_mems_modbus_crc16(const u8 *buf, size_t len) {
  u16 crc = 0xFFFF;
  while (len--) {
    int i;
    crc ^= *buf++;
    for (i = 0; i < 8; i++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

static void ssf_mems_rx_complete(struct ssf_mems_xyzs_data *data,
                                 struct ssf_mems_frame_slot *slot) {
  u16 crc_calc; // 自己算的crc
  u16 crc_recv; // 接收到的crc
  int ret = 0;

  if (!slot->data)
    return;

  if (slot->data_pos != slot->frame_len) {
    dev_err(&data->serdev->dev, "incomplete frame: pos=%zu len=%zu\n",
            slot->data_pos, slot->frame_len);
    return;
  }

  if (slot->data_len < 4) {
    dev_err(&data->serdev->dev, "invalid frame length: %zu\n", slot->data_len);
    return;
  }

  crc_calc = ssf_mems_modbus_crc16(slot->data, slot->data_len - 2);

  /* Modbus RTU CRC 先传输低8位 */
  crc_recv = slot->data[slot->data_len - 2] |
             ((u16)slot->data[slot->data_len - 1] << 8);

  if (crc_calc != crc_recv) {
    ssf_mems_rx_free_slot(slot);
    dev_err(&data->serdev->dev, "CRC error: calc=0x%04x recv=0x%04x\n",
            crc_calc, crc_recv);
    return;
  }

  /* 校验通过，将帧发送给协议处理模块 */
  if (ssf_mems_modbus_claim_frame(data, slot)) {
    ssf_mems_rx_free_slot(slot);
    return;
  }

  ret = ssf_mems_protocol_process(data, slot);
  if (ret < 0) {
    dev_err(&data->serdev->dev, "failed to ssf_mems_protocol_process: %d\n",
            ret);
  }
}

static void ssf_mems_rx_process_candidate(struct ssf_mems_xyzs_data *data,
                                          struct ssf_mems_frame_slot *slot,
                                          u8 byte) {
  int ret;

  switch (slot->state) {
  case SSF_MEMS_RX_FUNC:

    ret = ssf_mems_rx_set_function(slot, byte);

    if (ret) {
      ssf_mems_rx_free_slot(slot);
      return;
    }

    break;

  case SSF_MEMS_RX_DATA:

    /*
     * 0x03：
     *
     * 当前 candidate 还没有 data buffer，
     * 说明当前 byte 是 Byte Count。
     */
    if (slot->function == SSF_MEMS_MODBUS_FUNC_READ && !slot->data) {

      ret = ssf_mems_rx_start_read_frame(slot, byte);

      if (ret) {
        ssf_mems_rx_free_slot(slot);
        return;
      }

      break;
    }

    /*
     * 0x03：
     * 接收 Data。
     *
     * 0x10：
     * 接收 Starting Address 和
     * Register Quantity。
     */
    ret = ssf_mems_rx_append_byte(slot, byte);

    if (ret) {
      ssf_mems_rx_free_slot(slot);
      return;
    }

    /*
     * frame_len 已经包含 CRC。
     *
     * 剩下两个字节就是 CRC。
     */
    if (slot->data_pos == slot->frame_len - SSF_MEMS_MODBUS_CRC_LEN) {

      slot->state = SSF_MEMS_RX_CRC;
    }

    break;

  case SSF_MEMS_RX_CRC:

    ret = ssf_mems_rx_append_byte(slot, byte);

    if (ret) {
      ssf_mems_rx_free_slot(slot);
      return;
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

static void ssf_mems_rx_create_candidate(struct ssf_mems_xyzs_data *data,
                                         u8 byte) {
  struct ssf_mems_frame_slot *slot;

  /*
   * 当前只识别 Modbus Slave ID。
   *
   * 0x15 的私有协议以后单独增加。
   */
  if (byte != ssf_mems_frames[SSF_MEMS_FRAME_MODBUS].slave_id) {
    return;
  }

  slot = ssf_mems_rx_alloc_slot(data);

  if (!slot) {
    dev_warn(&data->serdev->dev, "no free frame slot for new candidate\n");
    return;
  }

  ssf_mems_rx_init_candidate(slot, byte);
}

static void ssf_mems_rx_process_byte(struct ssf_mems_xyzs_data *data, u8 byte) {
  struct ssf_mems_frame_slot *slot;
  int index;

  /*
   * 第一阶段：
   *
   * 让所有已经存在的 candidate
   * 消费当前字节。
   */
  for (index = 0; index < SSF_MEMS_FRAME_SLOT_NUM; index++) {

    slot = &data->frame[index];

    if (!ssf_mems_rx_slot_is_used(slot))
      continue;

    ssf_mems_rx_process_candidate(data, slot, byte);
  }

  /*
   * 第二阶段：
   *
   * 当前字节还可能是一个新的
   * Slave ID。
   *
   * 因此建立新的 candidate。
   *
   * 注意：
   * 必须在已有 candidate 消费完之后
   * 再创建。
   */
  ssf_mems_rx_create_candidate(data, byte);
}

int ssf_mems_rx_push(struct serdev_device *serdev, const unsigned char *buf,
                     size_t count) {
  struct ssf_mems_xyzs_data *data;
  unsigned int ret;

  data = serdev_device_get_drvdata(serdev);

  if (!count)
    return 0;

  ret = kfifo_in_spinlocked(&data->rx_fifo, buf, count, &data->rx_fifo_lock);

  ssf_mems_modbus_queue_parse(serdev);
  if (ret != count) {
    dev_err(&serdev->dev,
            "rx fifo overflow: "
            "received %zu bytes, "
            "stored %u bytes\n",
            count, ret);
    return -ENOSPC;
  }

  return ret;
}

int ssf_mems_modbus_parse_frame(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data;
  u8 recv_data;
  unsigned int ret;

  data = serdev_device_get_drvdata(serdev);

  while (1) {
    ret = kfifo_out_spinlocked(&data->rx_fifo, &recv_data, sizeof(recv_data),
                               &data->rx_fifo_lock);

    if (ret != sizeof(recv_data))
      break;

    ssf_mems_rx_process_byte(data, recv_data);
  }

  return 0;
}