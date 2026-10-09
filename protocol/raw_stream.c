// SPDX-License-Identifier: GPL-2.0-only
#include "raw_stream.h"

#include "core.h"
#include "modbus.h"
#include "ssf_mems_iio.h"

#include <linux/kernel.h>
#include <linux/string.h>

/* 初始化等待队列和计数器，后续启停由协议层在 bus_lock 下负责。 */
void ssf_mems_raw_init(struct ssf_mems_xyzs_data *data) {
  memset(&data->raw, 0, sizeof(data->raw));
  init_waitqueue_head(&data->raw.waitq);
}

/* 按固件格式拆出 64 组 XYZ；不转换成浮点数、不调换固件已经映射的 Y/Z。 */
static void ssf_mems_raw_publish_packet(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_raw_state *raw = &data->raw;
  u32 sequence = ((u32)raw->packet[1] << 24) |
                 ((u32)raw->packet[2] << 16) |
                 ((u32)raw->packet[3] << 8) | raw->packet[4];
  unsigned int i;

  WRITE_ONCE(raw->stream_seen, true);
  if (raw->have_sequence == true && sequence != raw->next_sequence) {
    if (raw->publishing == true) {
      WRITE_ONCE(raw->discontinuities, raw->discontinuities + 1U);
      dev_warn_ratelimited(&data->serdev->dev,
                        "raw sequence gap: expected %u, received %u\n",
                        raw->next_sequence, sequence);
    } else {
      WRITE_ONCE(raw->drain_discontinuities, raw->drain_discontinuities + 1U);
    }
  }
  raw->have_sequence = true;
  raw->next_sequence = sequence + 1U;
  for (i = 0; i < SSF_MEMS_RAW_SAMPLES_PER_PACKET; i++) {
    struct ssf_mems_raw_sample sample;
    unsigned int axis;
    size_t offset = 5U + i * 6U;
    int ret;

    if (raw->publishing == false)
      break;

    for (axis = 0; axis < 3U; axis++)
      sample.xyz[axis] = (s16)(((u16)raw->packet[offset + axis * 2U] << 8) |
                               raw->packet[offset + axis * 2U + 1U]);
    sample.packet_sequence = sequence;
    sample.sample_index = i;
    ret = ssf_mems_iio_publish_raw(data, &sample);
    if (ret != 0)
      WRITE_ONCE(raw->buffer_errors, raw->buffer_errors + 1U);
  }
  if (raw->publishing == true)
    WRITE_ONCE(raw->packets, raw->packets + 1U);
  else
    WRITE_ONCE(raw->drain_packets, raw->drain_packets + 1U);
  wake_up_interruptible(&raw->waitq);
}

/* 私有包是 0x15 + BE32 包号 + 384 字节 XYZ + CRC16 + 0x17；没有从站地址。
 * 损坏后逐字节滑动找下一个通过 CRC 的包，兼容拆包、粘包和任意二进制载荷。 */
void ssf_mems_raw_receive_byte(struct ssf_mems_xyzs_data *data, u8 byte) {
  struct ssf_mems_raw_state *raw = &data->raw;

  if (raw->used == 0 && byte != 0x15)
    return;
  raw->packet[raw->used++] = byte;
  if (raw->used != SSF_MEMS_RAW_PACKET_BYTES)
    return;
  if (raw->packet[391] == 0x17 &&
      ssf_mems_modbus_check_crc(raw->packet, 391) == 0) {
    ssf_mems_raw_publish_packet(data);
    raw->used = 0;
    return;
  }
  if (raw->publishing == true)
    WRITE_ONCE(raw->crc_errors, raw->crc_errors + 1U);
  else
    WRITE_ONCE(raw->drain_crc_errors, raw->drain_crc_errors + 1U);
  do {
    raw->used--;
    memmove(raw->packet, raw->packet + 1, raw->used);
  } while (raw->used != 0 && raw->packet[0] != 0x15);
}
