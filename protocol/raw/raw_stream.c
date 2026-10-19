// SPDX-License-Identifier: GPL-2.0-only
#include "raw/raw_stream.h"

#include "core.h"
#include "modbus/modbus.h"

#include <asm/unaligned.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

/* 初始化定长原始数据接收状态。
 * 返回：无。
 */
void ssf_mems_raw_init(struct ssf_mems_data *data) {
  memset(&data->raw, 0, sizeof(data->raw));
  init_waitqueue_head(&data->raw.waitq);
}

/* 解析一包 XYZ 样本并交给已绑定的样本接收函数。
 * 返回：无。
 * 序号和推送错误记入统计，末包标记批次结束。
 */
static void ssf_mems_raw_publish_packet(struct ssf_mems_data *data) {
  struct ssf_mems_raw_state *raw = &data->raw;
  u32 sequence = get_unaligned_be32(raw->packet + SSF_MEMS_RAW_SEQUENCE_OFFSET);
  unsigned int i;

  if (sequence != raw->next_sequence || sequence >= SSF_MEMS_RAW_BATCH_PACKETS ||
      raw->batch_done == true) {
    if (raw->publishing == true) {
      WRITE_ONCE(raw->discontinuities, raw->discontinuities + 1U);
      dev_warn_ratelimited(&data->serdev->dev,
                        "raw sequence gap: expected %u, received %u\n",
                        raw->next_sequence, sequence);
    } else {
      WRITE_ONCE(raw->drain_discontinuities, raw->drain_discontinuities + 1U);
    }
  }
  raw->next_sequence = sequence + 1U;
  WRITE_ONCE(raw->batch_packets, raw->batch_packets + 1U);
  for (i = 0; i < SSF_MEMS_RAW_SAMPLES_PER_PACKET; i++) {
    struct ssf_mems_raw_sample sample;
    unsigned int axis;
    size_t offset = SSF_MEMS_RAW_PAYLOAD_OFFSET + i * SSF_MEMS_RAW_SAMPLE_BYTES;
    int ret;

    if (raw->publishing == false || sequence >= SSF_MEMS_RAW_BATCH_PACKETS ||
        raw->batch_done == true)
      break;

    /* 每组六字节按 X/Y/Z 的大端有符号 16 位值解码。 */
    for (axis = 0; axis < 3U; axis++)
      sample.xyz[axis] = (s16)get_unaligned_be16(raw->packet + offset +
          data->sensor->profile.raw_axis_order[axis] * sizeof(s16));
    sample.packet_sequence = sequence;
    sample.sample_index = i;
    ret = raw->sample_handler ? raw->sample_handler(data, &sample) : -ENODEV;
    if (ret != 0)
      WRITE_ONCE(raw->buffer_errors, raw->buffer_errors + 1U);
  }
  if (raw->publishing == true)
    WRITE_ONCE(raw->packets, raw->packets + 1U);
  else
    WRITE_ONCE(raw->drain_packets, raw->drain_packets + 1U);
  if (sequence == SSF_MEMS_RAW_BATCH_PACKETS - 1U)
    WRITE_ONCE(raw->batch_done, true);
  wake_up_interruptible(&raw->waitq);
}

/* 用一个字节推进定长原始数据拼包。
 * 返回：无。
 * 调用者持 parse_lock；校验失败后定位下一包头。
 */
void ssf_mems_raw_receive_byte(struct ssf_mems_data *data, u8 byte) {
  struct ssf_mems_raw_state *raw = &data->raw;
  const u8 *next;

  if (raw->used == 0 && byte != SSF_MEMS_RAW_HEADER) {
    WRITE_ONCE(raw->ignored_bytes, raw->ignored_bytes + 1U);
    return;
  }
  raw->packet[raw->used++] = byte;
  if (raw->used != SSF_MEMS_RAW_PACKET_BYTES)
    return;
  if (raw->packet[SSF_MEMS_RAW_TAIL_OFFSET] == SSF_MEMS_RAW_TAIL &&
      ssf_mems_modbus_check_crc(raw->packet, SSF_MEMS_RAW_TAIL_OFFSET) == 0) {
    ssf_mems_raw_publish_packet(data);
    raw->used = 0;
    return;
  }
  if (raw->publishing == true)
    WRITE_ONCE(raw->crc_errors, raw->crc_errors + 1U);
  else
    WRITE_ONCE(raw->drain_crc_errors, raw->drain_crc_errors + 1U);
  next = memchr(raw->packet + 1, SSF_MEMS_RAW_HEADER, raw->used - 1U);
  raw->used = next ? raw->used - (next - raw->packet) : 0;
  if (raw->used != 0)
    memmove(raw->packet, next, raw->used);
}
