// SPDX-License-Identifier: GPL-2.0-only
#include "raw/sound.h"
#include "core.h"
#include "modbus/modbus.h"
#include <asm/unaligned.h>
#include <linux/errno.h>
#include <linux/string.h>

void ssf_mems_sound_init(struct ssf_mems_data *data) {
  memset(&data->sound, 0, sizeof(data->sound));
  init_waitqueue_head(&data->sound.waitq);
}

/* 校验从机、64 包总数、包号及保留字段；不发布未验收的声音片段。 */
static void ssf_mems_sound_packet(struct ssf_mems_data *data) {
  struct ssf_mems_sound_state *s = &data->sound;
  const u8 *p = s->packet;
  u16 sequence = get_unaligned_be16(p + 5);
  unsigned int i;

  if (p[2] != 0 || get_unaligned_be16(p + 3) != SSF_MEMS_SOUND_PACKETS ||
      p[7] != 0 || p[8] != 0 || sequence >= SSF_MEMS_SOUND_PACKETS ||
      sequence != s->packets || s->done) {
    s->sequence_gaps++;
  } else {
    for (i = 0; i < SSF_MEMS_SOUND_PACKET_SAMPLES; i++)
      s->samples[sequence * SSF_MEMS_SOUND_PACKET_SAMPLES + i] =
          (s32)get_unaligned_be32(p + 9 + i * sizeof(s32));
  }
  s->packets++;
  if (sequence == SSF_MEMS_SOUND_PACKETS - 1U)
    s->done = true;
}

void ssf_mems_sound_receive_byte(struct ssf_mems_data *data, u8 byte) {
  struct ssf_mems_sound_state *s = &data->sound;
  size_t length;

  if (s->used == 0 && byte != data->slave_id) {
    s->ignored_bytes++;
    return;
  }
  s->packet[s->used++] = byte;
  if (s->used < 2)
    return;
  switch (s->packet[1]) {
  case 0x03: length = SSF_MEMS_SOUND_PACKET_BYTES; break;
  case 0x06: length = 8; break;
  case 0x86: length = 5; break;
  default: goto resync;
  }
  if (s->used < length)
    return;
  if (ssf_mems_modbus_check_crc(s->packet, length) != 0) {
    s->crc_errors++;
    goto resync;
  }
  if (s->packet[1] == 0x03) {
    ssf_mems_sound_packet(data);
  } else if (s->packet[1] == 0x86) {
    s->error = -EREMOTEIO;
    s->done = true;
  } else if (get_unaligned_be16(s->packet + 2) == s->trigger_address &&
             get_unaligned_be16(s->packet + 4) == 1 && !s->acknowledged) {
    s->acknowledged = true;
  } else {
    s->sequence_gaps++;
  }
  s->used = 0;
  wake_up_interruptible(&s->waitq);
  return;
resync:
  /* 噪声中寻找下一从机字节，所有丢弃字节均计数。 */
  do {
    memmove(s->packet, s->packet + 1, --s->used);
    s->ignored_bytes++;
  } while (s->used && s->packet[0] != data->slave_id);
}
