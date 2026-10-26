/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once
#include <linux/types.h>
#include <linux/wait.h>

struct ssf_mems_data;
#define SSF_MEMS_SOUND_SAMPLES 4096U
#define SSF_MEMS_SOUND_PACKET_SAMPLES 64U
#define SSF_MEMS_SOUND_PACKETS 64U
#define SSF_MEMS_SOUND_PACKET_BYTES 267U

/* 保留线上的完整 32 位声音字；无声压标定，不能套用 dB 特征的 scale。 */
struct ssf_mems_sound_sample {
  s32 pcm;
  u16 packet_sequence;
  u16 sample_index;
};

struct ssf_mems_sound_state {
  bool active; /* 接收格式切换；由 parse_lock 保护。 */
  bool running, acknowledged, done;
  u16 trigger_address;
  u32 packets, sequence_gaps, crc_errors, ignored_bytes, buffer_errors;
  u32 captures;
  int error;
  unsigned long deadline;
  size_t used;
  u8 packet[SSF_MEMS_SOUND_PACKET_BYTES];
  s32 samples[SSF_MEMS_SOUND_SAMPLES]; /* 先验收完整批次，再发布到 IIO。 */
  wait_queue_head_t waitq;
  int (*sample_handler)(struct ssf_mems_data *, const struct ssf_mems_sound_sample *);
};

void ssf_mems_sound_init(struct ssf_mems_data *data);
/* 调用者持 parse_lock；仅在声音接收格式启用后调用。 */
void ssf_mems_sound_receive_byte(struct ssf_mems_data *data, u8 byte);
/* 调用者持采集锁；一次触发只发送一次开启命令。 */
int ssf_mems_sound_collect_finite(struct ssf_mems_data *data);
/* 调用者持 bus_lock；等待在途上传自然结束，只读确认普通通信。 */
int ssf_mems_sound_finish_locked(struct ssf_mems_data *data);
