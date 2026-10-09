/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/types.h>
#include <linux/wait.h>

struct ssf_mems_xyzs_data;

#define SSF_MEMS_RAW_PACKET_BYTES 392U
#define SSF_MEMS_RAW_SAMPLES_PER_PACKET 64U

/* 连续流解析状态；字节缓冲和序号由接收模块的 parse_lock 保护。 */
struct ssf_mems_raw_state {
  bool active; /* 为 true 时接收字节交给私有流解析器，而非 Modbus 解析器。 */
  bool publishing; /* 停止/排空阶段仍解析数据，但不再向 IIO 发布样本。 */
  bool stream_seen; /* 本轮是否见过有效原始包；停止时也保留这个启动证据。 */
  wait_queue_head_t waitq; /* 首包到达或驱动退出时唤醒采集线程。 */
  u8 packet[SSF_MEMS_RAW_PACKET_BYTES]; /* 正在拼接的固定长度数据包。 */
  size_t used; /* 缓冲区中已经接收的字节数。 */
  bool have_sequence; /* 是否已有一个经过 CRC 校验的包序号。 */
  u32 next_sequence; /* 下一个期望包号，按 32 位无符号数回绕。 */
  u32 packets; /* 有效采集阶段通过 CRC 校验的包数，不包含停止排空。 */
  u32 discontinuities; /* 有效采集阶段的序号跳变次数。 */
  u32 crc_errors; /* 有效采集阶段的候选包 CRC/尾标记错误次数。 */
  u32 buffer_errors; /* 本轮 IIO 推送失败的样本数。 */
  u32 drain_packets; /* 停止/恢复期间通过 CRC 校验但不发布的包数。 */
  u32 drain_discontinuities; /* 停止/恢复期间的序号跳变次数。 */
  u32 drain_crc_errors; /* 停止/恢复期间的损坏候选包数。 */
  u32 start_wait_ms; /* 开始发送启动命令到采集线程确认首包的耗时。 */
  u32 stop_attempts; /* 本轮累计发送停止命令的次数，包含后续恢复重试。 */
  u16 sampling_rate_index; /* 本轮启动前读回的采样率索引，用于解释原始序列。 */
};

/* 初始化私有流状态；不发送数据，也不切换串口。 */
void ssf_mems_raw_init(struct ssf_mems_xyzs_data *data);

/* 消费一个字节；仅由接收工作项在持有 parse_lock 时调用。 */
void ssf_mems_raw_receive_byte(struct ssf_mems_xyzs_data *data, u8 byte);
