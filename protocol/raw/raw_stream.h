/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/types.h>
#include <linux/wait.h>

struct ssf_mems_xyzs_data;
struct ssf_mems_raw_sample;

typedef int (*ssf_mems_raw_sample_handler_t)(
    struct ssf_mems_xyzs_data *data, const struct ssf_mems_raw_sample *sample);

#define SSF_MEMS_RAW_HEADER 0x15U
#define SSF_MEMS_RAW_TAIL 0x17U
#define SSF_MEMS_RAW_SEQUENCE_OFFSET 1U
#define SSF_MEMS_RAW_PAYLOAD_OFFSET 5U
#define SSF_MEMS_RAW_SAMPLE_BYTES (3U * sizeof(s16))
#define SSF_MEMS_RAW_SAMPLES_PER_PACKET 64U /* 一个私有包内同步 XYZ 样本数。 */
#define SSF_MEMS_RAW_CRC_OFFSET \
  (SSF_MEMS_RAW_PAYLOAD_OFFSET + SSF_MEMS_RAW_SAMPLES_PER_PACKET * SSF_MEMS_RAW_SAMPLE_BYTES)
#define SSF_MEMS_RAW_TAIL_OFFSET (SSF_MEMS_RAW_CRC_OFFSET + 2U)
#define SSF_MEMS_RAW_PACKET_BYTES (SSF_MEMS_RAW_TAIL_OFFSET + 1U)
#define SSF_MEMS_RAW_BATCH_SAMPLES 1024U /* 一批 XYZ 组数，对应长度参数为零时的默认批次。 */
#define SSF_MEMS_RAW_BATCH_PACKETS \
  (SSF_MEMS_RAW_BATCH_SAMPLES / SSF_MEMS_RAW_SAMPLES_PER_PACKET)

/* 定长包解析状态；字节缓冲和批次边界由接收模块的 parse_lock 保护。 */
struct ssf_mems_raw_state {
  u32 batch_packets; /* 当前批次 CRC 正确的包数，包含卸载时排空的包。 */
  bool batch_done; /* 是否收到本批末包；错误统计另行决定该批是否完整有效。 */
  u32 batches; /* 当前原始采集窗口中成功完成的批次数。 */
  u32 ignored_bytes; /* 未能组成私有包的字节数，不能静默忽略链路异常。 */
  bool active; /* 为 true 时接收字节交给私有流解析器，而非 Modbus 解析器。 */
  bool publishing; /* 卸载/异常排空时仍解析数据，但不再向 IIO 发布样本。 */
  ssf_mems_raw_sample_handler_t sample_handler; /* core 在采集启动前绑定，此后不修改。 */
  wait_queue_head_t waitq; /* 首包到达或驱动退出时唤醒采集线程。 */
  u8 packet[SSF_MEMS_RAW_PACKET_BYTES]; /* 正在拼接的固定长度数据包。 */
  size_t used; /* 缓冲区中已经接收的字节数。 */
  u32 next_sequence; /* 当前批次下一个期望包号，每批从零开始。 */
  u32 packets; /* 有效采集阶段通过 CRC 校验的包数，不包含停止排空。 */
  u32 discontinuities; /* 有效采集阶段的序号跳变次数。 */
  u32 crc_errors; /* 有效采集阶段的候选包 CRC/尾标记错误次数。 */
  u32 buffer_errors; /* 本轮 IIO 推送失败的样本数。 */
  u32 drain_packets; /* 停止/恢复期间通过 CRC 校验但不发布的包数。 */
  u32 drain_discontinuities; /* 停止/恢复期间的序号跳变次数。 */
  u32 drain_crc_errors; /* 停止/恢复期间的损坏候选包数。 */
  u32 start_wait_ms; /* 开始发送启动命令到采集线程确认首包的耗时。 */
  u16 sampling_rate_index; /* 本轮启动前读回的采样率索引，用于解释原始序列。 */
};

/* 初始化定长原始数据接收状态。
 * 返回：无。
 */
void ssf_mems_raw_init(struct ssf_mems_xyzs_data *data);

/* 用一个字节推进定长原始数据拼包。
 * 返回：无。
 * 调用者持 parse_lock；校验失败后定位下一包头。
 */
void ssf_mems_raw_receive_byte(struct ssf_mems_xyzs_data *data, u8 byte);
