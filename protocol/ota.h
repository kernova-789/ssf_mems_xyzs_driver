/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

struct ssf_mems_xyzs_data;

#define SSF_MEMS_OTA_BLOCK_SIZE 256U /* 一包写入的固定载荷字节数。 */
#define SSF_MEMS_OTA_FRAME_SIZE 263U /* 6 字节头 + 256 字节载荷 + 1 字节校验。 */
#define SSF_MEMS_OTA_APP_ADDRESS 0x08020000U /* Bootloader 的应用区起点。 */
#define SSF_MEMS_OTA_MAX_SIZE (256U * 1024U) /* 应用区/暂存区容量，字节。 */
#define SSF_MEMS_OTA_NAME_SIZE 128U /* 文件名缓冲区容量，包含结尾 NUL。 */

/* complete 表示传输应答及重启后通信已确认，不代表整镜像读回校验。 */
enum ssf_mems_ota_stage {
  SSF_MEMS_OTA_IDLE,        /* 尚未执行升级。 */
  SSF_MEMS_OTA_LOADING,     /* 正在从 firmware 目录读取 bin。 */
  SSF_MEMS_OTA_WAITING,     /* 等待当前采集窗口结束。 */
  SSF_MEMS_OTA_CHECKING,    /* 确认普通通信和固件包计数。 */
  SSF_MEMS_OTA_SENDING,     /* 逐包写入传感器暂存区。 */
  SSF_MEMS_OTA_REBOOTING,   /* 末包已尝试发送，等待复制和重启。 */
  SSF_MEMS_OTA_VERIFYING,   /* 只读确认重启后的通信和版本号。 */
  SSF_MEMS_OTA_COMPLETE,   /* 所有包获得应答，重启后可通信。 */
  SSF_MEMS_OTA_FAILED,     /* 末包之前失败，未主动提交升级。 */
  SSF_MEMS_OTA_UNCERTAIN,  /* 末包可能已生效，但不能完整确认结果。 */
};

/* 状态快照可在升级期间读取，不获取采集锁或发送串口数据。 */
struct ssf_mems_ota_status {
  enum ssf_mems_ota_stage stage; /* 当前阶段。 */
  size_t acknowledged;          /* 已收到对应包应答的原始文件字节数。 */
  size_t total;                 /* bin 原始字节数，不含尾包填充。 */
  int error;                    /* 最近一次操作的负 errno，成功为 0。 */
  int version;                  /* 重启后读到的版本号；-1 表示尚未读到。 */
};

/* 每设备一个 OTA 上下文；锁顺序为 update_lock -> 采集锁 -> bus_lock。
 * 接收工作项仅获取 lock，绝不能获取上述三个长事务锁。 */
struct ssf_mems_ota_state {
  struct mutex update_lock;     /* 串行化整个文件加载和升级操作。 */
  struct mutex lock;            /* 保护状态快照、应答拼包和 pending。 */
  wait_queue_head_t waitq;      /* 等待应答及卸载通知。 */
  struct ssf_mems_ota_status status; /* 最近一次升级状态。 */
  bool stopping;               /* 卸载后拒绝新升级并中断等待。 */
  bool pending;                /* 当前有一个包等待应答。 */
  u8 reply[6];                 /* 固件回复：地址、0x41、下一包号，无 CRC。 */
  size_t used;                 /* 已拼接的应答字节数。 */
  u32 next_packet;             /* 当前应答报告的下一包号。 */
};

/* 初始化固件升级状态。
 * 返回：无。
 */
void ssf_mems_ota_init(struct ssf_mems_xyzs_data *data);
/* 禁止新升级并唤醒升级等待者。
 * 返回：无。
 */
void ssf_mems_ota_shutdown(struct ssf_mems_xyzs_data *data);
/* 检查固件大小和启动向量是否有效。
 * 返回：0 通过，-EINVAL 数据无效，-EFBIG 过大，-ENOEXEC 启动向量不符。
 */
int ssf_mems_ota_validate_image(const u8 *image, size_t size);
/* 检查并复制 bin 文件名。
 * 返回：0 成功，-EINVAL 文件名或参数无效。
 * name 缓冲区须有 SSF_MEMS_OTA_NAME_SIZE 字节。
 */
int ssf_mems_ota_parse_name(const char *buf, size_t len, char *name);
/* 组装一个固件升级数据包。
 * 返回：0 成功，-EINVAL 参数无效。
 * frame 须有 SSF_MEMS_OTA_FRAME_SIZE 字节，不足一块填 0xff。
 */
int ssf_mems_ota_build_packet(u8 address, u32 sequence, const u8 *payload,
                              size_t size, u8 *frame);
/* 用一个字节拼接固件升级应答。
 * 返回：无。
 * 收齐应答后唤醒发送者。
 */
void ssf_mems_ota_receive_byte(struct ssf_mems_xyzs_data *data, u8 byte);
/* 传输固件并确认传感器重启后的通信。
 * 返回：0 升级应答及通信已确认，负数为镜像、状态或传输错误。
 * 调用者持 update_lock 和采集锁；末包不重发，结果不明时标记 uncertain。
 */
int ssf_mems_ota_transfer(struct ssf_mems_xyzs_data *data,
                          const u8 *image, size_t size);
/* 更新升级阶段和错误码。
 * 返回：无。
 * 开始加载时清空旧进度。
 */
void ssf_mems_ota_set_stage(struct ssf_mems_xyzs_data *data,
                            enum ssf_mems_ota_stage stage, int error);
/* 复制当前固件升级进度。
 * 返回：无。
 * 只持升级状态锁，升级期间也可读取。
 */
void ssf_mems_ota_get_status(struct ssf_mems_xyzs_data *data,
                             struct ssf_mems_ota_status *status);
/* 把升级阶段转换为名称。
 * 返回：静态名称字符串，无效阶段为 "unknown"。
 */
const char *ssf_mems_ota_stage_name(enum ssf_mems_ota_stage stage);
