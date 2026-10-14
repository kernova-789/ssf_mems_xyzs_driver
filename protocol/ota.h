/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

struct ssf_mems_xyzs_data;

#define SSF_MEMS_OTA_BLOCK_SIZE 256U
#define SSF_MEMS_OTA_FRAME_SIZE 263U
#define SSF_MEMS_OTA_APP_ADDRESS 0x08020000U
#define SSF_MEMS_OTA_MAX_SIZE (256U * 1024U)
#define SSF_MEMS_OTA_NAME_SIZE 128U

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

/* 初始化/关闭仅改变主机状态，不发送升级或重启命令。 */
void ssf_mems_ota_init(struct ssf_mems_xyzs_data *data);
void ssf_mems_ota_shutdown(struct ssf_mems_xyzs_data *data);
/* 检查应用区大小、栈指针和 Thumb 复位入口；不证明机型或签名匹配。 */
int ssf_mems_ota_validate_image(const u8 *image, size_t size);
/* 只允许 firmware 目录下的简单 .bin 文件名，不接受路径或嵌入 NUL。 */
int ssf_mems_ota_parse_name(const char *buf, size_t len, char *name);
/* 组包时尾块填充 0xff；最后一个有数据的包使用 0xffffffff 包号。 */
int ssf_mems_ota_build_packet(u8 address, u32 sequence, const u8 *payload,
                              size_t size, u8 *frame);
/* 仅在 OTA 模式由接收工作项调用，可处理碎片和连续应答。 */
void ssf_mems_ota_receive_byte(struct ssf_mems_xyzs_data *data, u8 byte);
/* 调用者持 update_lock 和采集锁；函数独占 bus_lock，失败不自动提交或重启。 */
int ssf_mems_ota_transfer(struct ssf_mems_xyzs_data *data,
                          const u8 *image, size_t size);
/* 短锁读写状态，供采集入口和 IIO 展示使用。 */
void ssf_mems_ota_set_stage(struct ssf_mems_xyzs_data *data,
                            enum ssf_mems_ota_stage stage, int error);
void ssf_mems_ota_get_status(struct ssf_mems_xyzs_data *data,
                             struct ssf_mems_ota_status *status);
const char *ssf_mems_ota_stage_name(enum ssf_mems_ota_stage stage);
