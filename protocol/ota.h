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

/* 初始化 OTA 状态；无返回值，data 必须有效，不发送串口命令。 */
void ssf_mems_ota_init(struct ssf_mems_xyzs_data *data);
/* 禁止新升级并唤醒等待者；无返回值，不发送重启命令。 */
void ssf_mems_ota_shutdown(struct ssf_mems_xyzs_data *data);
/* 检查应用区大小、栈指针和 Thumb 复位入口；不证明机型或签名匹配。
 * 返回 0 基础校验通过，-EINVAL 空数据/不足 8 字节，-EFBIG 超过 256 KiB，
 * -ENOEXEC 栈指针/复位入口不满足应用区要求。 */
int ssf_mems_ota_validate_image(const u8 *image, size_t size);
/* 解析简单 .bin 文件名；返回 0 成功，-EINVAL 空参数、路径、非法字符/长度/后缀。
 * name 必须提供 SSF_MEMS_OTA_NAME_SIZE 字节空间。 */
int ssf_mems_ota_parse_name(const char *buf, size_t len, char *name);
/* 组包时尾块填充 0xff；最后一个有数据的包使用 0xffffffff 包号。
 * 返回 0 成功，-EINVAL 地址非法、载荷超长或缓冲区指针为空。
 * frame 必须提供 SSF_MEMS_OTA_FRAME_SIZE 字节空间。 */
int ssf_mems_ota_build_packet(u8 address, u32 sequence, const u8 *payload,
                              size_t size, u8 *frame);
/* 接收工作项处理一个 OTA 字节；无返回值，收齐应答后唤醒发送者。 */
void ssf_mems_ota_receive_byte(struct ssf_mems_xyzs_data *data, u8 byte);
/* 调用者持 update_lock 和采集锁，函数独占 bus_lock；失败不自动重启。
 * 返回 0 分包应答及重启后通信已确认，不代表整镜像读回验证；
 * -EINVAL 参数非法，-EFBIG 镜像过大，-ENOEXEC 向量表不符，-EBUSY 模式/速率不允许，
 * -ENODEV 停止中，-EPROTO 包计数不符，-EIO 串口短写，-ETIMEDOUT 应答/启动验证超时，
 * -EINTR 被信号打断；其他为版本读取请求错误（见 modbus_request.h）或串口负 errno。 */
int ssf_mems_ota_transfer(struct ssf_mems_xyzs_data *data,
                          const u8 *image, size_t size);
/* 更新阶段/错误码，开始加载时清旧进度；无返回值，只获取短锁。 */
void ssf_mems_ota_set_stage(struct ssf_mems_xyzs_data *data,
                            enum ssf_mems_ota_stage stage, int error);
/* 复制升级进度快照；无返回值，data/status 必须有效，不获取采集锁。 */
void ssf_mems_ota_get_status(struct ssf_mems_xyzs_data *data,
                             struct ssf_mems_ota_status *status);
/* 返回静态阶段名称字符串，非法枚举返回 "unknown"，不会返回 NULL。 */
const char *ssf_mems_ota_stage_name(enum ssf_mems_ota_stage stage);
