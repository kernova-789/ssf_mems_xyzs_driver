/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "acquisition_policy.h"
#include "protocol.h"

struct task_struct;
struct ssf_mems_xyzs_data;

/* 采集策略；raw 是反复执行完整定长批次，不是固件的连续上传命令。 */
enum ssf_mems_acquisition_mode {
  SSF_MEMS_ACQ_MODE_RAW,         /* 只重复定长原始采集。 */
  SSF_MEMS_ACQ_MODE_FEATURE,     /* 只周期读取特征。 */
  SSF_MEMS_ACQ_MODE_ALTERNATING, /* 特征与定长原始按窗口交替。 */
  SSF_MEMS_ACQ_MODE_COUNT,       /* 模式数量，用于参数检查。 */
};

#define SSF_MEMS_ACQ_MIN_DURATION_MS 100U
#define SSF_MEMS_ACQ_MAX_DURATION_MS 60000U

/* 一次加锁读出的采集配置快照，不触发串口操作。 */
struct ssf_mems_acquisition_config {
  enum ssf_mems_acquisition_mode mode; /* 当前选择的采集策略。 */
  unsigned int raw_duration_ms;       /* 原始采集窗口；交替时决定原始阶段时长，单位 ms。 */
  unsigned int feature_duration_ms;   /* 交替模式的特征窗口，单位 ms。 */
};

/* 后台线程当前工作阶段；错误后进入退避，再重新连接。 */
enum ssf_mems_acquisition_phase {
  SSF_MEMS_ACQ_STARTUP, /* 刚加载，等待传感器启动。 */
  SSF_MEMS_ACQ_MATCH, /* 查找能够通信的波特率。 */
  SSF_MEMS_ACQ_SWITCH, /* 按保存/重启流程切到目标波特率。 */
  SSF_MEMS_ACQ_CONFIGURE, /* 设置并读回采样配置。 */
  SSF_MEMS_ACQ_VERIFY, /* 验证通信或首份完整特征。 */
  SSF_MEMS_ACQ_POLL, /* 已连接，按当前模式调度采集。 */
  SSF_MEMS_ACQ_RAW, /* 独占串口采集定长 XYZ 批次，完成窗口后回到特征轮询。 */
  SSF_MEMS_ACQ_BACKOFF, /* 连接失败，延迟后再尝试。 */
  SSF_MEMS_ACQ_STOPPED, /* 线程已退出。 */
};

/* 每设备的后台采集上下文；普通状态由 lock 保护，等待条件用 READ_ONCE 读取。 */
struct ssf_mems_acquisition_state {
  struct task_struct *thread; /* 采集内核线程，未启动/已退出时为 NULL。 */
  wait_queue_head_t waitq; /* 休眠、配置变化、重连和停止的唤醒队列。 */
  /* 串行化采集流程和 IIO 波特率操作；定时休眠期间不持锁。 */
  struct mutex lock;
  bool stopping; /* 正在卸载，不再开始新采集。 */
  bool reconnect; /* 下轮需要重新匹配并验证设备。 */
  bool online; /* 最近连接验证成功，允许采集和配置请求。 */
  bool firmware_blocked; /* 末包结果不确定时停止自动通信，等待人工处理/再次升级。 */
  bool have_sample; /* 是否已有可计算年龄的完整特征样本。 */
  enum ssf_mems_acquisition_phase phase; /* 当前连接/采集阶段。 */
  enum ssf_mems_baudrate target_baudrate; /* 启动配置/手动设置的目标，不被扫描覆盖。 */
  unsigned int retry_ms; /* 下次连接失败后的退避时长，ms。 */
  unsigned long last_sample; /* 最近成功获取特征的 jiffies。 */
  struct ssf_mems_acquisition_policy policy; /* 轮询周期和断线判定统计。 */
  enum ssf_mems_acquisition_mode mode; /* 调度使用的模式，不是串口协议模式。 */
  bool config_changed; /* 唤醒调度并从新的窗口起点应用配置，不重新配置传感器。 */
  unsigned int raw_duration_ms; /* 每轮定长批采集窗口；收完在途批次。 */
  unsigned int feature_duration_ms; /* 每轮特征轮询时长，不含原始模式退出时间。 */
  unsigned long feature_until; /* 本轮特征采集结束的 jiffies 时刻。 */
};

/* 一次加锁复制的只读状态，供 IIO 状态属性使用。 */
struct ssf_mems_acquisition_status {
  bool online; /* 当前是否经过连接验证。 */
  bool have_sample; /* sample_age_ms 是否有效。 */
  unsigned int interval_ms; /* 当前特征轮询周期，ms。 */
  unsigned int sample_age_ms; /* 最近完整特征的年龄，ms；无样本时为 0。 */
};

/* 初始化状态和设备树窗口，在注册 IIO 前调用；返回 0 成功，-EINVAL 设备为空。 */
int ssf_mems_acquisition_init(struct ssf_mems_xyzs_data *data);
/* 注册 IIO 后启动线程；返回 0 成功，-EINVAL 无设备/IIO，-ENODEV 正在停止，
 * -EBUSY 已有线程；其他负值来自 kthread_run（如 -ENOMEM 内存不足）。 */
int ssf_mems_acquisition_start(struct ssf_mems_xyzs_data *data);
/* 调用前关闭请求模块以唤醒在途请求；线程退出后仍清理未确认的私有流，
 * 无返回值；完成后才允许释放 IIO/接收资源，收尾错误只记录日志。 */
void ssf_mems_acquisition_stop(struct ssf_mems_xyzs_data *data);
/* 确认当前速率；返回 0 成功，-EINVAL 空指针，-ENODEV 停止中，-EBUSY OTA 暂停；
 * 其余负值及含义同 ssf_mems_protocol_get_baudrate()。 */
int ssf_mems_acquisition_get_baudrate(struct ssf_mems_xyzs_data *data,
                                    enum ssf_mems_baudrate *baudrate);
/* 同步改速并更新恢复目标；返回 0 成功，-EINVAL 设备/枚举非法，-ENODEV 停止中，
 * -EBUSY OTA 暂停；其余负值及含义同 ssf_mems_protocol_set_baudrate()。 */
int ssf_mems_acquisition_set_baudrate(struct ssf_mems_xyzs_data *data,
                                    enum ssf_mems_baudrate baudrate);
/* 复制主机采集状态；无返回值，data/status 必须有效，不发送串口请求。 */
void ssf_mems_acquisition_get_status(
    struct ssf_mems_xyzs_data *data, struct ssf_mems_acquisition_status *status);
/* 读取传感器配置；返回 0 成功，-EINVAL 空指针/配置项非法，-ENODEV 停止中，
 * -EBUSY OTA 暂停，-ENODATA 未上线；其余负值同 protocol_read_setting()。 */
int ssf_mems_acquisition_read_setting(struct ssf_mems_xyzs_data *data,
                                     enum ssf_mems_sensor_setting setting,
                                     u16 *value);
/* 写入配置；返回 0 成功，-EINVAL 参数非法，-ENODEV 停止中，-EBUSY OTA 暂停，
 * -ENODATA 未上线；其余负值同 protocol_write_setting()，失败也可能使旧缓存失效。 */
int ssf_mems_acquisition_write_setting(struct ssf_mems_xyzs_data *data,
                                      enum ssf_mems_sensor_setting setting,
                                      u16 value);
/* 与采集串行复制调度配置；返回 0 成功，-EINVAL 空指针，-ENODEV 停止中。
 * 读写都可能等待整个原始窗口完成，不截断在途批次。 */
int ssf_mems_acquisition_get_config(
    struct ssf_mems_xyzs_data *data, struct ssf_mems_acquisition_config *config);
/* 切换主机调度；返回 0 成功/同值，-EINVAL 模式非法，-ENODEV 停止中，
 * -EOPNOTSUPP 当前校验位不支持原始采集；不写传感器参数。 */
int ssf_mems_acquisition_set_mode(struct ssf_mems_xyzs_data *data,
                                  enum ssf_mems_acquisition_mode mode);
/* raw_window=true 配置原始窗口，否则配置特征窗口；范围 100..60000 ms。
 * 返回 0 成功/同值，-EINVAL 设备/时长非法，-ENODEV 停止中。 */
int ssf_mems_acquisition_set_duration(struct ssf_mems_xyzs_data *data,
                                      bool raw_window, unsigned int duration_ms);
/* 同步加载 /lib/firmware 下的 bin，等采集窗口结束后独占升级。
 * 成功后按原配置恢复采集；末包结果不确定时暂停自动串口操作。
 * 返回 0 升级确认完成；-EINVAL 文件名非法，-ENOENT 文件不存在，-ENOMEM 加载内存不足，
 * -EFBIG 镜像过大，-ENOEXEC 向量表不匹配，-EBUSY 升级占用/链路模式不允许，
 * -ENODATA 未上线，-ENODEV 停止中，-EINTR 被信号中断；其余含义见 ota_transfer()。
 * 其他负 errno 原样来自 firmware 加载器（如 -EACCES 文件访问受限、-EIO 读取失败）。 */
int ssf_mems_acquisition_update_firmware(struct ssf_mems_xyzs_data *data,
                                         const char *name);
