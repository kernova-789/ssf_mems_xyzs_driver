/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "acquisition_policy.h"
#include "protocol.h"

struct task_struct;
struct ssf_mems_xyzs_data;

/* 采集策略；raw 反复执行完整定长批次，每批结束后确认普通通信。 */
enum ssf_mems_acquisition_mode {
  SSF_MEMS_ACQ_MODE_RAW,         /* 只重复定长原始采集。 */
  SSF_MEMS_ACQ_MODE_FEATURE,     /* 只周期读取特征。 */
  SSF_MEMS_ACQ_MODE_ALTERNATING, /* 特征与定长原始按窗口交替。 */
  SSF_MEMS_ACQ_MODE_COUNT,       /* 模式数量，用于参数检查。 */
};

#define SSF_MEMS_ACQ_MIN_DURATION_MS 100U
#define SSF_MEMS_ACQ_MAX_DURATION_MS 60000U

/* 可单独修改的轮询配置项，避免多次写入互相覆盖。 */
enum ssf_mems_poll_setting {
  SSF_MEMS_POLL_SETTING_MODE,
  SSF_MEMS_POLL_SETTING_FIXED_INTERVAL,
  SSF_MEMS_POLL_SETTING_MIN_INTERVAL,
  SSF_MEMS_POLL_SETTING_MAX_INTERVAL,
};

/* 一次加锁读出的采集配置快照，不触发串口操作。 */
struct ssf_mems_acquisition_config {
  enum ssf_mems_acquisition_mode mode; /* 当前选择的采集策略。 */
  unsigned int raw_duration_ms;       /* 原始采集窗口；交替时决定原始阶段时长，单位 ms。 */
  unsigned int feature_duration_ms;   /* 交替模式的特征窗口，单位 ms。 */
  struct ssf_mems_poll_config poll;    /* 特征轮询模式及周期范围。 */
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
  struct mutex status_lock; /* 只保护在线、样本年龄和生效周期，不跨越串口等待。 */
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
  bool poll_changed; /* 唤醒轮询等待，不重新开始特征窗口。 */
  unsigned int raw_duration_ms; /* 每轮定长批采集窗口；收完在途批次。 */
  unsigned int feature_duration_ms; /* 每轮特征轮询时长，不含原始模式退出时间。 */
  unsigned long feature_until; /* 本轮特征采集结束的 jiffies 时刻。 */
};

/* 用短临界区复制的只读状态，供 IIO 状态属性使用。 */
struct ssf_mems_acquisition_status {
  bool online; /* 当前是否经过连接验证。 */
  bool have_sample; /* sample_age_ms 是否有效。 */
  unsigned int interval_ms; /* 当前特征轮询周期，ms。 */
  unsigned int sample_age_ms; /* 最近完整特征的年龄，ms；无样本时为 0。 */
};

/* 初始化采集状态和窗口配置。
 * 返回：0 成功，-EINVAL 设备无效。
 */
int ssf_mems_acquisition_init(struct ssf_mems_xyzs_data *data);
/* 启动后台采集线程。
 * 返回：0 成功，负数为状态或线程创建错误。
 * 在 IIO 注册后调用。
 */
int ssf_mems_acquisition_start(struct ssf_mems_xyzs_data *data);
/* 停止采集线程并收尾在途批次。
 * 返回：无。
 * 调用前关闭请求模块，完成后才能释放接收和 IIO 资源。
 */
void ssf_mems_acquisition_stop(struct ssf_mems_xyzs_data *data);
/* 确认当前通信波特率。
 * 返回：0 成功，负数为参数、停止、升级占用或通信错误。
 * 与后台采集共用采集锁。
 */
int ssf_mems_acquisition_get_baudrate(struct ssf_mems_xyzs_data *data,
                                    enum ssf_mems_baudrate *baudrate);
/* 修改波特率并更新重连目标。
 * 返回：0 成功，负数为参数、停止、升级占用或通信错误。
 * 与后台采集共用采集锁。
 */
int ssf_mems_acquisition_set_baudrate(struct ssf_mems_xyzs_data *data,
                                    enum ssf_mems_baudrate baudrate);
/* 复制当前采集状态。
 * 返回：无。
 * 只读主机状态，不发送串口请求。
 */
void ssf_mems_acquisition_get_status(
    struct ssf_mems_xyzs_data *data, struct ssf_mems_acquisition_status *status);
/* 读取一个传感器配置项。
 * 返回：0 成功，负数为参数、设备状态或通信错误。
 * 设备上线后才发送请求。
 */
int ssf_mems_acquisition_read_setting(struct ssf_mems_xyzs_data *data,
                                     enum ssf_mems_sensor_setting setting,
                                     u16 *value);
/* 写入一个传感器配置项。
 * 返回：0 成功，负数为参数、设备状态或通信错误。
 * 设备上线后才发送请求。
 */
int ssf_mems_acquisition_write_setting(struct ssf_mems_xyzs_data *data,
                                      enum ssf_mems_sensor_setting setting,
                                      u16 value);
/* 复制当前采集配置。
 * 返回：0 成功，-EINVAL 参数无效，-ENODEV 正在停止。
 * 可能等待在途定长采集窗口完成。
 */
int ssf_mems_acquisition_get_config(
    struct ssf_mems_xyzs_data *data, struct ssf_mems_acquisition_config *config);
/* 设置原始、特征或交替采集模式。
 * 返回：0 成功，负数为参数、停止或校验位错误。
 * 只改变主机调度，可能等待在途采集窗口完成。
 */
int ssf_mems_acquisition_set_mode(struct ssf_mems_xyzs_data *data,
                                  enum ssf_mems_acquisition_mode mode);
/* 设置一个采集窗口的时长。
 * 返回：0 成功，-EINVAL 参数无效，-ENODEV 正在停止。
 */
int ssf_mems_acquisition_set_duration(struct ssf_mems_xyzs_data *data,
                                      bool raw_window, unsigned int duration_ms);
/* 修改一个主机特征轮询配置项。
 * 返回：0 成功，-EINVAL 配置无效，-ENODEV 正在停止。
 * 设置周期不写传感器参数，也不截断在途原始批次。
 */
int ssf_mems_acquisition_set_poll_setting(struct ssf_mems_xyzs_data *data,
                                          enum ssf_mems_poll_setting setting,
                                          unsigned int value);
/* 加载 bin 文件并升级传感器固件。
 * 返回：0 升级已确认，负数为文件、设备状态或传输错误。
 * 升级期间独占采集流程，末包结果不明时暂停自动重连。
 */
int ssf_mems_acquisition_update_firmware(struct ssf_mems_xyzs_data *data,
                                         const char *name);
