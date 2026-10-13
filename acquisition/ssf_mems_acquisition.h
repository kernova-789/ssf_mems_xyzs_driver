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

enum ssf_mems_acquisition_phase {
  SSF_MEMS_ACQ_STARTUP,
  SSF_MEMS_ACQ_MATCH,
  SSF_MEMS_ACQ_SWITCH,
  SSF_MEMS_ACQ_CONFIGURE,
  SSF_MEMS_ACQ_VERIFY,
  SSF_MEMS_ACQ_POLL,
  SSF_MEMS_ACQ_RAW, /* 独占串口采集定长 XYZ 批次，完成窗口后回到特征轮询。 */
  SSF_MEMS_ACQ_BACKOFF,
  SSF_MEMS_ACQ_STOPPED,
};

struct ssf_mems_acquisition_state {
  struct task_struct *thread;
  wait_queue_head_t waitq;
  /* 串行化采集流程和 IIO 波特率操作；定时休眠期间不持锁。 */
  struct mutex lock;
  bool stopping;
  bool reconnect;
  bool online;
  bool have_sample;
  enum ssf_mems_acquisition_phase phase;
  enum ssf_mems_baudrate target_baudrate; /* 启动配置/手动设置的目标，不被扫描覆盖。 */
  unsigned int retry_ms;
  unsigned long last_sample;
  struct ssf_mems_acquisition_policy policy;
  enum ssf_mems_acquisition_mode mode; /* 调度使用的模式，不是串口协议模式。 */
  bool config_changed; /* 唤醒调度并从新的窗口起点应用配置，不重新配置传感器。 */
  unsigned int raw_duration_ms; /* 每轮定长批采集窗口；收完在途批次。 */
  unsigned int feature_duration_ms; /* 每轮特征轮询时长，不含原始模式退出时间。 */
  unsigned long feature_until; /* 本轮特征采集结束的 jiffies 时刻。 */
};

struct ssf_mems_acquisition_status {
  bool online;
  bool have_sample;
  unsigned int interval_ms;
  unsigned int sample_age_ms;
};

/* init 在注册 IIO 前调用，start 在 IIO 注册成功后调用。 */
int ssf_mems_acquisition_init(struct ssf_mems_xyzs_data *data);
int ssf_mems_acquisition_start(struct ssf_mems_xyzs_data *data);
/* 调用前关闭请求模块以唤醒在途请求；线程退出后仍清理未确认的私有流，
 * 返回后才允许释放 IIO/接收资源。 */
void ssf_mems_acquisition_stop(struct ssf_mems_xyzs_data *data);
int ssf_mems_acquisition_get_baudrate(struct ssf_mems_xyzs_data *data,
                                    enum ssf_mems_baudrate *baudrate);
/* 保持同步写入语义，成功后同时更新自动恢复的目标速率。 */
int ssf_mems_acquisition_set_baudrate(struct ssf_mems_xyzs_data *data,
                                    enum ssf_mems_baudrate baudrate);
void ssf_mems_acquisition_get_status(
    struct ssf_mems_xyzs_data *data, struct ssf_mems_acquisition_status *status);
int ssf_mems_acquisition_read_setting(struct ssf_mems_xyzs_data *data,
                                     enum ssf_mems_sensor_setting setting,
                                     u16 *value);
int ssf_mems_acquisition_write_setting(struct ssf_mems_xyzs_data *data,
                                      enum ssf_mems_sensor_setting setting,
                                      u16 value);
/* 配置读写与采集串行；写入等待当前原始窗口完成，不截断在途批次。 */
int ssf_mems_acquisition_get_config(
    struct ssf_mems_xyzs_data *data, struct ssf_mems_acquisition_config *config);
/* 切换调度策略，只修改主机状态，不写传感器参数或 Flash。 */
int ssf_mems_acquisition_set_mode(struct ssf_mems_xyzs_data *data,
                                  enum ssf_mems_acquisition_mode mode);
/* raw_window=true 配置原始窗口，否则配置特征窗口；范围 100..60000 ms。 */
int ssf_mems_acquisition_set_duration(struct ssf_mems_xyzs_data *data,
                                      bool raw_window, unsigned int duration_ms);
