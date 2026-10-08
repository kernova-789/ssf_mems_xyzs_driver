/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "acquisition_policy.h"
#include "protocol.h"

struct task_struct;
struct ssf_mems_xyzs_data;

enum ssf_mems_acquisition_phase {
  SSF_MEMS_ACQ_STARTUP,
  SSF_MEMS_ACQ_MATCH,
  SSF_MEMS_ACQ_SWITCH,
  SSF_MEMS_ACQ_VERIFY,
  SSF_MEMS_ACQ_POLL,
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
  enum ssf_mems_baudrate target_baudrate;
  unsigned int retry_ms;
  unsigned long last_sample;
  struct ssf_mems_acquisition_policy policy;
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
/* 调用前关闭请求模块以唤醒在途请求；返回后才允许释放 IIO/接收资源。 */
void ssf_mems_acquisition_stop(struct ssf_mems_xyzs_data *data);
int ssf_mems_acquisition_get_baudrate(struct ssf_mems_xyzs_data *data,
                                    enum ssf_mems_baudrate *baudrate);
/* 保持同步写入语义，成功后同时更新自动恢复的目标速率。 */
int ssf_mems_acquisition_set_baudrate(struct ssf_mems_xyzs_data *data,
                                    enum ssf_mems_baudrate baudrate);
void ssf_mems_acquisition_get_status(
    struct ssf_mems_xyzs_data *data, struct ssf_mems_acquisition_status *status);
