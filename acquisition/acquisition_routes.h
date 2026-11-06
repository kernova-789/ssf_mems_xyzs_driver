/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once
#include <linux/types.h>

struct ssf_mems_data;
struct ssf_mems_acquisition_config;

enum ssf_mems_acquisition_mode {
  SSF_MEMS_ACQ_MODE_RAW,
  SSF_MEMS_ACQ_MODE_FEATURE,
  SSF_MEMS_ACQ_MODE_ALTERNATING,
  SSF_MEMS_ACQ_MODE_SOUND,
  SSF_MEMS_ACQ_MODE_COUNT,
};

/* 固定轮转顺序；NONE 表示当前无可运行路线。 */
enum ssf_mems_acquisition_route {
  SSF_MEMS_ACQ_ROUTE_FEATURE,
  SSF_MEMS_ACQ_ROUTE_RAW,
  SSF_MEMS_ACQ_ROUTE_SOUND,
  SSF_MEMS_ACQ_ROUTE_COUNT,
  SSF_MEMS_ACQ_ROUTE_NONE = SSF_MEMS_ACQ_ROUTE_COUNT,
};

enum ssf_mems_acquisition_transfer {
  SSF_MEMS_ACQ_SNAPSHOT,
  SSF_MEMS_ACQ_FINITE_BATCH,
};

/* 路线描述集中绑定能力、Buffer 条件和单次事务；不拥有调度状态。 */
struct ssf_mems_acquisition_route_desc {
  const char *name;
  enum ssf_mems_acquisition_mode single_mode;
  enum ssf_mems_acquisition_transfer transfer;
  size_t duration_offset;
  bool manual_window;
  bool (*supported)(const struct ssf_mems_data *data);
  bool (*ready)(struct ssf_mems_data *data);
  int (*begin)(struct ssf_mems_data *data); /* 有限批次的交付代次快照。 */
  int (*collect)(struct ssf_mems_data *data,
                 const struct ssf_mems_acquisition_config *config,
                 unsigned int timeout_ms, bool reset_stats);
};

const struct ssf_mems_acquisition_route_desc *ssf_mems_acquisition_route_desc(
    enum ssf_mems_acquisition_route route);
const char *ssf_mems_acquisition_route_name(enum ssf_mems_acquisition_route route);
const char *ssf_mems_acquisition_mode_name(enum ssf_mems_acquisition_mode mode);
bool ssf_mems_acquisition_mode_supported(const struct ssf_mems_data *data,
                                         enum ssf_mems_acquisition_mode mode);
bool ssf_mems_acquisition_route_selected(
    const struct ssf_mems_acquisition_config *config,
    enum ssf_mems_acquisition_route route);
unsigned int *ssf_mems_acquisition_duration(
    struct ssf_mems_acquisition_config *config,
    enum ssf_mems_acquisition_route route);
