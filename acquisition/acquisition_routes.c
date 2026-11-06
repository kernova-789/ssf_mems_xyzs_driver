// SPDX-License-Identifier: GPL-2.0-only
#include "acquisition_routes.h"
#include "ssf_mems_acquisition.h"
#include "ssf_mems_iio.h"
#include "core.h"
#include <linux/errno.h>
#include <linux/kernel.h>

static bool feature_supported(const struct ssf_mems_data *data) {
  return data && data->sensor_ops && data->sensor_ops->read_features;
}
static bool raw_supported(const struct ssf_mems_data *data) {
  return data && data->sensor_ops && data->sensor_ops->collect_raw;
}
static bool sound_supported(const struct ssf_mems_data *data) {
  return data && data->sensor_ops && data->sensor_ops->collect_sound;
}
static bool feature_ready(struct ssf_mems_data *data) {
  /* 特征还维护直接读取缓存，不依赖特征 Buffer。 */
  return feature_supported(data);
}
static int collect_feature(struct ssf_mems_data *data,
    const struct ssf_mems_acquisition_config *config,
    unsigned int timeout_ms, bool reset_stats) {
  (void)config;
  (void)reset_stats;
  return data->sensor_ops->read_features(data, timeout_ms);
}
static int collect_raw(struct ssf_mems_data *data,
    const struct ssf_mems_acquisition_config *config,
    unsigned int timeout_ms, bool reset_stats) {
  (void)timeout_ms;
  return data->sensor_ops->collect_raw(data, config->raw_batch_samples, reset_stats);
}
static int collect_sound(struct ssf_mems_data *data,
    const struct ssf_mems_acquisition_config *config,
    unsigned int timeout_ms, bool reset_stats) {
  (void)config;
  (void)timeout_ms;
  (void)reset_stats;
  return data->sensor_ops->collect_sound(data);
}

static const struct ssf_mems_acquisition_route_desc routes[] = {
  [SSF_MEMS_ACQ_ROUTE_FEATURE] = {
    .name = "feature", .single_mode = SSF_MEMS_ACQ_MODE_FEATURE,
    .transfer = SSF_MEMS_ACQ_SNAPSHOT,
    .duration_offset = offsetof(struct ssf_mems_acquisition_config, feature_duration_ms),
    .supported = feature_supported, .ready = feature_ready, .collect = collect_feature,
  },
  [SSF_MEMS_ACQ_ROUTE_RAW] = {
    .name = "raw", .single_mode = SSF_MEMS_ACQ_MODE_RAW,
    .transfer = SSF_MEMS_ACQ_FINITE_BATCH, .manual_window = true,
    .duration_offset = offsetof(struct ssf_mems_acquisition_config, raw_duration_ms),
    .supported = raw_supported, .ready = ssf_mems_iio_raw_enabled, .begin = ssf_mems_iio_begin_raw, .collect = collect_raw,
  },
  [SSF_MEMS_ACQ_ROUTE_SOUND] = {
    .name = "sound", .single_mode = SSF_MEMS_ACQ_MODE_SOUND,
    .transfer = SSF_MEMS_ACQ_FINITE_BATCH,
    .duration_offset = offsetof(struct ssf_mems_acquisition_config, sound_duration_ms),
    .supported = sound_supported, .ready = ssf_mems_iio_sound_enabled, .begin = ssf_mems_iio_begin_sound, .collect = collect_sound,
  },
};

_Static_assert(ARRAY_SIZE(routes) == SSF_MEMS_ACQ_ROUTE_COUNT, "route table must cover every route");

const struct ssf_mems_acquisition_route_desc *ssf_mems_acquisition_route_desc(
    enum ssf_mems_acquisition_route route) {
  if ((unsigned int)route >= ARRAY_SIZE(routes))
    return NULL;
  return &routes[route];
}
const char *ssf_mems_acquisition_route_name(enum ssf_mems_acquisition_route route) {
  const struct ssf_mems_acquisition_route_desc *desc = ssf_mems_acquisition_route_desc(route);

  return desc ? desc->name : "none";
}
const char *ssf_mems_acquisition_mode_name(enum ssf_mems_acquisition_mode mode) {
  unsigned int i;

  if (mode == SSF_MEMS_ACQ_MODE_ALTERNATING)
    return "alternating";
  for (i = 0; i < ARRAY_SIZE(routes); i++)
    if (routes[i].single_mode == mode)
      return routes[i].name;
  return NULL;
}
bool ssf_mems_acquisition_mode_supported(const struct ssf_mems_data *data,
                                         enum ssf_mems_acquisition_mode mode) {
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE(routes); i++)
    if ((mode == SSF_MEMS_ACQ_MODE_ALTERNATING || routes[i].single_mode == mode) &&
        routes[i].supported(data))
      return true;
  return false;
}
bool ssf_mems_acquisition_route_selected(
    const struct ssf_mems_acquisition_config *config,
    enum ssf_mems_acquisition_route route) {
  const struct ssf_mems_acquisition_route_desc *desc = ssf_mems_acquisition_route_desc(route);

  return desc && (config->mode == SSF_MEMS_ACQ_MODE_ALTERNATING ?
      config->alternating_en[route] : config->mode == desc->single_mode);
}
unsigned int *ssf_mems_acquisition_duration(
    struct ssf_mems_acquisition_config *config,
    enum ssf_mems_acquisition_route route) {
  const struct ssf_mems_acquisition_route_desc *desc = ssf_mems_acquisition_route_desc(route);

  return desc ? (unsigned int *)((u8 *)config + desc->duration_offset) : NULL;
}
