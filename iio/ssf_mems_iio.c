// SPDX-License-Identifier: GPL-2.0-only
#include "ssf_mems_iio.h"

#include "core.h"
#include "protocol.h"
#include "sensor_data.h"

#include <linux/bitmap.h>
#include <linux/errno.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>
#include <linux/serdev.h>
#include <linux/string.h>

struct ssf_mems_iio_state {
  struct ssf_mems_xyzs_data *data;
};

enum ssf_mems_iio_feature {
  SSF_MEMS_IIO_X_HIGH_FREQ_ACC_RMS,
  SSF_MEMS_IIO_X_LOW_FREQ_VELOCITY_RMS,
  SSF_MEMS_IIO_Y_HIGH_FREQ_ACC_RMS,
  SSF_MEMS_IIO_Y_LOW_FREQ_VELOCITY_RMS,
  SSF_MEMS_IIO_Z_HIGH_FREQ_ACC_RMS,
  SSF_MEMS_IIO_Z_LOW_FREQ_VELOCITY_RMS,
  SSF_MEMS_IIO_TEMPERATURE,
  SSF_MEMS_IIO_X_ACC_PEAK_TO_PEAK,
  SSF_MEMS_IIO_Y_ACC_PEAK_TO_PEAK,
  SSF_MEMS_IIO_Z_ACC_PEAK_TO_PEAK,
  SSF_MEMS_IIO_X_ACC_PEAK,
  SSF_MEMS_IIO_Y_ACC_PEAK,
  SSF_MEMS_IIO_Z_ACC_PEAK,
  SSF_MEMS_IIO_X_ACC_RMS,
  SSF_MEMS_IIO_Y_ACC_RMS,
  SSF_MEMS_IIO_Z_ACC_RMS,
  SSF_MEMS_IIO_X_KURTOSIS,
  SSF_MEMS_IIO_Y_KURTOSIS,
  SSF_MEMS_IIO_Z_KURTOSIS,
  SSF_MEMS_IIO_X_VELOCITY_RMS,
  SSF_MEMS_IIO_Y_VELOCITY_RMS,
  SSF_MEMS_IIO_Z_VELOCITY_RMS,
  SSF_MEMS_IIO_SOUND_RMS,
  SSF_MEMS_IIO_SOUND_PEAK,
  SSF_MEMS_IIO_SOUND_PEAK_TO_PEAK,
  SSF_MEMS_IIO_ZERO_CROSSING_RATE,
  SSF_MEMS_IIO_SPECTRAL_CENTROID,
  SSF_MEMS_IIO_SPECTRAL_FLUX,
  SSF_MEMS_IIO_STARTUP_FLAGS,
  SSF_MEMS_IIO_FEATURE_MAX,
};

enum ssf_mems_iio_scale {
  SSF_MEMS_IIO_SCALE_ACCEL,
  SSF_MEMS_IIO_SCALE_VELOCITY,
  SSF_MEMS_IIO_SCALE_TEMPERATURE,
  SSF_MEMS_IIO_SCALE_PRESSURE,
  SSF_MEMS_IIO_SCALE_CENTI,
  SSF_MEMS_IIO_SCALE_UNITY,
};

struct ssf_mems_iio_feature_desc {
  size_t offset;
  u8 width;
  enum ssf_mems_iio_scale scale;
};

#define SSF_MEMS_IIO_DESC(_id, _member, _scale)                              \
  [_id] = {                                                                  \
      .offset = offsetof(struct ssf_mems_sensor_data, _member),               \
      .width = sizeof(((struct ssf_mems_sensor_data *)0)->_member),           \
      .scale = (_scale),                                                      \
  }

static const struct ssf_mems_iio_feature_desc
    ssf_mems_iio_feature_descs[SSF_MEMS_IIO_FEATURE_MAX] = {
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_X_HIGH_FREQ_ACC_RMS,
                          x.high_freq_acc_rms_x100,
                          SSF_MEMS_IIO_SCALE_VELOCITY),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_X_LOW_FREQ_VELOCITY_RMS,
                          x.low_freq_velocity_rms_x100,
                          SSF_MEMS_IIO_SCALE_VELOCITY),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Y_HIGH_FREQ_ACC_RMS,
                          y.high_freq_acc_rms_x100,
                          SSF_MEMS_IIO_SCALE_VELOCITY),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Y_LOW_FREQ_VELOCITY_RMS,
                          y.low_freq_velocity_rms_x100,
                          SSF_MEMS_IIO_SCALE_VELOCITY),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Z_HIGH_FREQ_ACC_RMS,
                          z.high_freq_acc_rms_x100,
                          SSF_MEMS_IIO_SCALE_VELOCITY),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Z_LOW_FREQ_VELOCITY_RMS,
                          z.low_freq_velocity_rms_x100,
                          SSF_MEMS_IIO_SCALE_VELOCITY),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_TEMPERATURE, temperature_x100,
                          SSF_MEMS_IIO_SCALE_TEMPERATURE),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_X_ACC_PEAK_TO_PEAK,
                          x.acc_peak_to_peak_x100,
                          SSF_MEMS_IIO_SCALE_ACCEL),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Y_ACC_PEAK_TO_PEAK,
                          y.acc_peak_to_peak_x100,
                          SSF_MEMS_IIO_SCALE_ACCEL),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Z_ACC_PEAK_TO_PEAK,
                          z.acc_peak_to_peak_x100,
                          SSF_MEMS_IIO_SCALE_ACCEL),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_X_ACC_PEAK, x.acc_peak_x100,
                          SSF_MEMS_IIO_SCALE_ACCEL),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Y_ACC_PEAK, y.acc_peak_x100,
                          SSF_MEMS_IIO_SCALE_ACCEL),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Z_ACC_PEAK, z.acc_peak_x100,
                          SSF_MEMS_IIO_SCALE_ACCEL),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_X_ACC_RMS, x.acc_rms_x100,
                          SSF_MEMS_IIO_SCALE_ACCEL),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Y_ACC_RMS, y.acc_rms_x100,
                          SSF_MEMS_IIO_SCALE_ACCEL),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Z_ACC_RMS, z.acc_rms_x100,
                          SSF_MEMS_IIO_SCALE_ACCEL),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_X_KURTOSIS, x.kurtosis_x100,
                          SSF_MEMS_IIO_SCALE_CENTI),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Y_KURTOSIS, y.kurtosis_x100,
                          SSF_MEMS_IIO_SCALE_CENTI),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Z_KURTOSIS, z.kurtosis_x100,
                          SSF_MEMS_IIO_SCALE_CENTI),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_X_VELOCITY_RMS, x.velocity_rms_x100,
                          SSF_MEMS_IIO_SCALE_VELOCITY),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Y_VELOCITY_RMS, y.velocity_rms_x100,
                          SSF_MEMS_IIO_SCALE_VELOCITY),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Z_VELOCITY_RMS, z.velocity_rms_x100,
                          SSF_MEMS_IIO_SCALE_VELOCITY),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_SOUND_RMS, sound.rms_x100,
                          SSF_MEMS_IIO_SCALE_PRESSURE),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_SOUND_PEAK, sound.peak_x100,
                          SSF_MEMS_IIO_SCALE_PRESSURE),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_SOUND_PEAK_TO_PEAK,
                          sound.peak_to_peak_x100,
                          SSF_MEMS_IIO_SCALE_PRESSURE),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_ZERO_CROSSING_RATE,
                          zero_crossing_rate_x100,
                          SSF_MEMS_IIO_SCALE_CENTI),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_SPECTRAL_CENTROID,
                          spectral_centroid_x100,
                          SSF_MEMS_IIO_SCALE_CENTI),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_SPECTRAL_FLUX, spectral_flux_x100,
                          SSF_MEMS_IIO_SCALE_CENTI),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_STARTUP_FLAGS, startup_flags,
                          SSF_MEMS_IIO_SCALE_UNITY),
};

#define SSF_MEMS_IIO_AXIS_CHANNEL(_type, _modifier, _name, _feature)         \
  {                                                                          \
      .type = (_type), .modified = 1, .channel2 = (_modifier),               \
      .extend_name = (_name), .address = (_feature),                         \
      .scan_index = (_feature),                                               \
      .scan_type = {.sign = 'u',                                             \
                    .realbits = 16,                                           \
                    .storagebits = 16,                                        \
                    .endianness = IIO_CPU},                                   \
      .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |                         \
                            BIT(IIO_CHAN_INFO_SCALE),                         \
  }

#define SSF_MEMS_IIO_NAMED_CHANNEL(_type, _channel, _name, _feature)         \
  {                                                                          \
      .type = (_type), .indexed = 1, .channel = (_channel),                  \
      .extend_name = (_name), .address = (_feature),                         \
      .scan_index = (_feature),                                               \
      .scan_type = {.sign = 'u',                                             \
                    .realbits = 16,                                           \
                    .storagebits = 16,                                        \
                    .endianness = IIO_CPU},                                   \
      .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |                         \
                            BIT(IIO_CHAN_INFO_SCALE),                         \
  }

static const struct iio_chan_spec ssf_mems_iio_channels[] = {
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_VELOCITY, IIO_MOD_X, "high_freq_acc_rms",
                              SSF_MEMS_IIO_X_HIGH_FREQ_ACC_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_VELOCITY, IIO_MOD_X, "low_freq_rms",
                              SSF_MEMS_IIO_X_LOW_FREQ_VELOCITY_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_VELOCITY, IIO_MOD_Y, "high_freq_acc_rms",
                              SSF_MEMS_IIO_Y_HIGH_FREQ_ACC_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_VELOCITY, IIO_MOD_Y, "low_freq_rms",
                              SSF_MEMS_IIO_Y_LOW_FREQ_VELOCITY_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_VELOCITY, IIO_MOD_Z, "high_freq_acc_rms",
                              SSF_MEMS_IIO_Z_HIGH_FREQ_ACC_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_VELOCITY, IIO_MOD_Z, "low_freq_rms",
                              SSF_MEMS_IIO_Z_LOW_FREQ_VELOCITY_RMS),
    {
        .type = IIO_TEMP,
        .address = SSF_MEMS_IIO_TEMPERATURE,
        .scan_index = SSF_MEMS_IIO_TEMPERATURE,
        .scan_type =
            {
                .sign = 'u',
                .realbits = 16,
                .storagebits = 16,
                .endianness = IIO_CPU,
            },
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
                              BIT(IIO_CHAN_INFO_SCALE),
    },
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_X, "peak_to_peak",
                              SSF_MEMS_IIO_X_ACC_PEAK_TO_PEAK),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_Y, "peak_to_peak",
                              SSF_MEMS_IIO_Y_ACC_PEAK_TO_PEAK),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_Z, "peak_to_peak",
                              SSF_MEMS_IIO_Z_ACC_PEAK_TO_PEAK),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_X, "peak",
                              SSF_MEMS_IIO_X_ACC_PEAK),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_Y, "peak",
                              SSF_MEMS_IIO_Y_ACC_PEAK),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_Z, "peak",
                              SSF_MEMS_IIO_Z_ACC_PEAK),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_X, "rms",
                              SSF_MEMS_IIO_X_ACC_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_Y, "rms",
                              SSF_MEMS_IIO_Y_ACC_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_Z, "rms",
                              SSF_MEMS_IIO_Z_ACC_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_INDEX, IIO_MOD_X, "kurtosis",
                              SSF_MEMS_IIO_X_KURTOSIS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_INDEX, IIO_MOD_Y, "kurtosis",
                              SSF_MEMS_IIO_Y_KURTOSIS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_INDEX, IIO_MOD_Z, "kurtosis",
                              SSF_MEMS_IIO_Z_KURTOSIS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_VELOCITY, IIO_MOD_X, "rms",
                              SSF_MEMS_IIO_X_VELOCITY_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_VELOCITY, IIO_MOD_Y, "rms",
                              SSF_MEMS_IIO_Y_VELOCITY_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_VELOCITY, IIO_MOD_Z, "rms",
                              SSF_MEMS_IIO_Z_VELOCITY_RMS),
    SSF_MEMS_IIO_NAMED_CHANNEL(IIO_PRESSURE, 0, "sound_rms",
                               SSF_MEMS_IIO_SOUND_RMS),
    SSF_MEMS_IIO_NAMED_CHANNEL(IIO_PRESSURE, 1, "sound_peak",
                               SSF_MEMS_IIO_SOUND_PEAK),
    SSF_MEMS_IIO_NAMED_CHANNEL(IIO_PRESSURE, 2, "sound_peak_to_peak",
                               SSF_MEMS_IIO_SOUND_PEAK_TO_PEAK),
    SSF_MEMS_IIO_NAMED_CHANNEL(IIO_COUNT, 0, "zero_crossing_rate_percent",
                               SSF_MEMS_IIO_ZERO_CROSSING_RATE),
    SSF_MEMS_IIO_NAMED_CHANNEL(IIO_COUNT, 1, "spectral_centroid_hz",
                               SSF_MEMS_IIO_SPECTRAL_CENTROID),
    SSF_MEMS_IIO_NAMED_CHANNEL(IIO_COUNT, 2, "spectral_flux_pa2",
                               SSF_MEMS_IIO_SPECTRAL_FLUX),
    {
        .type = IIO_COUNT,
        .indexed = 1,
        .channel = 3,
        .extend_name = "startup_flags",
        .address = SSF_MEMS_IIO_STARTUP_FLAGS,
        .scan_index = SSF_MEMS_IIO_STARTUP_FLAGS,
        .scan_type =
            {
                .sign = 'u',
                .realbits = 8,
                .storagebits = 8,
                .endianness = IIO_CPU,
            },
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
    },
    IIO_CHAN_SOFT_TIMESTAMP(SSF_MEMS_IIO_FEATURE_MAX),
};

/* 只从最新完整特征快照取出一个字段，不访问传感器。 */
static int ssf_mems_iio_read_feature(struct ssf_mems_iio_state *state,
                                     const struct iio_chan_spec *chan,
                                     int *value) {
  const struct ssf_mems_iio_feature_desc *desc;
  struct ssf_mems_sensor_data features;
  u16 value16;
  int ret;

  if (chan->address >= SSF_MEMS_IIO_FEATURE_MAX)
    return -EINVAL;
  desc = &ssf_mems_iio_feature_descs[chan->address];

  ret = ssf_mems_protocol_get_features(state->data->serdev, &features);
  if (ret)
    return ret;

  if (desc->width == sizeof(u16)) {
    memcpy(&value16, (u8 *)&features + desc->offset, sizeof(value16));
    *value = value16;
  } else if (desc->width == sizeof(u8)) {
    *value = *((u8 *)&features + desc->offset);
  } else {
    return -EINVAL;
  }
  return 0;
}

/* 按 IIO ABI 单位返回寄存器 x100 数据的比例。 */
static int ssf_mems_iio_read_scale(const struct iio_chan_spec *chan, int *val,
                                   int *val2) {
  const struct ssf_mems_iio_feature_desc *desc;

  if (chan->address >= SSF_MEMS_IIO_FEATURE_MAX)
    return -EINVAL;
  desc = &ssf_mems_iio_feature_descs[chan->address];

  switch (desc->scale) {
  case SSF_MEMS_IIO_SCALE_ACCEL:
    /* 0.01 g -> 0.0980665 m/s^2. */
    *val = 0;
    *val2 = 98066500;
    return IIO_VAL_INT_PLUS_NANO;
  case SSF_MEMS_IIO_SCALE_VELOCITY:
    /* 0.01 mm/s -> 0.00001 m/s. */
    *val = 0;
    *val2 = 10;
    return IIO_VAL_INT_PLUS_MICRO;
  case SSF_MEMS_IIO_SCALE_TEMPERATURE:
    /* IIO 温度单位是毫摄氏度：0.01 C -> 10 mC。 */
    *val = 10;
    *val2 = 0;
    return IIO_VAL_INT;
  case SSF_MEMS_IIO_SCALE_PRESSURE:
    /* IIO 压力单位是 kPa：0.01 Pa -> 0.00001 kPa。 */
    *val = 0;
    *val2 = 10;
    return IIO_VAL_INT_PLUS_MICRO;
  case SSF_MEMS_IIO_SCALE_CENTI:
    *val = 0;
    *val2 = 10000;
    return IIO_VAL_INT_PLUS_MICRO;
  case SSF_MEMS_IIO_SCALE_UNITY:
    *val = 1;
    *val2 = 0;
    return IIO_VAL_INT;
  }
  return -EINVAL;
}

static int ssf_mems_iio_read_raw(struct iio_dev *indio_dev,
                                 const struct iio_chan_spec *chan, int *val,
                                 int *val2, long mask) {
  struct ssf_mems_iio_state *state = iio_priv(indio_dev);
  int ret;

  switch (mask) {
  case IIO_CHAN_INFO_RAW:
    ret = ssf_mems_iio_read_feature(state, chan, val);
    if (ret)
      return ret;
    return IIO_VAL_INT;
  case IIO_CHAN_INFO_SCALE:
    return ssf_mems_iio_read_scale(chan, val, val2);
  default:
    return -EINVAL;
  }
}

static ssize_t ssf_mems_iio_baudrate_show(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf) {
  struct iio_dev *indio_dev = dev_to_iio_dev(dev);
  struct ssf_mems_iio_state *state = iio_priv(indio_dev);
  enum ssf_mems_baudrate baudrate;
  int value;
  int ret;

  ret = ssf_mems_protocol_get_baudrate(state->data->serdev, &baudrate, 0);
  if (ret)
    return ret;
  value = ssf_mems_baudrate_to_value(baudrate);
  if (value < 0)
    return value;
  return sysfs_emit(buf, "%d\n", value);
}

static ssize_t ssf_mems_iio_baudrate_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf, size_t len) {
  struct iio_dev *indio_dev = dev_to_iio_dev(dev);
  struct ssf_mems_iio_state *state = iio_priv(indio_dev);
  enum ssf_mems_baudrate baudrate;
  unsigned int value;
  unsigned int i;
  int ret;

  ret = kstrtouint(buf, 0, &value);
  if (ret)
    return ret;

  /* 数值 9600 对应显式 9600 枚举，而不是“默认”别名。 */
  for (i = SSF_MEMS_BAUDRATE_2400; i < SSF_MEMS_BAUDRATE_MAX; i++) {
    if (ssf_mems_baudrate_to_value(i) == value)
      break;
  }
  if (i == SSF_MEMS_BAUDRATE_MAX)
    return -EINVAL;
  baudrate = i;

  ret = ssf_mems_protocol_set_baudrate(state->data->serdev, baudrate, 0);
  if (ret)
    return ret;
  return len;
}

static IIO_DEVICE_ATTR(sensor_baudrate, 0644, ssf_mems_iio_baudrate_show,
                       ssf_mems_iio_baudrate_store, 0);
static IIO_CONST_ATTR(
    sensor_baudrate_available,
    "2400 4800 9600 19200 38400 57600 115200 128000 230400 256000 "
    "460800 500000 512000 600000 750000 921600 1000000");

static struct attribute *ssf_mems_iio_attributes[] = {
    &iio_dev_attr_sensor_baudrate.dev_attr.attr,
    &iio_const_attr_sensor_baudrate_available.dev_attr.attr,
    NULL,
};

static const struct attribute_group ssf_mems_iio_attribute_group = {
    .attrs = ssf_mems_iio_attributes,
};

static const struct iio_info ssf_mems_iio_info = {
    .attrs = &ssf_mems_iio_attribute_group,
    .read_raw = ssf_mems_iio_read_raw,
};

static const struct iio_buffer_setup_ops ssf_mems_iio_buffer_ops = {};

/*
 * 所有特征中只有 startup_flags 为 8 bit，按全部 16 bit 估算可得到
 * 稍大但固定的上界；额外预留对齐后的 64 bit 时间戳。
 */
#define SSF_MEMS_IIO_MAX_SCAN_BYTES                                         \
  (ALIGN(SSF_MEMS_IIO_FEATURE_MAX * sizeof(u16), sizeof(s64)) + sizeof(s64))

static int ssf_mems_iio_push_buffer(
    struct iio_dev *indio_dev, const struct ssf_mems_sensor_data *features,
    s64 timestamp_ns) {
  u8 scan[SSF_MEMS_IIO_MAX_SCAN_BYTES] __aligned(sizeof(s64)) = {0};
  size_t offset = 0;
  unsigned int bit;

  if (!iio_buffer_enabled(indio_dev))
    return 0;
  if (!indio_dev->active_scan_mask)
    return -EINVAL;
  if (indio_dev->scan_bytes > sizeof(scan))
    return -EOVERFLOW;

  for_each_set_bit(bit, indio_dev->active_scan_mask,
                   SSF_MEMS_IIO_FEATURE_MAX) {
    const struct ssf_mems_iio_feature_desc *desc =
        &ssf_mems_iio_feature_descs[bit];

    offset = ALIGN(offset, desc->width);
    if (offset + desc->width > sizeof(scan))
      return -EOVERFLOW;
    memcpy(scan + offset, (const u8 *)features + desc->offset, desc->width);
    offset += desc->width;
  }

  if (!timestamp_ns)
    timestamp_ns = iio_get_time_ns(indio_dev);
  return iio_push_to_buffers_with_timestamp(indio_dev, scan, timestamp_ns);
}

int ssf_mems_iio_publish_features(
    struct ssf_mems_xyzs_data *data,
    const struct ssf_mems_sensor_data *features, s64 timestamp_ns) {
  int ret;

  if (!data || !features)
    return -EINVAL;

  ret = ssf_mems_protocol_store_features(data, features);
  if (ret)
    return ret;
  if (!data->indio_dev)
    return -ENODEV;

  return ssf_mems_iio_push_buffer(data->indio_dev, features, timestamp_ns);
}

int ssf_mems_iio_register(struct ssf_mems_xyzs_data *data) {
  struct iio_buffer *buffer;
  struct ssf_mems_iio_state *state;
  struct iio_dev *indio_dev;
  int ret;

  if (!data || !data->serdev)
    return -EINVAL;
  if (data->indio_dev)
    return -EBUSY;

  indio_dev = iio_device_alloc(&data->serdev->dev, sizeof(*state));
  if (!indio_dev)
    return -ENOMEM;

  state = iio_priv(indio_dev);
  state->data = data;
  indio_dev->name = "ssf_mems_xyzs";
  indio_dev->info = &ssf_mems_iio_info;
  indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
  indio_dev->channels = ssf_mems_iio_channels;
  indio_dev->num_channels = ARRAY_SIZE(ssf_mems_iio_channels);
  indio_dev->setup_ops = &ssf_mems_iio_buffer_ops;

  buffer = iio_kfifo_allocate();
  if (!buffer) {
    iio_device_free(indio_dev);
    return -ENOMEM;
  }
  iio_device_attach_buffer(indio_dev, buffer);

  ret = iio_device_register(indio_dev);
  if (ret) {
    iio_kfifo_free(indio_dev->buffer);
    iio_device_free(indio_dev);
    return ret;
  }

  data->indio_dev = indio_dev;
  return 0;
}

void ssf_mems_iio_unregister(struct ssf_mems_xyzs_data *data) {
  if (!data || !data->indio_dev)
    return;

  iio_device_unregister(data->indio_dev);
  iio_kfifo_free(data->indio_dev->buffer);
  iio_device_free(data->indio_dev);
  data->indio_dev = NULL;
}
