// SPDX-License-Identifier: GPL-2.0-only
#include "ssf_mems_iio.h"

#include "core.h"
#include "protocol.h"
#include "sensor_data.h"
#include "ssf_mems_acquisition.h"

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
  SSF_MEMS_IIO_SCALE_CENTI,
  SSF_MEMS_IIO_SCALE_DECI,
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
                          SSF_MEMS_IIO_SCALE_ACCEL),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_X_LOW_FREQ_VELOCITY_RMS,
                          x.low_freq_velocity_rms_x100,
                          SSF_MEMS_IIO_SCALE_VELOCITY),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Y_HIGH_FREQ_ACC_RMS,
                          y.high_freq_acc_rms_x100,
                          SSF_MEMS_IIO_SCALE_ACCEL),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Y_LOW_FREQ_VELOCITY_RMS,
                          y.low_freq_velocity_rms_x100,
                          SSF_MEMS_IIO_SCALE_VELOCITY),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_Z_HIGH_FREQ_ACC_RMS,
                          z.high_freq_acc_rms_x100,
                          SSF_MEMS_IIO_SCALE_ACCEL),
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
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_SOUND_RMS, sound.rms_db_x100,
                          SSF_MEMS_IIO_SCALE_CENTI),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_SOUND_PEAK, sound.peak_db_x100,
                          SSF_MEMS_IIO_SCALE_CENTI),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_SOUND_PEAK_TO_PEAK,
                          sound.peak_to_peak_db_x100,
                          SSF_MEMS_IIO_SCALE_CENTI),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_ZERO_CROSSING_RATE,
                          zero_crossing_rate_percent,
                          SSF_MEMS_IIO_SCALE_UNITY),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_SPECTRAL_CENTROID,
                          spectral_centroid_hz_x10,
                          SSF_MEMS_IIO_SCALE_DECI),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_SPECTRAL_FLUX, spectral_flux_x100,
                          SSF_MEMS_IIO_SCALE_CENTI),
        SSF_MEMS_IIO_DESC(SSF_MEMS_IIO_STARTUP_FLAGS, startup_flags,
                          SSF_MEMS_IIO_SCALE_UNITY),
};

#define SSF_MEMS_IIO_AXIS_CHANNEL(_type, _modifier, _name, _feature)         \
  {                                                                          \
      .type = (_type), .modified = 1, .channel2 = (_modifier),               \
      .extend_name = (_name), .address = (_feature),                         \
      .scan_index = (_feature),                                              \
      .scan_type = {.sign = 'u',                                             \
                    .realbits = 16,                                          \
                    .storagebits = 16,                                       \
                    .endianness = IIO_CPU},                                  \
      .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |                         \
                            BIT(IIO_CHAN_INFO_SCALE),                        \
  }

#define SSF_MEMS_IIO_NAMED_CHANNEL(_type, _channel, _name, _feature)         \
  {                                                                          \
      .type = (_type), .indexed = 1, .channel = (_channel),                  \
      .extend_name = (_name), .address = (_feature),                         \
      .scan_index = (_feature),                                              \
      .scan_type = {.sign = 'u',                                             \
                    .realbits = 16,                                          \
                    .storagebits = 16,                                       \
                    .endianness = IIO_CPU},                                  \
      .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |                         \
                            BIT(IIO_CHAN_INFO_SCALE),                        \
  }

static const struct iio_chan_spec ssf_mems_iio_channels[] = {
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_X, "high_freq_rms",
                              SSF_MEMS_IIO_X_HIGH_FREQ_ACC_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_VELOCITY, IIO_MOD_X, "low_freq_rms",
                              SSF_MEMS_IIO_X_LOW_FREQ_VELOCITY_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_Y, "high_freq_rms",
                              SSF_MEMS_IIO_Y_HIGH_FREQ_ACC_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_VELOCITY, IIO_MOD_Y, "low_freq_rms",
                              SSF_MEMS_IIO_Y_LOW_FREQ_VELOCITY_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_ACCEL, IIO_MOD_Z, "high_freq_rms",
                              SSF_MEMS_IIO_Z_HIGH_FREQ_ACC_RMS),
    SSF_MEMS_IIO_AXIS_CHANNEL(IIO_VELOCITY, IIO_MOD_Z, "low_freq_rms",
                              SSF_MEMS_IIO_Z_LOW_FREQ_VELOCITY_RMS),
    {
        .type = IIO_TEMP,
        .address = SSF_MEMS_IIO_TEMPERATURE,
        .scan_index = SSF_MEMS_IIO_TEMPERATURE,
        .scan_type =
            {
                .sign = 's',
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
    SSF_MEMS_IIO_NAMED_CHANNEL(IIO_COUNT, 0, "sound_rms_db",
                               SSF_MEMS_IIO_SOUND_RMS),
    SSF_MEMS_IIO_NAMED_CHANNEL(IIO_COUNT, 1, "sound_peak_db",
                               SSF_MEMS_IIO_SOUND_PEAK),
    SSF_MEMS_IIO_NAMED_CHANNEL(IIO_COUNT, 2, "sound_peak_to_peak_db",
                               SSF_MEMS_IIO_SOUND_PEAK_TO_PEAK),
    SSF_MEMS_IIO_NAMED_CHANNEL(IIO_COUNT, 3, "zero_crossing_rate_percent",
                               SSF_MEMS_IIO_ZERO_CROSSING_RATE),
    SSF_MEMS_IIO_NAMED_CHANNEL(IIO_COUNT, 4, "spectral_centroid_hz",
                               SSF_MEMS_IIO_SPECTRAL_CENTROID),
    SSF_MEMS_IIO_NAMED_CHANNEL(IIO_COUNT, 5, "spectral_flux",
                               SSF_MEMS_IIO_SPECTRAL_FLUX),
    {
        .type = IIO_COUNT,
        .indexed = 1,
        .channel = 6,
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
  s16 signed_value16;
  u16 value16;
  int ret;

  if (chan->address >= SSF_MEMS_IIO_FEATURE_MAX)
    return -EINVAL;
  desc = &ssf_mems_iio_feature_descs[chan->address];

  ret = ssf_mems_protocol_get_features(state->data->serdev, &features);
  if (ret != 0)
    return ret;

  if (desc->width == sizeof(u16)) {
    memcpy(&value16, (u8 *)&features + desc->offset, sizeof(value16));
    if (chan->scan_type.sign == 's') {
      memcpy(&signed_value16, &value16, sizeof(signed_value16));
      *value = signed_value16;
    } else {
      *value = value16;
    }
  } else if (desc->width == sizeof(u8)) {
    *value = *((u8 *)&features + desc->offset);
  } else {
    return -EINVAL;
  }
  return 0;
}

/* 按 IIO ABI 或通道名明示的单位返回寄存器比例。 */
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
  case SSF_MEMS_IIO_SCALE_CENTI:
    *val = 0;
    *val2 = 10000;
    return IIO_VAL_INT_PLUS_MICRO;
  case SSF_MEMS_IIO_SCALE_DECI:
    *val = 0;
    *val2 = 100000;
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
    if (ret != 0)
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

  ret = ssf_mems_acquisition_get_baudrate(state->data, &baudrate);
  if (ret != 0)
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
  if (ret != 0)
    return ret;

  /* 数值 9600 对应显式 9600 枚举，而不是“默认”别名。 */
  for (i = SSF_MEMS_BAUDRATE_2400; i < SSF_MEMS_BAUDRATE_MAX; i++) {
    if (ssf_mems_baudrate_to_value(i) == value)
      break;
  }
  if (i == SSF_MEMS_BAUDRATE_MAX)
    return -EINVAL;
  baudrate = i;

  ret = ssf_mems_acquisition_set_baudrate(state->data, baudrate);
  if (ret != 0)
    return ret;
  return len;
}

static IIO_DEVICE_ATTR(sensor_baudrate, 0644, ssf_mems_iio_baudrate_show,
                       ssf_mems_iio_baudrate_store, 0);

static const char *const ssf_mems_sampling_frequencies[] = {
    "533.34", "888.9", "1066.68", "1333.35", "2666.7",
    "2963", "5333.4", "8889", "13333.5", "26667",
};

static ssize_t ssf_mems_iio_setting_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  unsigned long address = to_iio_dev_attr(attr)->address;
  enum ssf_mems_sensor_setting setting = address == SSF_MEMS_SETTING_MAX ?
      SSF_MEMS_SETTING_SAMPLING_RATE : address;
  u16 value;
  int ret;

  ret = ssf_mems_acquisition_read_setting(state->data, setting, &value);
  if (ret != 0)
    return ret;
  if (address == SSF_MEMS_SETTING_MAX) {
    if (value >= ARRAY_SIZE(ssf_mems_sampling_frequencies))
      return -EINVAL;
    return sysfs_emit(buf, "%s\n", ssf_mems_sampling_frequencies[value]);
  }
  if (setting == SSF_MEMS_SETTING_PARAMETER_SWITCH)
    return sysfs_emit(buf, "0x%04x\n", value);
  return sysfs_emit(buf, "%u\n", value);
}

static ssize_t ssf_mems_iio_setting_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t len) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  unsigned int value;
  int ret;

  ret = kstrtouint(buf, 0, &value);
  if (ret != 0)
    return ret;
  if (value > 0xffff)
    return -EINVAL;
  ret = ssf_mems_acquisition_write_setting(
      state->data, to_iio_dev_attr(attr)->address, value);
  if (ret != 0)
    return ret;
  return len;
}

static IIO_DEVICE_ATTR(sensor_sampling_rate_index, 0644,
                       ssf_mems_iio_setting_show, ssf_mems_iio_setting_store,
                       SSF_MEMS_SETTING_SAMPLING_RATE);
static IIO_DEVICE_ATTR(sensor_sampling_frequency, 0444,
                       ssf_mems_iio_setting_show, NULL, SSF_MEMS_SETTING_MAX);
static IIO_DEVICE_ATTR(sensor_sampling_length_index, 0444,
                       ssf_mems_iio_setting_show, NULL,
                       SSF_MEMS_SETTING_SAMPLING_LENGTH);
static IIO_DEVICE_ATTR(sensor_parameter_switch, 0444,
                       ssf_mems_iio_setting_show, NULL,
                       SSF_MEMS_SETTING_PARAMETER_SWITCH);
static IIO_DEVICE_ATTR(sensor_feature_enable, 0644,
                       ssf_mems_iio_setting_show, ssf_mems_iio_setting_store,
                       SSF_MEMS_SETTING_FEATURE_ENABLE);
static IIO_DEVICE_ATTR(sensor_firmware_version, 0444,
                       ssf_mems_iio_setting_show, NULL,
                       SSF_MEMS_SETTING_FIRMWARE_VERSION);
static IIO_CONST_ATTR(sensor_sampling_rate_index_available, "0 1 2 3 4 5 6 7 8 9");

static ssize_t ssf_mems_iio_acquisition_show(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);
  struct ssf_mems_acquisition_status status;

  ssf_mems_acquisition_get_status(state->data, &status);
  switch (iio_attr->address) {
  case 0:
    return sysfs_emit(buf, "%u\n", status.online);
  case 1:
    return sysfs_emit(buf, "%u\n", status.interval_ms);
  case 2:
    if (status.have_sample == false)
      return -ENODATA;
    return sysfs_emit(buf, "%u\n", status.sample_age_ms);
  default:
    return -EINVAL;
  }
}

static IIO_DEVICE_ATTR(sensor_online, 0444, ssf_mems_iio_acquisition_show, NULL, 0);
static IIO_DEVICE_ATTR(sensor_poll_interval_ms, 0444,
                       ssf_mems_iio_acquisition_show, NULL, 1);
static IIO_DEVICE_ATTR(sensor_sample_age_ms, 0444,
                       ssf_mems_iio_acquisition_show, NULL, 2);
static IIO_CONST_ATTR(
    sensor_baudrate_available,
    "2400 4800 9600 19200 38400 57600 115200 128000 230400 256000 "
    "460800 500000 512000 600000 750000 921600 1000000");

static struct attribute *ssf_mems_iio_attributes[] = {
    &iio_dev_attr_sensor_baudrate.dev_attr.attr,
    &iio_dev_attr_sensor_online.dev_attr.attr,
    &iio_dev_attr_sensor_poll_interval_ms.dev_attr.attr,
    &iio_dev_attr_sensor_sample_age_ms.dev_attr.attr,
    &iio_const_attr_sensor_baudrate_available.dev_attr.attr,
    &iio_dev_attr_sensor_sampling_rate_index.dev_attr.attr,
    &iio_dev_attr_sensor_sampling_frequency.dev_attr.attr,
    &iio_dev_attr_sensor_sampling_length_index.dev_attr.attr,
    &iio_dev_attr_sensor_parameter_switch.dev_attr.attr,
    &iio_dev_attr_sensor_feature_enable.dev_attr.attr,
    &iio_dev_attr_sensor_firmware_version.dev_attr.attr,
    &iio_const_attr_sensor_sampling_rate_index_available.dev_attr.attr,
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

  if (iio_buffer_enabled(indio_dev) == false)
    return 0;
  if (indio_dev->active_scan_mask == NULL)
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

  if (timestamp_ns == 0)
    timestamp_ns = iio_get_time_ns(indio_dev);
  return iio_push_to_buffers_with_timestamp(indio_dev, scan, timestamp_ns);
}

int ssf_mems_iio_publish_features(
    struct ssf_mems_xyzs_data *data,
    const struct ssf_mems_sensor_data *features, s64 timestamp_ns) {
  int ret;

  if (data == NULL || features == NULL)
    return -EINVAL;

  ret = ssf_mems_protocol_store_features(data, features);
  if (ret != 0)
    return ret;
  if (data->indio_dev == NULL)
    return -ENODEV;

  /* IIO 配置 buffer/扫描掩码时同样持有 mlock。覆盖检查、打包和推送整个
   * 步骤，防止中途关闭 buffer 或释放 active_scan_mask。采集停止后才注销
   * IIO；这里不获取 info_exist_lock，也不在 buffer 回调中获取采集锁。 */
  mutex_lock(&data->indio_dev->mlock);
  ret = ssf_mems_iio_push_buffer(data->indio_dev, features, timestamp_ns);
  mutex_unlock(&data->indio_dev->mlock);
  return ret;
}

/* 注册原有特征设备；由统一注册入口负责原始设备失败时的回滚。 */
static int ssf_mems_iio_register_features(struct ssf_mems_xyzs_data *data) {
  struct iio_buffer *buffer;
  struct ssf_mems_iio_state *state;
  struct iio_dev *indio_dev;
  int ret;

  if (data == NULL || data->serdev == NULL)
    return -EINVAL;
  if (data->indio_dev != NULL)
    return -EBUSY;

  indio_dev = devm_iio_device_alloc(&data->serdev->dev, sizeof(*state));
  if (indio_dev == NULL)
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
  if (buffer == NULL) {
    return -ENOMEM;
  }
  iio_device_attach_buffer(indio_dev, buffer);

  ret = iio_device_register(indio_dev);
  if (ret != 0) {
    iio_kfifo_free(indio_dev->buffer);
    return ret;
  }

  data->indio_dev = indio_dev;
  return 0;
}

/* 一个原始扫描字段的内存布局；无需复制带有填充字节的整个结构体。 */
struct ssf_mems_raw_field {
  size_t offset; /* 字段在 ssf_mems_raw_sample 中的字节偏移。 */
  size_t width; /* 字段的存储字节数，也是 IIO 扫描对齐宽度。 */
};

static const struct ssf_mems_raw_field ssf_mems_raw_fields[] = {
    {offsetof(struct ssf_mems_raw_sample, xyz[0]), sizeof(s16)},
    {offsetof(struct ssf_mems_raw_sample, xyz[1]), sizeof(s16)},
    {offsetof(struct ssf_mems_raw_sample, xyz[2]), sizeof(s16)},
    {offsetof(struct ssf_mems_raw_sample, packet_sequence), sizeof(u32)},
    {offsetof(struct ssf_mems_raw_sample, sample_index), sizeof(u16)},
};

#define SSF_MEMS_RAW_AXIS(_axis, _index)                                    \
  {.type = IIO_ACCEL, .modified = 1, .channel2 = (_axis),                    \
   .scan_index = (_index), .info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),   \
   .scan_type = {.sign = 's', .realbits = 16, .storagebits = 16,              \
                 .endianness = IIO_CPU}}

static const struct iio_chan_spec ssf_mems_raw_channels[] = {
    SSF_MEMS_RAW_AXIS(IIO_MOD_X, 0),
    SSF_MEMS_RAW_AXIS(IIO_MOD_Y, 1),
    SSF_MEMS_RAW_AXIS(IIO_MOD_Z, 2),
    {.type = IIO_COUNT, .indexed = 1, .channel = 0,
     .extend_name = "packet_sequence", .scan_index = 3,
     .scan_type = {.sign = 'u', .realbits = 32, .storagebits = 32,
                   .endianness = IIO_CPU}},
    {.type = IIO_COUNT, .indexed = 1, .channel = 1,
     .extend_name = "sample_index", .scan_index = 4,
     .scan_type = {.sign = 'u', .realbits = 16, .storagebits = 16,
                   .endianness = IIO_CPU}},
    IIO_CHAN_SOFT_TIMESTAMP(5),
};

/* 原始 ADC 单位为 16/32768 g，IIO 加速度 scale 使用 m/s²，九位小数。 */
static int ssf_mems_raw_read_scale(struct iio_dev *indio_dev,
                                   const struct iio_chan_spec *chan,
                                   int *val, int *val2, long mask) {
  if (chan->type != IIO_ACCEL || mask != IIO_CHAN_INFO_SCALE)
    return -EINVAL;
  *val = 0;
  *val2 = 4788403; /* 9.80665 * 16 / 32768，四舍五入到纳米单位。 */
  return IIO_VAL_INT_PLUS_NANO;
}

/* 读取最近一轮原始采集统计；不会发送任何串口命令。 */
static ssize_t ssf_mems_raw_status_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  struct ssf_mems_raw_state *raw = &state->data->raw;
  unsigned int value;

  switch (to_iio_dev_attr(attr)->address) {
  case 0: value = READ_ONCE(raw->active); break;
  case 1: value = READ_ONCE(raw->packets); break;
  case 2: value = READ_ONCE(raw->discontinuities); break;
  case 3: value = READ_ONCE(raw->crc_errors); break;
  case 4: value = READ_ONCE(raw->buffer_errors); break;
  case 5:
    if (READ_ONCE(raw->packets) == 0 && READ_ONCE(raw->active) == false)
      return -ENODATA;
    value = READ_ONCE(raw->sampling_rate_index);
    if (value >= ARRAY_SIZE(ssf_mems_sampling_frequencies))
      return -ENODATA;
    return sysfs_emit(buf, "%s\n", ssf_mems_sampling_frequencies[value]);
  case 6: value = READ_ONCE(raw->drain_packets); break;
  case 7: value = READ_ONCE(raw->drain_discontinuities); break;
  case 8: value = READ_ONCE(raw->drain_crc_errors); break;
  case 9: value = READ_ONCE(raw->start_wait_ms); break;
  case 10: value = READ_ONCE(raw->stop_attempts); break;
  case 11: value = READ_ONCE(state->data->protocol.mode); break;
  case 12: value = READ_ONCE(raw->publishing); break;
  default: return -EINVAL;
  }
  return sysfs_emit(buf, "%u\n", value);
}

static IIO_DEVICE_ATTR(raw_active, 0444, ssf_mems_raw_status_show, NULL, 0);
static IIO_DEVICE_ATTR(raw_packets, 0444, ssf_mems_raw_status_show, NULL, 1);
static IIO_DEVICE_ATTR(raw_sequence_gaps, 0444, ssf_mems_raw_status_show, NULL, 2);
static IIO_DEVICE_ATTR(raw_crc_errors, 0444, ssf_mems_raw_status_show, NULL, 3);
static IIO_DEVICE_ATTR(raw_buffer_drops, 0444, ssf_mems_raw_status_show, NULL, 4);
static IIO_DEVICE_ATTR(raw_sampling_frequency, 0444,
                       ssf_mems_raw_status_show, NULL, 5);
/* 有效采集和排空分开观测；这些属性只读，不触发串口请求。 */
static IIO_DEVICE_ATTR(raw_drain_packets, 0444, ssf_mems_raw_status_show, NULL, 6);
static IIO_DEVICE_ATTR(raw_drain_sequence_gaps, 0444, ssf_mems_raw_status_show, NULL, 7);
static IIO_DEVICE_ATTR(raw_drain_crc_errors, 0444, ssf_mems_raw_status_show, NULL, 8);
static IIO_DEVICE_ATTR(raw_start_wait_ms, 0444, ssf_mems_raw_status_show, NULL, 9);
static IIO_DEVICE_ATTR(raw_stop_attempts, 0444, ssf_mems_raw_status_show, NULL, 10);
static IIO_DEVICE_ATTR(raw_link_mode, 0444, ssf_mems_raw_status_show, NULL, 11);
static IIO_DEVICE_ATTR(raw_publishing, 0444, ssf_mems_raw_status_show, NULL, 12);

static struct attribute *ssf_mems_raw_attributes[] = {
    &iio_dev_attr_raw_active.dev_attr.attr,
    &iio_dev_attr_raw_packets.dev_attr.attr,
    &iio_dev_attr_raw_sequence_gaps.dev_attr.attr,
    &iio_dev_attr_raw_crc_errors.dev_attr.attr,
    &iio_dev_attr_raw_buffer_drops.dev_attr.attr,
    &iio_dev_attr_raw_sampling_frequency.dev_attr.attr,
    &iio_dev_attr_raw_drain_packets.dev_attr.attr,
    &iio_dev_attr_raw_drain_sequence_gaps.dev_attr.attr,
    &iio_dev_attr_raw_drain_crc_errors.dev_attr.attr,
    &iio_dev_attr_raw_start_wait_ms.dev_attr.attr,
    &iio_dev_attr_raw_stop_attempts.dev_attr.attr,
    &iio_dev_attr_raw_link_mode.dev_attr.attr,
    &iio_dev_attr_raw_publishing.dev_attr.attr,
    NULL,
};

static const struct attribute_group ssf_mems_raw_attribute_group = {
    .attrs = ssf_mems_raw_attributes,
};

static const struct iio_info ssf_mems_raw_info = {
    .attrs = &ssf_mems_raw_attribute_group,
    .read_raw = ssf_mems_raw_read_scale,
};

/* 按 active_scan_mask 打包原始字段；与 buffer 重配置共享 mlock。 */
int ssf_mems_iio_publish_raw(struct ssf_mems_xyzs_data *data,
                            const struct ssf_mems_raw_sample *sample) {
  u8 scan[24] __aligned(sizeof(s64)) = {0};
  struct iio_dev *indio_dev;
  unsigned int bit;
  size_t offset = 0;
  int ret = 0;

  if (data == NULL || sample == NULL)
    return -EINVAL;
  indio_dev = data->raw_indio_dev;
  if (indio_dev == NULL)
    return -ENODEV;
  mutex_lock(&indio_dev->mlock);
  if (iio_buffer_enabled(indio_dev) == false)
    goto out;
  if (indio_dev->active_scan_mask == NULL || indio_dev->scan_bytes > sizeof(scan)) {
    ret = -EINVAL;
    goto out;
  }
  for_each_set_bit(bit, indio_dev->active_scan_mask,
                   ARRAY_SIZE(ssf_mems_raw_fields)) {
    const struct ssf_mems_raw_field *field = &ssf_mems_raw_fields[bit];

    offset = ALIGN(offset, field->width);
    memcpy(scan + offset, (const u8 *)sample + field->offset, field->width);
    offset += field->width;
  }
  ret = iio_push_to_buffers_with_timestamp(indio_dev, scan,
                                           iio_get_time_ns(indio_dev));
out:
  mutex_unlock(&indio_dev->mlock);
  return ret;
}

/* 注册独立的原始数据 IIO 设备，保持原有特征扫描 ABI 不变。 */
static int ssf_mems_iio_register_raw(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_iio_state *state;
  struct iio_dev *indio_dev;
  struct iio_buffer *buffer;
  int ret;

  indio_dev = devm_iio_device_alloc(&data->serdev->dev, sizeof(*state));
  if (indio_dev == NULL)
    return -ENOMEM;
  state = iio_priv(indio_dev);
  state->data = data;
  indio_dev->name = "ssf_mems_xyzs_raw";
  indio_dev->info = &ssf_mems_raw_info;
  indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
  indio_dev->channels = ssf_mems_raw_channels;
  indio_dev->num_channels = ARRAY_SIZE(ssf_mems_raw_channels);
  indio_dev->setup_ops = &ssf_mems_iio_buffer_ops;
  buffer = iio_kfifo_allocate();
  if (buffer == NULL)
    return -ENOMEM;
  iio_device_attach_buffer(indio_dev, buffer);
  ret = iio_device_register(indio_dev);
  if (ret != 0) {
    iio_kfifo_free(buffer);
    return ret;
  }
  data->raw_indio_dev = indio_dev;
  return 0;
}

/* 两个 IIO 设备必须全部注册成功；失败时撤销已注册设备。 */
int ssf_mems_iio_register(struct ssf_mems_xyzs_data *data) {
  int ret = ssf_mems_iio_register_features(data);

  if (ret != 0)
    return ret;
  ret = ssf_mems_iio_register_raw(data);
  if (ret != 0)
    ssf_mems_iio_unregister(data);
  return ret;
}

void ssf_mems_iio_unregister(struct ssf_mems_xyzs_data *data) {
  if (data == NULL)
    return;

  if (data->raw_indio_dev != NULL) {
    iio_device_unregister(data->raw_indio_dev);
    iio_kfifo_free(data->raw_indio_dev->buffer);
    data->raw_indio_dev = NULL;
  }
  if (data->indio_dev == NULL)
    return;

  iio_device_unregister(data->indio_dev);
  iio_kfifo_free(data->indio_dev->buffer);
  /* IIO 对象由 serdev 设备的 devres 在解绑或 probe 失败时释放。 */
  data->indio_dev = NULL;
}
