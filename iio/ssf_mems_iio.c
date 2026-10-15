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

/* IIO 私有区只保存设备指针，实际缓存和采集状态由相应模块管理。 */
struct ssf_mems_iio_state {
  struct ssf_mems_xyzs_data *data; /* 此 IIO 设备所属的驱动实例。 */
};

/* 特征索引同时用于描述表和 scan_index；变更顺序会改变缓冲区 ABI。 */
enum ssf_mems_iio_feature {
  SSF_MEMS_IIO_X_HIGH_FREQ_ACC_RMS, /* X 轴高频加速度有效值。 */
  SSF_MEMS_IIO_X_LOW_FREQ_VELOCITY_RMS, /* X 轴低频速度有效值。 */
  SSF_MEMS_IIO_Y_HIGH_FREQ_ACC_RMS, /* Y 轴高频加速度有效值。 */
  SSF_MEMS_IIO_Y_LOW_FREQ_VELOCITY_RMS, /* Y 轴低频速度有效值。 */
  SSF_MEMS_IIO_Z_HIGH_FREQ_ACC_RMS, /* Z 轴高频加速度有效值。 */
  SSF_MEMS_IIO_Z_LOW_FREQ_VELOCITY_RMS, /* Z 轴低频速度有效值。 */
  SSF_MEMS_IIO_TEMPERATURE, /* 有符号温度值。 */
  SSF_MEMS_IIO_X_ACC_PEAK_TO_PEAK, /* X 轴加速度峰峰值。 */
  SSF_MEMS_IIO_Y_ACC_PEAK_TO_PEAK, /* Y 轴加速度峰峰值。 */
  SSF_MEMS_IIO_Z_ACC_PEAK_TO_PEAK, /* Z 轴加速度峰峰值。 */
  SSF_MEMS_IIO_X_ACC_PEAK, /* X 轴加速度峰值。 */
  SSF_MEMS_IIO_Y_ACC_PEAK, /* Y 轴加速度峰值。 */
  SSF_MEMS_IIO_Z_ACC_PEAK, /* Z 轴加速度峰值。 */
  SSF_MEMS_IIO_X_ACC_RMS, /* X 轴加速度有效值。 */
  SSF_MEMS_IIO_Y_ACC_RMS, /* Y 轴加速度有效值。 */
  SSF_MEMS_IIO_Z_ACC_RMS, /* Z 轴加速度有效值。 */
  SSF_MEMS_IIO_X_KURTOSIS, /* X 轴峭度。 */
  SSF_MEMS_IIO_Y_KURTOSIS, /* Y 轴峭度。 */
  SSF_MEMS_IIO_Z_KURTOSIS, /* Z 轴峭度。 */
  SSF_MEMS_IIO_X_VELOCITY_RMS, /* X 轴速度有效值。 */
  SSF_MEMS_IIO_Y_VELOCITY_RMS, /* Y 轴速度有效值。 */
  SSF_MEMS_IIO_Z_VELOCITY_RMS, /* Z 轴速度有效值。 */
  SSF_MEMS_IIO_SOUND_RMS, /* 声音有效值。 */
  SSF_MEMS_IIO_SOUND_PEAK, /* 声音峰值。 */
  SSF_MEMS_IIO_SOUND_PEAK_TO_PEAK, /* 声音峰峰值。 */
  SSF_MEMS_IIO_ZERO_CROSSING_RATE, /* 声音过零率，百分数。 */
  SSF_MEMS_IIO_SPECTRAL_CENTROID, /* 声音频谱重心。 */
  SSF_MEMS_IIO_SPECTRAL_FLUX, /* 声音频谱通量。 */
  SSF_MEMS_IIO_STARTUP_FLAGS, /* XYZ 运行判定位，不是启动完成标志。 */
  SSF_MEMS_IIO_FEATURE_MAX, /* 特征数量，同时为时间戳的扫描索引。 */
};

/* 定点寄存器值转换到 IIO 通道单位的比例类型。 */
enum ssf_mems_iio_scale {
  SSF_MEMS_IIO_SCALE_ACCEL, /* g × 100 转换为 m/s²。 */
  SSF_MEMS_IIO_SCALE_VELOCITY, /* mm/s × 100 转换为 m/s。 */
  SSF_MEMS_IIO_SCALE_TEMPERATURE, /* ℃ × 100 转换为毫摄氏度。 */
  SSF_MEMS_IIO_SCALE_CENTI, /* 寄存器比例为 0.01。 */
  SSF_MEMS_IIO_SCALE_DECI, /* 寄存器比例为 0.1。 */
  SSF_MEMS_IIO_SCALE_UNITY, /* 寄存器比例为 1。 */
};

/* 将一个 IIO 特征通道定位到完整特征快照中的字段。 */
struct ssf_mems_iio_feature_desc {
  size_t offset; /* 字段在 ssf_mems_sensor_data 中的字节偏移。 */
  u8 width; /* 字段存储宽度，当前为 1 或 2 字节。 */
  enum ssf_mems_iio_scale scale; /* 读取 scale 时采用的单位换算。 */
};

/* 按真实结构体成员生成偏移/宽度，避免手工维护一份字节布局。 */
#define SSF_MEMS_IIO_DESC(_id, _member, _scale)                              \
  [_id] = {                                                                  \
      .offset = offsetof(struct ssf_mems_sensor_data, _member),               \
      .width = sizeof(((struct ssf_mems_sensor_data *)0)->_member),           \
      .scale = (_scale),                                                      \
  }

/* 下标与特征枚举一一对应，用于直接读取和扫描记录打包。 */
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

/* 声明带 X/Y/Z 方向、16 位无符号扫描值的特征通道。 */
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

/* 声明按编号和名称区分的非轴向特征通道。 */
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

/* 特征设备的固定扫描布局；温度有符号，运行标志为 8 位，其余为 16 位。 */
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

/* 从特征缓存读取一个通道值。
 * 返回：0 成功，-ENODATA 无有效缓存，其他负数为参数或设备错误。
 */
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

/* 读取特征通道的单位换算比例。
 * 返回：IIO_VAL_* 表示数值格式，-EINVAL 通道无效。
 */
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

/* 读取 IIO 通道值或换算比例。
 * 返回：IIO_VAL_* 表示数值格式，负数为属性或缓存错误。
 * 特征值来自缓存，不发送串口请求。
 */
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

/* 读取通信波特率并输出文本。
 * 返回：输出字节数，负数为设备状态或通信错误。
 */
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

/* 解析用户输入并修改波特率。
 * 返回：成功返回 len，失败返回负数错误码。
 */
static ssize_t ssf_mems_iio_baudrate_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf, size_t len) {
  struct iio_dev *indio_dev = dev_to_iio_dev(dev);
  struct ssf_mems_iio_state *state = iio_priv(indio_dev);
  enum ssf_mems_baudrate baudrate;
  unsigned int value;
  int ret;

  ret = kstrtouint(buf, 0, &value);
  if (ret != 0)
    return ret;

  ret = ssf_mems_baudrate_from_value(value, &baudrate);
  if (ret != 0)
    return ret;

  ret = ssf_mems_acquisition_set_baudrate(state->data, baudrate);
  if (ret != 0)
    return ret;
  return len;
}

static IIO_DEVICE_ATTR(sensor_baudrate, 0644, ssf_mems_iio_baudrate_show,
                       ssf_mems_iio_baudrate_store, 0);

/* 属性编号只用于 IIO 分派，与协议配置枚举分开。 */
enum ssf_mems_iio_setting_attr {
  SSF_MEMS_IIO_SETTING_RATE,
  SSF_MEMS_IIO_SETTING_FREQUENCY,
  SSF_MEMS_IIO_SETTING_LENGTH,
  SSF_MEMS_IIO_SETTING_SWITCH,
  SSF_MEMS_IIO_SETTING_ENABLE,
  SSF_MEMS_IIO_SETTING_VERSION,
};
enum ssf_mems_iio_setting_format {
  SSF_MEMS_IIO_SETTING_DECIMAL,
  SSF_MEMS_IIO_SETTING_HEX,
  SSF_MEMS_IIO_SETTING_HZ,
};
struct ssf_mems_iio_setting_desc {
  enum ssf_mems_sensor_setting setting;
  enum ssf_mems_iio_setting_format format;
  bool writable;
};
static const struct ssf_mems_iio_setting_desc ssf_mems_iio_settings[] = {
    [SSF_MEMS_IIO_SETTING_RATE] =
        {SSF_MEMS_SETTING_SAMPLING_RATE, SSF_MEMS_IIO_SETTING_DECIMAL, true},
    [SSF_MEMS_IIO_SETTING_FREQUENCY] =
        {SSF_MEMS_SETTING_SAMPLING_RATE, SSF_MEMS_IIO_SETTING_HZ, false},
    [SSF_MEMS_IIO_SETTING_LENGTH] =
        {SSF_MEMS_SETTING_SAMPLING_LENGTH, SSF_MEMS_IIO_SETTING_DECIMAL, false},
    [SSF_MEMS_IIO_SETTING_SWITCH] =
        {SSF_MEMS_SETTING_PARAMETER_SWITCH, SSF_MEMS_IIO_SETTING_HEX, false},
    [SSF_MEMS_IIO_SETTING_ENABLE] =
        {SSF_MEMS_SETTING_FEATURE_ENABLE, SSF_MEMS_IIO_SETTING_DECIMAL, true},
    [SSF_MEMS_IIO_SETTING_VERSION] =
        {SSF_MEMS_SETTING_FIRMWARE_VERSION, SSF_MEMS_IIO_SETTING_DECIMAL, false},
};

/* 读取传感器配置并输出文本。
 * 返回：输出字节数，负数为配置或通信错误。
 */
static ssize_t ssf_mems_iio_setting_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  unsigned long address = to_iio_dev_attr(attr)->address;
  const struct ssf_mems_iio_setting_desc *desc;
  u16 value;
  int ret;

  if (address >= ARRAY_SIZE(ssf_mems_iio_settings))
    return -EINVAL;
  desc = &ssf_mems_iio_settings[address];
  ret = ssf_mems_acquisition_read_setting(state->data, desc->setting, &value);
  if (ret != 0)
    return ret;
  if (desc->format == SSF_MEMS_IIO_SETTING_HZ) {
    if (value >= ssf_mems_sensor_profile.sampling_frequency_count)
      return -EINVAL;
    return sysfs_emit(buf, "%s\n", ssf_mems_sensor_profile.sampling_frequencies[value]);
  }
  if (desc->format == SSF_MEMS_IIO_SETTING_HEX)
    return sysfs_emit(buf, "0x%04x\n", value);
  return sysfs_emit(buf, "%u\n", value);
}

/* 解析用户输入并修改传感器配置。
 * 返回：成功返回 len，失败返回负数错误码。
 */
static ssize_t ssf_mems_iio_setting_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t len) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  unsigned long address = to_iio_dev_attr(attr)->address;
  unsigned int value;
  int ret;

  if (address >= ARRAY_SIZE(ssf_mems_iio_settings) ||
      ssf_mems_iio_settings[address].writable == false)
    return -EINVAL;
  ret = kstrtouint(buf, 0, &value);
  if (ret != 0)
    return ret;
  if (value > 0xffff)
    return -EINVAL;
  ret = ssf_mems_acquisition_write_setting(
      state->data, ssf_mems_iio_settings[address].setting, value);
  if (ret != 0)
    return ret;
  return len;
}

static IIO_DEVICE_ATTR(sensor_sampling_rate_index, 0644,
                       ssf_mems_iio_setting_show, ssf_mems_iio_setting_store,
                       SSF_MEMS_IIO_SETTING_RATE);
static IIO_DEVICE_ATTR(sensor_sampling_frequency, 0444,
                       ssf_mems_iio_setting_show, NULL, SSF_MEMS_IIO_SETTING_FREQUENCY);
static IIO_DEVICE_ATTR(sensor_sampling_length_index, 0444,
                       ssf_mems_iio_setting_show, NULL,
                       SSF_MEMS_IIO_SETTING_LENGTH);
static IIO_DEVICE_ATTR(sensor_parameter_switch, 0444,
                       ssf_mems_iio_setting_show, NULL,
                       SSF_MEMS_IIO_SETTING_SWITCH);
static IIO_DEVICE_ATTR(sensor_feature_enable, 0644,
                       ssf_mems_iio_setting_show, ssf_mems_iio_setting_store,
                       SSF_MEMS_IIO_SETTING_ENABLE);
static IIO_DEVICE_ATTR(sensor_firmware_version, 0444,
                       ssf_mems_iio_setting_show, NULL,
                       SSF_MEMS_IIO_SETTING_VERSION);
/* 从型号表生成可选采样率编号。
 * 返回：输出字节数。
 */
static ssize_t ssf_mems_iio_rates_available(struct device *dev,
                                            struct device_attribute *attr,
                                            char *buf) {
  size_t i;
  int len = 0;

  for (i = 0; i < ssf_mems_sensor_profile.sampling_frequency_count; i++)
    len += sysfs_emit_at(buf, len, "%s%zu", i ? " " : "", i);
  return len + sysfs_emit_at(buf, len, "\n");
}
static IIO_DEVICE_ATTR(sensor_sampling_rate_index_available, 0444,
                       ssf_mems_iio_rates_available, NULL, 0);

/* 属性地址仅分派主机调度配置，不是 Modbus 寄存器地址。 */
enum ssf_mems_iio_acquisition_control {
  SSF_MEMS_IIO_ACQ_MODE,             /* 采集模式，raw / feature / alternating。 */
  SSF_MEMS_IIO_ACQ_RAW_DURATION,     /* 原始窗口，单位 ms。 */
  SSF_MEMS_IIO_ACQ_FEATURE_DURATION, /* 特征窗口，单位 ms。 */
};

/* 与采集层枚举对应的用户态名称。 */
static const char *const ssf_mems_iio_acquisition_modes[] = {
    [SSF_MEMS_ACQ_MODE_RAW] = "raw",                  //只进行原始数据定长采集
    [SSF_MEMS_ACQ_MODE_FEATURE] = "feature",          //只进行特征数据采集
    [SSF_MEMS_ACQ_MODE_ALTERNATING] = "alternating",  //两种数据交替采集
};

/* 读取主机采集模式或窗口配置。
 * 返回：输出字节数，负数为属性或设备状态错误。
 */
static ssize_t ssf_mems_iio_control_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  struct ssf_mems_acquisition_config config;
  unsigned long address = to_iio_dev_attr(attr)->address;
  int ret;

  ret = ssf_mems_acquisition_get_config(state->data, &config);
  if (ret != 0)
    return ret;
  switch (address) {
  case SSF_MEMS_IIO_ACQ_MODE:
    if ((unsigned int)config.mode >= ARRAY_SIZE(ssf_mems_iio_acquisition_modes))
      return -EINVAL;
    return sysfs_emit(buf, "%s\n", ssf_mems_iio_acquisition_modes[config.mode]);
  case SSF_MEMS_IIO_ACQ_RAW_DURATION:
    return sysfs_emit(buf, "%u\n", config.raw_duration_ms);
  case SSF_MEMS_IIO_ACQ_FEATURE_DURATION:
    return sysfs_emit(buf, "%u\n", config.feature_duration_ms);
  default:
    return -EINVAL;
  }
}

/* 修改主机采集模式或窗口配置。
 * 返回：成功返回 len，失败返回负数错误码。
 */
static ssize_t ssf_mems_iio_control_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t len) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  unsigned long address = to_iio_dev_attr(attr)->address;
  unsigned int value;
  unsigned int i;
  int ret;

  if (address == SSF_MEMS_IIO_ACQ_MODE) {
    for (i = 0; i < ARRAY_SIZE(ssf_mems_iio_acquisition_modes); i++) {
      if (sysfs_streq(buf, ssf_mems_iio_acquisition_modes[i]) == true)
        break;
    }
    if (i == ARRAY_SIZE(ssf_mems_iio_acquisition_modes))
      return -EINVAL;
    ret = ssf_mems_acquisition_set_mode(state->data, i);
  } else {
    if (address != SSF_MEMS_IIO_ACQ_RAW_DURATION &&
        address != SSF_MEMS_IIO_ACQ_FEATURE_DURATION)
      return -EINVAL;
    ret = kstrtouint(buf, 10, &value);
    if (ret != 0)
      return ret;
    ret = ssf_mems_acquisition_set_duration(
        state->data, address == SSF_MEMS_IIO_ACQ_RAW_DURATION, value);
  }
  if (ret != 0)
    return ret;
  return len;
}

static IIO_DEVICE_ATTR(sensor_acquisition_mode, 0644,
                       ssf_mems_iio_control_show, ssf_mems_iio_control_store,
                       SSF_MEMS_IIO_ACQ_MODE);
static IIO_CONST_ATTR(sensor_acquisition_mode_available, "raw feature alternating");
static IIO_DEVICE_ATTR(sensor_raw_duration_ms, 0644,
                       ssf_mems_iio_control_show, ssf_mems_iio_control_store,
                       SSF_MEMS_IIO_ACQ_RAW_DURATION);
static IIO_DEVICE_ATTR(sensor_feature_duration_ms, 0644,
                       ssf_mems_iio_control_show, ssf_mems_iio_control_store,
                       SSF_MEMS_IIO_ACQ_FEATURE_DURATION);

static const char *const ssf_mems_poll_modes[] = {
    [SSF_MEMS_POLL_FIXED] = "fixed",
    [SSF_MEMS_POLL_ADAPTIVE] = "adaptive",
};

/* 读取主机特征轮询模式或周期配置。
 * 返回：输出字节数，负数为属性或设备状态错误。
 */
static ssize_t ssf_mems_iio_poll_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  struct ssf_mems_acquisition_config config;
  unsigned int value;
  int ret;

  ret = ssf_mems_acquisition_get_config(state->data, &config);
  if (ret != 0)
    return ret;
  switch (to_iio_dev_attr(attr)->address) {
  case SSF_MEMS_POLL_SETTING_MODE:
    if ((unsigned int)config.poll.mode >= ARRAY_SIZE(ssf_mems_poll_modes))
      return -EINVAL;
    return sysfs_emit(buf, "%s\n", ssf_mems_poll_modes[config.poll.mode]);
  case SSF_MEMS_POLL_SETTING_FIXED_INTERVAL:
    value = config.poll.fixed_interval_ms;
    break;
  case SSF_MEMS_POLL_SETTING_MIN_INTERVAL:
    value = config.poll.min_interval_ms;
    break;
  case SSF_MEMS_POLL_SETTING_MAX_INTERVAL:
    value = config.poll.max_interval_ms;
    break;
  default:
    return -EINVAL;
  }
  return sysfs_emit(buf, "%u\n", value);
}

/* 修改主机特征轮询模式或周期配置。
 * 返回：成功返回 len，失败返回负数错误码。
 */
static ssize_t ssf_mems_iio_poll_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t len) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  unsigned long setting = to_iio_dev_attr(attr)->address;
  unsigned int value;
  int ret;

  if (setting == SSF_MEMS_POLL_SETTING_MODE) {
    for (value = 0; value < ARRAY_SIZE(ssf_mems_poll_modes); value++) {
      if (sysfs_streq(buf, ssf_mems_poll_modes[value]) == true)
        break;
    }
    if (value == ARRAY_SIZE(ssf_mems_poll_modes))
      return -EINVAL;
  } else {
    if (setting != SSF_MEMS_POLL_SETTING_FIXED_INTERVAL &&
        setting != SSF_MEMS_POLL_SETTING_MIN_INTERVAL &&
        setting != SSF_MEMS_POLL_SETTING_MAX_INTERVAL)
      return -EINVAL;
    ret = kstrtouint(buf, 10, &value);
    if (ret != 0)
      return ret;
  }
  ret = ssf_mems_acquisition_set_poll_setting(state->data, setting, value);
  return ret == 0 ? (ssize_t)len : ret;
}

static IIO_DEVICE_ATTR(sensor_poll_mode, 0644,
                       ssf_mems_iio_poll_show, ssf_mems_iio_poll_store,
                       SSF_MEMS_POLL_SETTING_MODE);
static IIO_CONST_ATTR(sensor_poll_mode_available, "fixed adaptive");
static IIO_DEVICE_ATTR(sensor_poll_fixed_interval_ms, 0644,
                       ssf_mems_iio_poll_show, ssf_mems_iio_poll_store,
                       SSF_MEMS_POLL_SETTING_FIXED_INTERVAL);
static IIO_DEVICE_ATTR(sensor_poll_min_interval_ms, 0644,
                       ssf_mems_iio_poll_show, ssf_mems_iio_poll_store,
                       SSF_MEMS_POLL_SETTING_MIN_INTERVAL);
static IIO_DEVICE_ATTR(sensor_poll_max_interval_ms, 0644,
                       ssf_mems_iio_poll_show, ssf_mems_iio_poll_store,
                       SSF_MEMS_POLL_SETTING_MAX_INTERVAL);

enum ssf_mems_iio_acquisition_status {
  SSF_MEMS_IIO_STATUS_ONLINE,
  SSF_MEMS_IIO_STATUS_POLL_INTERVAL,
  SSF_MEMS_IIO_STATUS_SAMPLE_AGE,
};

/* 读取在线状态、轮询周期或样本年龄。
 * 返回：输出字节数，-EINVAL 属性无效，-ENODATA 无样本年龄。
 */
static ssize_t ssf_mems_iio_acquisition_show(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);
  struct ssf_mems_acquisition_status status;

  ssf_mems_acquisition_get_status(state->data, &status);
  switch (iio_attr->address) {
  case SSF_MEMS_IIO_STATUS_ONLINE:
    return sysfs_emit(buf, "%u\n", status.online);
  case SSF_MEMS_IIO_STATUS_POLL_INTERVAL:
    return sysfs_emit(buf, "%u\n", status.interval_ms);
  case SSF_MEMS_IIO_STATUS_SAMPLE_AGE:
    if (status.have_sample == false)
      return -ENODATA;
    return sysfs_emit(buf, "%u\n", status.sample_age_ms);
  default:
    return -EINVAL;
  }
}

static IIO_DEVICE_ATTR(sensor_online, 0444, ssf_mems_iio_acquisition_show, NULL,
                       SSF_MEMS_IIO_STATUS_ONLINE);
static IIO_DEVICE_ATTR(sensor_poll_interval_ms, 0444,
                       ssf_mems_iio_acquisition_show, NULL, SSF_MEMS_IIO_STATUS_POLL_INTERVAL);
static IIO_DEVICE_ATTR(sensor_sample_age_ms, 0444,
                       ssf_mems_iio_acquisition_show, NULL, SSF_MEMS_IIO_STATUS_SAMPLE_AGE);
/* 从型号表生成可选通信速率，跳过默认别名和重复值。
 * 返回：输出字节数。
 */
static ssize_t ssf_mems_iio_bauds_available(struct device *dev,
                                            struct device_attribute *attr,
                                            char *buf) {
  size_t i, j;
  int len = 0;

  for (i = 1; i < ssf_mems_sensor_profile.baudrate_count; i++) {
    for (j = 1; j < i; j++)
      if (ssf_mems_sensor_profile.baudrates[j] == ssf_mems_sensor_profile.baudrates[i])
        break;
    if (j != i)
      continue;
    len += sysfs_emit_at(buf, len, "%s%u", len ? " " : "",
                         ssf_mems_sensor_profile.baudrates[i]);
  }
  return len + sysfs_emit_at(buf, len, "\n");
}
static IIO_DEVICE_ATTR(sensor_baudrate_available, 0444,
                       ssf_mems_iio_bauds_available, NULL, 0);

/* 根据用户给出的文件名执行固件升级。
 * 返回：成功返回 len，失败返回负数错误码。
 */
static ssize_t ssf_mems_iio_firmware_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t len) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  char name[SSF_MEMS_OTA_NAME_SIZE];
  int ret;

  ret = ssf_mems_ota_parse_name(buf, len, name);
  if (ret != 0)
    return ret;
  ret = ssf_mems_acquisition_update_firmware(state->data, name);
  return ret != 0 ? ret : (ssize_t)len;
}

/* 输出当前固件升级进度。
 * 返回：输出文本的字节数。
 */
static ssize_t ssf_mems_iio_firmware_show(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  struct ssf_mems_ota_status status;

  ssf_mems_ota_get_status(state->data, &status);
  return sysfs_emit(buf, "%s %zu/%zu error=%d version=%d blocked=%u\n",
                    ssf_mems_ota_stage_name(status.stage), status.acknowledged,
                    status.total, status.error, status.version,
                    READ_ONCE(state->data->acquisition.firmware_blocked));
}

static IIO_DEVICE_ATTR(sensor_firmware_update, 0200, NULL,
                       ssf_mems_iio_firmware_store, 0);
static IIO_DEVICE_ATTR(sensor_firmware_update_status, 0444,
                       ssf_mems_iio_firmware_show, NULL, 0);

static struct attribute *ssf_mems_iio_attributes[] = {
    &iio_dev_attr_sensor_firmware_update.dev_attr.attr,
    &iio_dev_attr_sensor_firmware_update_status.dev_attr.attr,
    &iio_dev_attr_sensor_acquisition_mode.dev_attr.attr,
    &iio_const_attr_sensor_acquisition_mode_available.dev_attr.attr,
    &iio_dev_attr_sensor_raw_duration_ms.dev_attr.attr,
    &iio_dev_attr_sensor_feature_duration_ms.dev_attr.attr,
    &iio_dev_attr_sensor_poll_mode.dev_attr.attr,
    &iio_const_attr_sensor_poll_mode_available.dev_attr.attr,
    &iio_dev_attr_sensor_poll_fixed_interval_ms.dev_attr.attr,
    &iio_dev_attr_sensor_poll_min_interval_ms.dev_attr.attr,
    &iio_dev_attr_sensor_poll_max_interval_ms.dev_attr.attr,
    &iio_dev_attr_sensor_baudrate.dev_attr.attr,
    &iio_dev_attr_sensor_online.dev_attr.attr,
    &iio_dev_attr_sensor_poll_interval_ms.dev_attr.attr,
    &iio_dev_attr_sensor_sample_age_ms.dev_attr.attr,
    &iio_dev_attr_sensor_baudrate_available.dev_attr.attr,
    &iio_dev_attr_sensor_sampling_rate_index.dev_attr.attr,
    &iio_dev_attr_sensor_sampling_frequency.dev_attr.attr,
    &iio_dev_attr_sensor_sampling_length_index.dev_attr.attr,
    &iio_dev_attr_sensor_parameter_switch.dev_attr.attr,
    &iio_dev_attr_sensor_feature_enable.dev_attr.attr,
    &iio_dev_attr_sensor_firmware_version.dev_attr.attr,
    &iio_dev_attr_sensor_sampling_rate_index_available.dev_attr.attr,
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

/* 把启用的特征通道打包推入 IIO 缓冲区。
 * 返回：0 成功或未启用缓冲区，负数为布局或推送错误。
 */
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

/* 把一份特征推入 IIO 缓冲区。
 * 返回：0 成功或未启用缓冲区，负数为参数、设备或推送错误。
 * 协议层维护缓存；这里只持 mlock 打包和推送。
 */
int ssf_mems_iio_publish_features(
    struct ssf_mems_xyzs_data *data,
    const struct ssf_mems_sensor_data *features, s64 timestamp_ns) {
  int ret;

  if (data == NULL || features == NULL)
    return -EINVAL;

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

/* 注册特征 IIO 设备和缓冲区。
 * 返回：0 成功，负数为参数、分配或注册错误。
 */
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

/* 原始扫描索引 0..4 对应 XYZ、包号、包内样本索引。 */
static const struct ssf_mems_raw_field ssf_mems_raw_fields[] = {
    {offsetof(struct ssf_mems_raw_sample, xyz[0]), sizeof(s16)},
    {offsetof(struct ssf_mems_raw_sample, xyz[1]), sizeof(s16)},
    {offsetof(struct ssf_mems_raw_sample, xyz[2]), sizeof(s16)},
    {offsetof(struct ssf_mems_raw_sample, packet_sequence), sizeof(u32)},
    {offsetof(struct ssf_mems_raw_sample, sample_index), sizeof(u16)},
};

/* 原始轴通道只提供 scale，实际值通过有符号 16 位 buffer 输出。 */
#define SSF_MEMS_RAW_AXIS(_axis, _index)                                    \
  {.type = IIO_ACCEL, .modified = 1, .channel2 = (_axis),                    \
   .scan_index = (_index), .info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),   \
   .scan_type = {.sign = 's', .realbits = 16, .storagebits = 16,              \
                 .endianness = IIO_CPU}}

/* 独立原始设备的固定通道布局，索引 5 为主机时间戳。 */
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

/* 读取原始加速度的单位换算比例。
 * 返回：IIO_VAL_INT_PLUS_NANO，-EINVAL 通道或属性无效。
 */
static int ssf_mems_raw_read_scale(struct iio_dev *indio_dev,
                                   const struct iio_chan_spec *chan,
                                   int *val, int *val2, long mask) {
  if (chan->type != IIO_ACCEL || mask != IIO_CHAN_INFO_SCALE)
    return -EINVAL;
  *val = 0;
  *val2 = 4788403; /* 9.80665 * 16 / 32768，四舍五入到纳米单位。 */
  return IIO_VAL_INT_PLUS_NANO;
}

enum ssf_mems_iio_raw_status {
  SSF_MEMS_IIO_RAW_ACTIVE,
  SSF_MEMS_IIO_RAW_PACKETS,
  SSF_MEMS_IIO_RAW_SEQUENCE_GAPS,
  SSF_MEMS_IIO_RAW_CRC_ERRORS,
  SSF_MEMS_IIO_RAW_BUFFER_DROPS,
  SSF_MEMS_IIO_RAW_FREQUENCY,
  SSF_MEMS_IIO_RAW_DRAIN_PACKETS,
  SSF_MEMS_IIO_RAW_DRAIN_SEQUENCE_GAPS,
  SSF_MEMS_IIO_RAW_DRAIN_CRC_ERRORS,
  SSF_MEMS_IIO_RAW_START_WAIT,
  SSF_MEMS_IIO_RAW_LINK_MODE,
  SSF_MEMS_IIO_RAW_PUBLISHING,
  SSF_MEMS_IIO_RAW_BATCHES,
  SSF_MEMS_IIO_RAW_BATCH_PACKETS,
  SSF_MEMS_IIO_RAW_BATCH_SAMPLES,
  SSF_MEMS_IIO_RAW_IGNORED_BYTES,
};

/* 读取定长原始采集的统计数据。
 * 返回：输出字节数，-EINVAL 属性无效，-ENODATA 无有效采样率。
 * 各统计独立读取，不保证来自同一时刻。
 */
static ssize_t ssf_mems_raw_status_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf) {
  struct ssf_mems_iio_state *state = iio_priv(dev_to_iio_dev(dev));
  struct ssf_mems_raw_state *raw = &state->data->raw;
  unsigned int value;

  switch (to_iio_dev_attr(attr)->address) {
  case SSF_MEMS_IIO_RAW_ACTIVE: value = READ_ONCE(raw->active); break;
  case SSF_MEMS_IIO_RAW_PACKETS: value = READ_ONCE(raw->packets); break;
  case SSF_MEMS_IIO_RAW_SEQUENCE_GAPS: value = READ_ONCE(raw->discontinuities); break;
  case SSF_MEMS_IIO_RAW_CRC_ERRORS: value = READ_ONCE(raw->crc_errors); break;
  case SSF_MEMS_IIO_RAW_BUFFER_DROPS: value = READ_ONCE(raw->buffer_errors); break;
  case SSF_MEMS_IIO_RAW_FREQUENCY:
    if (READ_ONCE(raw->packets) == 0 && READ_ONCE(raw->active) == false)
      return -ENODATA;
    value = READ_ONCE(raw->sampling_rate_index);
    if (value >= ssf_mems_sensor_profile.sampling_frequency_count)
      return -ENODATA;
    return sysfs_emit(buf, "%s\n", ssf_mems_sensor_profile.sampling_frequencies[value]);
  case SSF_MEMS_IIO_RAW_DRAIN_PACKETS: value = READ_ONCE(raw->drain_packets); break;
  case SSF_MEMS_IIO_RAW_DRAIN_SEQUENCE_GAPS: value = READ_ONCE(raw->drain_discontinuities); break;
  case SSF_MEMS_IIO_RAW_DRAIN_CRC_ERRORS: value = READ_ONCE(raw->drain_crc_errors); break;
  case SSF_MEMS_IIO_RAW_START_WAIT: value = READ_ONCE(raw->start_wait_ms); break;
  case SSF_MEMS_IIO_RAW_LINK_MODE: value = READ_ONCE(state->data->protocol.mode); break;
  case SSF_MEMS_IIO_RAW_PUBLISHING: value = READ_ONCE(raw->publishing); break;
  case SSF_MEMS_IIO_RAW_BATCHES: value = READ_ONCE(raw->batches); break;
  case SSF_MEMS_IIO_RAW_BATCH_PACKETS: value = READ_ONCE(raw->batch_packets); break;
  case SSF_MEMS_IIO_RAW_BATCH_SAMPLES: value = SSF_MEMS_RAW_BATCH_SAMPLES; break;
  case SSF_MEMS_IIO_RAW_IGNORED_BYTES: value = READ_ONCE(raw->ignored_bytes); break;
  default: return -EINVAL;
  }
  return sysfs_emit(buf, "%u\n", value);
}

static IIO_DEVICE_ATTR(raw_active, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_ACTIVE);
static IIO_DEVICE_ATTR(raw_packets, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_PACKETS);
static IIO_DEVICE_ATTR(raw_sequence_gaps, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_SEQUENCE_GAPS);
static IIO_DEVICE_ATTR(raw_crc_errors, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_CRC_ERRORS);
static IIO_DEVICE_ATTR(raw_buffer_drops, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_BUFFER_DROPS);
static IIO_DEVICE_ATTR(raw_sampling_frequency, 0444,
                       ssf_mems_raw_status_show, NULL, SSF_MEMS_IIO_RAW_FREQUENCY);
/* 有效采集和排空分开观测；这些属性只读，不触发串口请求。 */
static IIO_DEVICE_ATTR(raw_drain_packets, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_DRAIN_PACKETS);
static IIO_DEVICE_ATTR(raw_drain_sequence_gaps, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_DRAIN_SEQUENCE_GAPS);
static IIO_DEVICE_ATTR(raw_drain_crc_errors, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_DRAIN_CRC_ERRORS);
static IIO_DEVICE_ATTR(raw_start_wait_ms, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_START_WAIT);
static IIO_DEVICE_ATTR(raw_link_mode, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_LINK_MODE);
static IIO_DEVICE_ATTR(raw_publishing, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_PUBLISHING);
/* 定长批次状态只读；包号在每批 1024 组 XYZ 后从零重新开始。 */
static IIO_DEVICE_ATTR(raw_batches, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_BATCHES);
static IIO_DEVICE_ATTR(raw_batch_packets, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_BATCH_PACKETS);
static IIO_DEVICE_ATTR(raw_batch_samples, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_BATCH_SAMPLES);
static IIO_DEVICE_ATTR(raw_ignored_bytes, 0444, ssf_mems_raw_status_show, NULL,
                       SSF_MEMS_IIO_RAW_IGNORED_BYTES);

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
    &iio_dev_attr_raw_link_mode.dev_attr.attr,
    &iio_dev_attr_raw_publishing.dev_attr.attr,
    &iio_dev_attr_raw_batches.dev_attr.attr,
    &iio_dev_attr_raw_batch_packets.dev_attr.attr,
    &iio_dev_attr_raw_batch_samples.dev_attr.attr,
    &iio_dev_attr_raw_ignored_bytes.dev_attr.attr,
    NULL,
};

static const struct attribute_group ssf_mems_raw_attribute_group = {
    .attrs = ssf_mems_raw_attributes,
};

static const struct iio_info ssf_mems_raw_info = {
    .attrs = &ssf_mems_raw_attribute_group,
    .read_raw = ssf_mems_raw_read_scale,
};

/* 把一组 XYZ 原始样本推入 IIO 缓冲区。
 * 返回：0 成功或未启用缓冲区，负数为参数、设备或推送错误。
 * 注销 IIO 前须停止生产者；持 mlock 打包和推送。
 */
int ssf_mems_iio_publish_raw(struct ssf_mems_xyzs_data *data,
                            const struct ssf_mems_raw_sample *sample) {
  u8 scan[ALIGN(sizeof(*sample), sizeof(s64)) + sizeof(s64)]
      __aligned(sizeof(s64)) = {0};
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

/* 注册原始数据 IIO 设备和缓冲区。
 * 返回：0 成功，负数为分配或注册错误。
 */
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

/* 注册特征和原始数据两个 IIO 设备。
 * 返回：0 全部成功，负数为初始化错误。
 * 任一设备注册失败时撤销已注册设备。
 */
int ssf_mems_iio_register(struct ssf_mems_xyzs_data *data) {
  int ret = ssf_mems_iio_register_features(data);

  if (ret != 0)
    return ret;
  ret = ssf_mems_iio_register_raw(data);
  if (ret != 0)
    ssf_mems_iio_unregister(data);
  return ret;
}

/* 注销两个 IIO 设备并释放缓冲区。
 * 返回：无。
 * 调用前须停止所有数据生产者。
 */
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
