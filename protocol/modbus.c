#include "modbus.h"

#include "core.h"
#include "modbus_receive.h"
#include "modbus_request.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#define SSF_MEMS_FEATURE_BYTE_COUNT (SSF_MEMS_FEATURE_REG_COUNT * 2U)
#define SSF_MEMS_MODBUS_READ_OVERHEAD 5U

enum ssf_mems_feature_index {
  SSF_FEATURE_X_HIGH_FREQ_ACC_RMS,
  SSF_FEATURE_X_LOW_FREQ_VELOCITY_RMS,
  SSF_FEATURE_Y_HIGH_FREQ_ACC_RMS,
  SSF_FEATURE_Y_LOW_FREQ_VELOCITY_RMS,
  SSF_FEATURE_Z_HIGH_FREQ_ACC_RMS,
  SSF_FEATURE_Z_LOW_FREQ_VELOCITY_RMS,
  SSF_FEATURE_TEMPERATURE,
  SSF_FEATURE_X_ACC_PEAK_TO_PEAK,
  SSF_FEATURE_Y_ACC_PEAK_TO_PEAK,
  SSF_FEATURE_Z_ACC_PEAK_TO_PEAK,
  SSF_FEATURE_X_ACC_PEAK,
  SSF_FEATURE_Y_ACC_PEAK,
  SSF_FEATURE_Z_ACC_PEAK,
  SSF_FEATURE_X_ACC_RMS,
  SSF_FEATURE_Y_ACC_RMS,
  SSF_FEATURE_Z_ACC_RMS,
  SSF_FEATURE_X_KURTOSIS,
  SSF_FEATURE_Y_KURTOSIS,
  SSF_FEATURE_Z_KURTOSIS,
  SSF_FEATURE_X_VELOCITY_RMS,
  SSF_FEATURE_Y_VELOCITY_RMS,
  SSF_FEATURE_Z_VELOCITY_RMS,
  SSF_FEATURE_SOUND_RMS,
  SSF_FEATURE_SOUND_PEAK,
  SSF_FEATURE_SOUND_PEAK_TO_PEAK,
  SSF_FEATURE_ZERO_CROSSING_RATE,
  SSF_FEATURE_SPECTRAL_CENTROID,
  SSF_FEATURE_SPECTRAL_FLUX,
  SSF_FEATURE_STARTUP_FLAGS,
};

static void
ssf_mems_modbus_store_features(struct ssf_mems_xyzs_data *data,
                               const struct ssf_mems_sensor_data *features) {
  mutex_lock(&data->sensor_data_lock);
  data->sensor_data = *features;
  data->sensor_data_valid = true;
  mutex_unlock(&data->sensor_data_lock);
}

int ssf_mems_modbus_decode_features(const u16 *registers, size_t count,
                                    struct ssf_mems_sensor_data *result) {
  if (!registers || !result || count != SSF_MEMS_FEATURE_REG_COUNT){
    return -EINVAL;}

  memset(result, 0, sizeof(*result));

  result->x.high_freq_acc_rms_x100 =
      registers[SSF_FEATURE_X_HIGH_FREQ_ACC_RMS];
  result->x.low_freq_velocity_rms_x100 =
      registers[SSF_FEATURE_X_LOW_FREQ_VELOCITY_RMS];
  result->y.high_freq_acc_rms_x100 =
      registers[SSF_FEATURE_Y_HIGH_FREQ_ACC_RMS];
  result->y.low_freq_velocity_rms_x100 =
      registers[SSF_FEATURE_Y_LOW_FREQ_VELOCITY_RMS];
  result->z.high_freq_acc_rms_x100 =
      registers[SSF_FEATURE_Z_HIGH_FREQ_ACC_RMS];
  result->z.low_freq_velocity_rms_x100 =
      registers[SSF_FEATURE_Z_LOW_FREQ_VELOCITY_RMS];

  result->temperature_x100 = registers[SSF_FEATURE_TEMPERATURE];

  result->x.acc_peak_to_peak_x100 =
      registers[SSF_FEATURE_X_ACC_PEAK_TO_PEAK];
  result->y.acc_peak_to_peak_x100 =
      registers[SSF_FEATURE_Y_ACC_PEAK_TO_PEAK];
  result->z.acc_peak_to_peak_x100 =
      registers[SSF_FEATURE_Z_ACC_PEAK_TO_PEAK];
  result->x.acc_peak_x100 = registers[SSF_FEATURE_X_ACC_PEAK];
  result->y.acc_peak_x100 = registers[SSF_FEATURE_Y_ACC_PEAK];
  result->z.acc_peak_x100 = registers[SSF_FEATURE_Z_ACC_PEAK];
  result->x.acc_rms_x100 = registers[SSF_FEATURE_X_ACC_RMS];
  result->y.acc_rms_x100 = registers[SSF_FEATURE_Y_ACC_RMS];
  result->z.acc_rms_x100 = registers[SSF_FEATURE_Z_ACC_RMS];
  result->x.kurtosis_x100 = registers[SSF_FEATURE_X_KURTOSIS];
  result->y.kurtosis_x100 = registers[SSF_FEATURE_Y_KURTOSIS];
  result->z.kurtosis_x100 = registers[SSF_FEATURE_Z_KURTOSIS];
  result->x.velocity_rms_x100 = registers[SSF_FEATURE_X_VELOCITY_RMS];
  result->y.velocity_rms_x100 = registers[SSF_FEATURE_Y_VELOCITY_RMS];
  result->z.velocity_rms_x100 = registers[SSF_FEATURE_Z_VELOCITY_RMS];

  result->sound.rms_x100 = registers[SSF_FEATURE_SOUND_RMS];
  result->sound.peak_x100 = registers[SSF_FEATURE_SOUND_PEAK];
  result->sound.peak_to_peak_x100 =
      registers[SSF_FEATURE_SOUND_PEAK_TO_PEAK];
  result->zero_crossing_rate_x100 =
      registers[SSF_FEATURE_ZERO_CROSSING_RATE];
  result->spectral_centroid_x100 = registers[SSF_FEATURE_SPECTRAL_CENTROID];
  result->spectral_flux_x100 = registers[SSF_FEATURE_SPECTRAL_FLUX];
  result->startup_flags = registers[SSF_FEATURE_STARTUP_FLAGS] & 0x07;

  return 0;
}

int ssf_mems_modbus_read_features(struct serdev_device *serdev,
                                  unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_sensor_data features;
  u16 registers[SSF_MEMS_FEATURE_REG_COUNT];
  int ret;

  if (!serdev)
    return -EINVAL;

  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;

  ret = ssf_mems_modbus_read(serdev, SSF_MEMS_FEATURE_FIRST_REG,
                             SSF_MEMS_FEATURE_REG_COUNT, registers,
                             ARRAY_SIZE(registers), timeout_ms);
  if (ret)
    return ret;

  ret = ssf_mems_modbus_decode_features(registers, ARRAY_SIZE(registers),
                                        &features);
  if (ret)
    return ret;

  ssf_mems_modbus_store_features(data, &features);
  return 0;
}

int ssf_mems_modbus_get_features(struct serdev_device *serdev,
                                 struct ssf_mems_sensor_data *result) {
  struct ssf_mems_xyzs_data *data;

  if (!serdev || !result)
    return -EINVAL;

  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;

  mutex_lock(&data->sensor_data_lock);
  if (!data->sensor_data_valid) {
    mutex_unlock(&data->sensor_data_lock);
    return -ENODATA;
  }

  *result = data->sensor_data;
  mutex_unlock(&data->sensor_data_lock);

  return 0;
}

int ssf_mems_modbus_process_frame(struct ssf_mems_xyzs_data *data,
                                  const struct ssf_mems_frame_slot *slot) {
  struct ssf_mems_sensor_data features;
  u16 registers[SSF_MEMS_FEATURE_REG_COUNT];
  const u8 *frame;
  size_t frame_len;
  u8 byte_count;
  size_t i;
  int ret;

  if (!data || !slot || !slot->data)
    return -EINVAL;

  frame = slot->data;
  frame_len = slot->data_len;

  if (frame_len < SSF_MEMS_MODBUS_READ_OVERHEAD ||
      frame[1] != SSF_MEMS_MODBUS_FUNC_READ)
    return -EOPNOTSUPP;

  byte_count = frame[2];
  if (frame_len != (size_t)byte_count + SSF_MEMS_MODBUS_READ_OVERHEAD)
    return -EMSGSIZE;

  /* 58 字节的响应可唯一标识完整的特征数据块。 */
  if (byte_count != SSF_MEMS_FEATURE_BYTE_COUNT)
    return -ENOMSG;

  for (i = 0; i < ARRAY_SIZE(registers); i++)
    registers[i] = ((u16)frame[3 + i * 2] << 8) | frame[4 + i * 2];

  ret = ssf_mems_modbus_decode_features(registers, ARRAY_SIZE(registers),
                                        &features);
  if (ret)
    return ret;

  ssf_mems_modbus_store_features(data, &features);
  return 0;
}
