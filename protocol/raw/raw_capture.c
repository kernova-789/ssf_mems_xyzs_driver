// SPDX-License-Identifier: GPL-2.0-only
#include "raw/raw_capture.h"
#include "transport/serial_transport.h"

#include "core.h"
#include "modbus/modbus.h"
#include "modbus/modbus_receive.h"
#include "modbus/modbus_request.h"
#include "model/modbus_table.h"

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>

/* 发送定长 XYZ 采集触发帧。
 * 返回：0 完整发送，负数为表项、组帧或串口错误。
 * 调用者须持 bus_lock，触发后不等待普通 Modbus 应答。
 */
static int ssf_mems_protocol_raw_command(struct ssf_mems_data *data) {
  struct ssf_modbus_transfer transfer = {0};
  const struct ssf_reg_desc *reg =
      ssf_mems_sensor_find_reg_by_name(data->sensor, "start_custom_length");
  u8 frame[8];
  int len;

  if (reg == NULL)
    return -ENOENT;
  transfer.frame = ssf_mems_modbus_find_frame(SSF_MEMS_MODBUS_FUNC_READ);
  transfer.protocol_addr = reg->protocol_addr;
  transfer.reg_count = 1;
  len = ssf_mems_modbus_build_request(&transfer, data->slave_id,
                                      NULL, 0, frame, sizeof(frame));
  if (len < 0)
    return len;
  return ssf_mems_serial_write(data, frame, len, 200);
}

/* 切换普通帧和原始数据接收方式。
 * 返回：无。
 * 切换前清理残留数据，不改变 UART 速率。
 */
static void ssf_mems_protocol_raw_receiver(struct ssf_mems_data *data,
                                          bool active) {
  if (READ_ONCE(data->raw.active) == active)
    return;
  ssf_mems_modbus_receive_flush(data);
  mutex_lock(&data->modbus_rx.parse_lock);
  data->raw.used = 0;
  WRITE_ONCE(data->raw.active, active);
  mutex_unlock(&data->modbus_rx.parse_lock);
}

/* 检查驱动是否正在停止。
 * 返回：true 正在停止，false 可以继续。
 */
static bool ssf_mems_protocol_raw_stopping(struct ssf_mems_data *data) {
  return READ_ONCE(data->acquisition.stopping) == true ||
         READ_ONCE(data->modbus_req.shutting_down) == true;
}

/* 等待串口接收持续安静 300 ms。
 * 返回：true 已静默，false 在 1 秒内未满足条件。
 */
static bool ssf_mems_protocol_raw_quiet(struct ssf_mems_data *data) {
  unsigned long began = jiffies;
  unsigned long deadline = began + msecs_to_jiffies(1000);
  unsigned long quiet = msecs_to_jiffies(SSF_MEMS_RAW_QUIET_MS);

  do {
    unsigned long last = ssf_mems_modbus_receive_last_activity(data);

    if (time_before(last, began) == true)
      last = began;
    if (time_after_eq(jiffies, last + quiet) == true)
      return true;
    msleep(20);
  } while (time_before(jiffies, deadline) == true);
  return false;
}

/* 收尾原始批次并确认普通通信恢复。
 * 返回：0 已恢复，负数为静默、表项或读回错误。
 * 等待在途批次自然结束，确认失败时禁止开始新业务。
 */
int ssf_mems_raw_finish_locked(struct ssf_mems_data *data) {
  struct ssf_mems_raw_state *raw = &data->raw;
  const struct ssf_reg_desc *reg;
  u16 reported;
  int ret;

  WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_RAW_RECOVERING);
  mutex_lock(&data->modbus_rx.parse_lock);
  WRITE_ONCE(raw->publishing, false);
  mutex_unlock(&data->modbus_rx.parse_lock);
  while (READ_ONCE(raw->batch_done) == false &&
         time_before(jiffies, data->protocol.raw_start_until) == true) {
    msleep(20);
  }
  if (ssf_mems_protocol_raw_quiet(data) == false)
    return -ETIMEDOUT;
  reg = ssf_mems_sensor_find_reg_by_name(data->sensor, "baudrate");
  if (reg == NULL)
    return -ENOENT;
  mutex_lock(&data->modbus_rx.parse_lock);
  if (raw->used != 0)
    WRITE_ONCE(raw->drain_crc_errors, raw->drain_crc_errors + 1U);
  mutex_unlock(&data->modbus_rx.parse_lock);
  ssf_mems_protocol_raw_receiver(data, false);
  ret = ssf_mems_modbus_read_reg_recovery_locked(data->serdev, reg->display_reg,
                                                &reported, 500);
  if (ret == 0 && ssf_mems_baudrate_to_value(data->sensor, reported) < 0)
    ret = -EPROTO;
  if (ret == 0) {
    WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_MODBUS);
  } else {
    ssf_mems_protocol_raw_receiver(data, true);
    dev_warn_ratelimited(&data->serdev->dev,
                         "finite batch completion unconfirmed: %d; read-only recovery, no stop/reset\n",
                         ret);
  }
  return ret;
}

/* 恢复尚未确认结束的原始批次。
 * 返回：0 已恢复或无需恢复，负数为参数或收尾错误。
 * 接收模块须仍可用，只读确认，不写 Flash。
 */
int ssf_mems_protocol_recover_raw(struct ssf_mems_data *data) {
  int ret = 0;

  if (data == NULL || data->serdev == NULL)
    return -EINVAL;
  mutex_lock(&data->protocol.bus_lock);
  if (data->protocol.mode != SSF_MEMS_LINK_MODBUS)
    ret = ssf_mems_raw_finish_locked(data);
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

/* 按型号提供的长度编码配置一批 XYZ。
 * 返回：0 成功，负数为表项、布局、通信或读回错误。
 * 只写长度高低字，并检查地址相邻。
 */
static int ssf_mems_protocol_raw_length(struct ssf_mems_data *data,
                                          u32 length) {
  const struct ssf_reg_desc *high_reg =
      ssf_mems_sensor_find_reg_by_name(data->sensor, "custom_length_high");
  const struct ssf_reg_desc *low_reg =
      ssf_mems_sensor_find_reg_by_name(data->sensor, "custom_length_low");
  const u16 desired[2] = {length >> 16, length & 0xffff};
  u16 high, low;
  int ret;

  if (high_reg == NULL || low_reg == NULL)
    return -ENOENT;
  /* FC10 必须同时写入高低字；换表后不能误写高字旁的其他配置。 */
  if ((u32)high_reg->display_reg + 1U != low_reg->display_reg ||
      (u32)high_reg->protocol_addr + 1U != low_reg->protocol_addr)
    return -EINVAL;
  ret = ssf_mems_modbus_read_reg_locked(data->serdev, high_reg->display_reg,
                                       &high, 500);
  if (ret != 0)
    return ret;
  ret = ssf_mems_modbus_read_reg_locked(data->serdev, low_reg->display_reg,
                                       &low, 500);
  if (ret != 0 || (high == desired[0] && low == desired[1]))
    return ret;
  ret = ssf_mems_modbus_write_regs_locked(data->serdev, high_reg->display_reg, 2,
                                          desired, ARRAY_SIZE(desired), 500);
  if (ret != 0)
    return ret;
  ret = ssf_mems_modbus_read_reg_locked(data->serdev, high_reg->display_reg,
                                       &high, 500);
  if (ret != 0)
    return ret;
  ret = ssf_mems_modbus_read_reg_locked(data->serdev, low_reg->display_reg,
                                       &low, 500);
  if (ret == 0 && (high != desired[0] || low != desired[1]))
    ret = -EIO;
  return ret;
}

/* 清零一个原始采集窗口的统计。
 * 返回：无。
 * 调用者持 bus_lock，在解析锁内整体更新。
 */
static void ssf_mems_raw_reset_window(struct ssf_mems_data *data,
                                      u16 sampling_index) {
  struct ssf_mems_raw_state *raw = &data->raw;

  mutex_lock(&data->modbus_rx.parse_lock);
  WRITE_ONCE(raw->packets, 0);
  WRITE_ONCE(raw->discontinuities, 0);
  WRITE_ONCE(raw->crc_errors, 0);
  WRITE_ONCE(raw->buffer_errors, 0);
  WRITE_ONCE(raw->drain_packets, 0);
  WRITE_ONCE(raw->drain_discontinuities, 0);
  WRITE_ONCE(raw->drain_crc_errors, 0);
  WRITE_ONCE(raw->batches, 0);
  WRITE_ONCE(raw->ignored_bytes, 0);
  WRITE_ONCE(raw->start_wait_ms, 0);
  WRITE_ONCE(raw->sampling_rate_index, sampling_index);
  mutex_unlock(&data->modbus_rx.parse_lock);
}

/* 准备新一批原始数据的接收状态。
 * 返回：无。
 * 调用者持 bus_lock；每批序号从零开始。
 */
static void ssf_mems_raw_begin_batch(struct ssf_mems_data *data) {
  struct ssf_mems_raw_state *raw = &data->raw;

  ssf_mems_protocol_raw_receiver(data, true);
  mutex_lock(&data->modbus_rx.parse_lock);
  raw->next_sequence = 0;
  WRITE_ONCE(raw->batch_packets, 0);
  WRITE_ONCE(raw->batch_done, false);
  WRITE_ONCE(raw->publishing, true);
  mutex_unlock(&data->modbus_rx.parse_lock);
}

/* 在指定窗口内逐批采集指定长度的 XYZ。
 * 返回：0 完成，负数为配置、通信、数据完整性或 IIO 错误。
 * 调用者持采集锁；全程独占总线，窗口到期仍收完在途批次。
 */
int ssf_mems_raw_collect_finite(struct ssf_mems_data *data,
                                 unsigned int duration_ms, u32 length) {
  struct ssf_mems_raw_state *raw;
  unsigned long window_until, started;
  unsigned int batch_ms, first_ms, hz = 0;
  u32 samples = length == 0 ? SSF_MEMS_RAW_BATCH_SAMPLES : length;
  u32 expected_packets;
  const char *frequency;
  u16 sampling_index;
  int ret, finish_ret;
  long waited;

  if (data == NULL || data->serdev == NULL || duration_ms == 0)
    return -EINVAL;
  if (data->protocol.parity != SSF_MEMS_PARITY_NONE)
    return -EOPNOTSUPP;
  if (!ssf_mems_sensor_raw_samples_valid(data->sensor, samples))
    return -EINVAL;
  expected_packets = samples / SSF_MEMS_RAW_SAMPLES_PER_PACKET;
  raw = &data->raw;
  mutex_lock(&data->protocol.bus_lock);
  if (data->protocol.mode != SSF_MEMS_LINK_MODBUS) {
    ret = ssf_mems_raw_finish_locked(data);
    if (ret != 0)
      goto out;
  }
  if (ssf_mems_sensor_find_reg_by_name(data->sensor, "start_custom_length") == NULL) {
    ret = -ENOENT;
    goto out;
  }
  ret = ssf_mems_modbus_read_reg_by_name_locked(
      data->serdev, "sampling_rate", &sampling_index, 500);
  if (ret != 0)
    goto out;
  if (sampling_index > (data->sensor->profile.sampling_frequency_count - 1U)) {
    ret = -EPROTO;
    goto out;
  }
  /* 定长采样率范围由型号表限定，当前手册包含索引 9。 */
  if (sampling_index > data->sensor->profile.raw_max_sampling_rate_index) {
    ret = -EOPNOTSUPP;
    goto out;
  }
  /* 部分型号边采边传；超过 UART 吞吐量会在设备端产生坏包。
   * 校验在长度写入、触发及缓存失效之前，不能靠忽略 CRC/序号来放行。
   */
  if (data->sensor->profile.raw_min_baudrates != NULL &&
      data->protocol.host_baudrate <
          data->sensor->profile.raw_min_baudrates[sampling_index]) {
    dev_warn_ratelimited(&data->serdev->dev,
        "finite raw sampling index %u requires baud >= %u (current %u)\n",
        sampling_index, data->sensor->profile.raw_min_baudrates[sampling_index],
        data->protocol.host_baudrate);
    ret = -EOPNOTSUPP;
    goto out;
  }
  /* 取 Hz 的整数部分，向保守方向估计设备采样耗时，无浮点运算。 */
  frequency = data->sensor->profile.sampling_frequencies[sampling_index];
  while (*frequency >= '0' && *frequency <= '9') {
    if (hz > 1000000U) {
      ret = -EINVAL;
      goto out;
    }
    hz = hz * 10U + *frequency++ - '0';
  }
  if (hz == 0) {
    ret = -EINVAL;
    goto out;
  }
  first_ms = SSF_MEMS_RAW_START_TIMEOUT_MS + DIV_ROUND_UP(samples * 1000U, hz);
  ret = ssf_mems_protocol_raw_length(data, length);
  if (ret != 0)
    goto out;
  if (ssf_mems_protocol_raw_stopping(data) == true) {
    ret = -ENODEV;
    goto out;
  }
  ssf_mems_protocol_invalidate_features(data);
  WRITE_ONCE(raw->batch_samples, samples);
  ssf_mems_raw_reset_window(data, sampling_index);
  batch_ms = first_ms + 2000U +
      ssf_mems_serial_wire_ms(data,
                              expected_packets * SSF_MEMS_RAW_PACKET_BYTES);
  window_until = jiffies + msecs_to_jiffies(duration_ms);
  do {
    ssf_mems_raw_begin_batch(data);
    msleep(max(5U, ssf_mems_serial_gap_ms(data)));
    if (ssf_mems_protocol_raw_stopping(data) == true) {
      WRITE_ONCE(raw->publishing, false);
      ssf_mems_protocol_raw_receiver(data, false);
      ret = -ENODEV;
      break;
    }
    WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_RAW_STARTING);
    started = jiffies;
    data->protocol.raw_start_until = started + msecs_to_jiffies(batch_ms);
    ret = ssf_mems_protocol_raw_command(data);
    if (ret == 0) {
      waited = wait_event_interruptible_timeout(
          raw->waitq, READ_ONCE(raw->batch_packets) != 0 ||
                          ssf_mems_protocol_raw_stopping(data) == true,
          msecs_to_jiffies(first_ms));
      WRITE_ONCE(raw->start_wait_ms, jiffies_to_msecs(jiffies - started));
      if (waited <= 0)
        ret = waited < 0 ? (int)waited : -ETIMEDOUT;
    }
    if (ret == 0 && ssf_mems_protocol_raw_stopping(data) == false &&
        READ_ONCE(raw->batch_done) == false) {
      WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_RAW_STREAMING);
      waited = wait_event_interruptible_timeout(
          raw->waitq, READ_ONCE(raw->batch_done) == true ||
                          ssf_mems_protocol_raw_stopping(data) == true,
          time_before(jiffies, data->protocol.raw_start_until) == true ?
              data->protocol.raw_start_until - jiffies : 1UL);
      if (waited <= 0)
        ret = waited < 0 ? (int)waited : -ETIMEDOUT;
    }
    if (ret != 0 || ssf_mems_protocol_raw_stopping(data) == true)
      WRITE_ONCE(raw->publishing, false);
    finish_ret = ssf_mems_raw_finish_locked(data);
    if (finish_ret != 0)
      ret = finish_ret;
    if (ret == 0 && (raw->batch_packets != expected_packets ||
                    raw->discontinuities != 0 || raw->crc_errors != 0 ||
                    raw->drain_discontinuities != 0 || raw->drain_crc_errors != 0 ||
                    raw->ignored_bytes != 0))
      ret = -EILSEQ;
    if (ret == 0 && raw->buffer_errors != 0)
      ret = -ENOSPC;
    if (ret == 0)
      WRITE_ONCE(raw->batches, raw->batches + 1U);
    if (ssf_mems_protocol_raw_stopping(data) == true)
      ret = -ENODEV;
    if (ret != 0)
      break;
  } while (time_before(jiffies, window_until) == true);
  dev_info(&data->serdev->dev,
           "finite raw capture: %u batches, %u packets, %u sequence gaps, %u bad candidates, %u IIO drops; drain %u/%u/%u, ignored %u, start %u ms, status %d\n",
           raw->batches, raw->packets, raw->discontinuities, raw->crc_errors,
           raw->buffer_errors, raw->drain_packets, raw->drain_discontinuities,
           raw->drain_crc_errors, raw->ignored_bytes, raw->start_wait_ms, ret);
out:
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

