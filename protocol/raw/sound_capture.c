// SPDX-License-Identifier: GPL-2.0-only
#include "raw/sound.h"
#include "raw/raw_capture.h"
#include "core.h"
#include "modbus/modbus.h"
#include "modbus/modbus_request.h"
#include "transport/serial_transport.h"
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>

static bool sound_stopping(struct ssf_mems_data *data) {
  return READ_ONCE(data->acquisition.stopping) ||
         READ_ONCE(data->modbus_req.shutting_down);
}

static void sound_receiver(struct ssf_mems_data *data, bool active) {
  ssf_mems_modbus_receive_flush(data);
  mutex_lock(&data->modbus_rx.parse_lock);
  data->sound.used = 0;
  WRITE_ONCE(data->sound.active, active);
  mutex_unlock(&data->modbus_rx.parse_lock);
}

int ssf_mems_sound_finish_locked(struct ssf_mems_data *data) {
  struct ssf_mems_sound_state *s = &data->sound;
  const struct ssf_reg_desc *reg;
  u16 value;
  int ret;

  WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_SOUND_RECOVERING);
  while (!READ_ONCE(s->done) && time_before(jiffies, s->deadline))
    msleep(20);
  if (!ssf_mems_raw_wait_quiet(data))
    return -ETIMEDOUT;
  mutex_lock(&data->modbus_rx.parse_lock);
  if (s->used)
    s->crc_errors++;
  mutex_unlock(&data->modbus_rx.parse_lock);
  sound_receiver(data, false);
  reg = ssf_mems_sensor_find_reg_by_name(data->sensor, "raw_sound_4096_enable");
  ret = reg ? ssf_mems_modbus_read_reg_recovery_locked(
                  data->serdev, reg->display_reg, &value, 500) : -ENOENT;
  if (ret == 0 && value != 0)
    ret = -EPROTO;
  if (ret == 0)
    WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_MODBUS);
  else
    sound_receiver(data, true);
  return ret;
}

int ssf_mems_sound_collect_finite(struct ssf_mems_data *data) {
  struct ssf_mems_sound_state *s;
  const struct ssf_reg_desc *reg;
  struct ssf_modbus_transfer transfer = {0};
  const u16 one = 1;
  u16 value;
  u8 frame[8];
  unsigned int i, timeout;
  int ret, finish_ret;
  long waited;

  if (!data || !data->serdev)
    return -EINVAL;
  if (data->protocol.parity != SSF_MEMS_PARITY_NONE)
    return -EOPNOTSUPP;
  reg = ssf_mems_sensor_find_reg_by_name(data->sensor, "raw_sound_4096_enable");
  if (!reg || !reg->write_single || !(reg->access & SSF_REG_WRITE))
    return -EOPNOTSUPP;
  s = &data->sound;
  mutex_lock(&data->protocol.bus_lock);
  if (data->protocol.mode != SSF_MEMS_LINK_MODBUS) {
    ret = ssf_mems_raw_finish_locked(data);
    if (ret)
      goto out;
  }
  /* 先经正式请求层确认空闲，继承旧应答隔离；不修改采样率或 Flash。 */
  ret = ssf_mems_modbus_read_reg_locked(data->serdev, reg->display_reg, &value, 500);
  if (ret)
    goto out;
  if (value != 0) {
    ret = -EBUSY;
    goto out;
  }
  transfer.frame = ssf_mems_modbus_find_frame(SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE);
  transfer.protocol_addr = reg->protocol_addr;
  transfer.reg_count = 1;
  ret = ssf_mems_modbus_build_request(&transfer, data->slave_id, &one, 1,
                                      frame, sizeof(frame));
  if (ret < 0)
    goto out;
  if (sound_stopping(data)) {
    ret = -ENODEV;
    goto out;
  }
  timeout = SSF_MEMS_RAW_START_TIMEOUT_MS + 2000U +
      ssf_mems_serial_wire_ms(data, SSF_MEMS_SOUND_PACKETS * SSF_MEMS_SOUND_PACKET_BYTES);
  ssf_mems_protocol_invalidate_features(data);
  ssf_mems_modbus_receive_flush(data);
  mutex_lock(&data->modbus_rx.parse_lock);
  s->running = true;
  s->acknowledged = s->done = false;
  s->packets = s->sequence_gaps = s->crc_errors = s->ignored_bytes = s->buffer_errors = 0;
  s->error = 0;
  s->trigger_address = reg->protocol_addr;
  s->deadline = jiffies + msecs_to_jiffies(timeout);
  s->used = 0;
  WRITE_ONCE(s->active, true);
  mutex_unlock(&data->modbus_rx.parse_lock);
  WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_SOUND);
  ret = ssf_mems_serial_write(data, frame, sizeof(frame), 200);
  if (ret == 0) {
    waited = wait_event_interruptible_timeout(
        s->waitq, READ_ONCE(s->done) || sound_stopping(data), msecs_to_jiffies(timeout));
    if (waited <= 0)
      ret = waited < 0 ? (int)waited : -ETIMEDOUT;
  }
  finish_ret = ssf_mems_sound_finish_locked(data);
  if (finish_ret)
    ret = finish_ret;
  if (ret == 0 && s->error)
    ret = s->error;
  if (ret == 0 && (!s->acknowledged || s->packets != SSF_MEMS_SOUND_PACKETS ||
                   s->sequence_gaps || s->crc_errors || s->ignored_bytes))
    ret = -EILSEQ;
  if (sound_stopping(data))
    ret = -ENODEV;
  if (ret == 0) {
    for (i = 0; i < SSF_MEMS_SOUND_SAMPLES; i++) {
      struct ssf_mems_sound_sample sample = {
          .pcm = s->samples[i], .packet_sequence = i / SSF_MEMS_SOUND_PACKET_SAMPLES,
          .sample_index = i % SSF_MEMS_SOUND_PACKET_SAMPLES,
      };
      if (sound_stopping(data)) {
        ret = -ENODEV;
        break;
      }
      if (!s->sample_handler || s->sample_handler(data, &sample) != 0)
        s->buffer_errors++;
    }
    if (ret == 0 && s->buffer_errors)
      ret = -ENOSPC;
  }
  if (ret == 0)
    s->captures++;
  WRITE_ONCE(s->running, false);
out:
  WRITE_ONCE(s->error, ret);
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}
