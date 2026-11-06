// SPDX-License-Identifier: GPL-2.0-only
#include "raw/capture_session.h"
#include "raw/raw_capture.h"
#include "raw/sound.h"
#include "core.h"
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/jiffies.h>

/* 等待串口接收持续安静 300 ms。
 * 返回：true 已静默，false 在 1 秒内未满足条件。
 */
bool ssf_mems_capture_wait_quiet(struct ssf_mems_data *data) {
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

/* 唯一收尾分派入口；OTA 不得被误当作振动流恢复。 */
int ssf_mems_capture_finish_locked(struct ssf_mems_data *data) {
  switch (READ_ONCE(data->protocol.mode)) {
  case SSF_MEMS_LINK_MODBUS:
    return 0;
  case SSF_MEMS_LINK_RAW_STARTING:
  case SSF_MEMS_LINK_RAW_STREAMING:
  case SSF_MEMS_LINK_RAW_RECOVERING:
    return ssf_mems_raw_finish_locked(data);
  case SSF_MEMS_LINK_SOUND:
  case SSF_MEMS_LINK_SOUND_RECOVERING:
    return ssf_mems_sound_finish_locked(data);
  default:
    return -EBUSY;
  }
}

int ssf_mems_protocol_recover_capture(struct ssf_mems_data *data) {
  int ret;

  if (!data || !data->serdev)
    return -EINVAL;
  mutex_lock(&data->protocol.bus_lock);
  ret = ssf_mems_capture_finish_locked(data);
  mutex_unlock(&data->protocol.bus_lock);
  return ret;
}
