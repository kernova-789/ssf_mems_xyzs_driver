// SPDX-License-Identifier: GPL-2.0-only
#include "ota.h"
#include "core.h"

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/string.h>

#define SSF_MEMS_OTA_FUNCTION 0x41U
#define SSF_MEMS_OTA_QUERY 0xfffffffeU /* 不匹配 nextPack，只返回计数，不写 Flash。 */
#define SSF_MEMS_OTA_FINAL 0xffffffffU
#define SSF_MEMS_OTA_ACK_MS 5000U /* 首包需擦除两个 128 KiB 扇区。 */
#define SSF_MEMS_OTA_REBOOT_MS 15000U /* 包含 Bootloader 复制和应用再次启动。 */
#define SSF_MEMS_OTA_RETRIES 3U /* 仅用于查询和普通包；末包只发送一次。 */
#define SSF_MEMS_OTA_GAP_MS 20U

/* 手工读小端向量表，避免未对齐访问以及主机字节序差异。 */
static u32 ssf_mems_ota_le32(const u8 *p) {
  return (u32)p[0] | (u32)p[1] << 8 | (u32)p[2] << 16 | (u32)p[3] << 24;
}

int ssf_mems_ota_validate_image(const u8 *image, size_t size) {
  u32 stack, entry;

  if (image == NULL || size < 8)
    return -EINVAL;
  if (size > SSF_MEMS_OTA_MAX_SIZE)
    return -EFBIG;
  stack = ssf_mems_ota_le32(image);
  entry = ssf_mems_ota_le32(image + 4);
  /* STM32F407 SRAM 128 KiB / CCM 64 KiB；栈顶允许恰好等于区间上界。 */
  if ((stack & 7U) != 0 ||
      !((stack > 0x20000000U && stack <= 0x20020000U) ||
        (stack > 0x10000000U && stack <= 0x10010000U)))
    return -ENOEXEC;
  if ((entry & 1U) == 0 || (entry & ~1U) < SSF_MEMS_OTA_APP_ADDRESS + 8U ||
      (entry & ~1U) >= SSF_MEMS_OTA_APP_ADDRESS + size)
    return -ENOEXEC;
  return 0;
}

int ssf_mems_ota_parse_name(const char *buf, size_t len, char *name) {
  size_t i;

  if (buf == NULL || name == NULL || len == 0)
    return -EINVAL;
  if (buf[len - 1] == '\n')
    len--;
  if (len <= 4 || len >= SSF_MEMS_OTA_NAME_SIZE || buf[0] == '.' ||
      memcmp(buf + len - 4, ".bin", 4) != 0)
    return -EINVAL;
  for (i = 0; i < len; i++) {
    char c = buf[i];

    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-'))
      return -EINVAL;
  }
  memcpy(name, buf, len);
  name[len] = '\0';
  return 0;
}

int ssf_mems_ota_build_packet(u8 address, u32 sequence, const u8 *payload,
                              size_t size, u8 *frame) {
  unsigned int i;
  u8 sum = 0;

  if (address == 0 || address > 247 || frame == NULL ||
      size > SSF_MEMS_OTA_BLOCK_SIZE || (size != 0 && payload == NULL))
    return -EINVAL;
  frame[0] = address;
  frame[1] = SSF_MEMS_OTA_FUNCTION;
  frame[2] = sequence >> 24;
  frame[3] = sequence >> 16;
  frame[4] = sequence >> 8;
  frame[5] = sequence;
  memset(frame + 6, 0xff, SSF_MEMS_OTA_BLOCK_SIZE);
  if (size != 0)
    memcpy(frame + 6, payload, size);
  for (i = 0; i < SSF_MEMS_OTA_FRAME_SIZE - 1; i++)
    sum += frame[i];
  frame[SSF_MEMS_OTA_FRAME_SIZE - 1] = sum;
  return 0;
}

void ssf_mems_ota_init(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_ota_state *ota = &data->ota;

  memset(ota, 0, sizeof(*ota));
  mutex_init(&ota->update_lock);
  mutex_init(&ota->lock);
  init_waitqueue_head(&ota->waitq);
  ota->status.stage = SSF_MEMS_OTA_IDLE;
  ota->status.version = -1;
}

void ssf_mems_ota_shutdown(struct ssf_mems_xyzs_data *data) {
  mutex_lock(&data->ota.lock);
  WRITE_ONCE(data->ota.stopping, true);
  mutex_unlock(&data->ota.lock);
  wake_up_interruptible(&data->ota.waitq);
}

void ssf_mems_ota_set_stage(struct ssf_mems_xyzs_data *data,
                            enum ssf_mems_ota_stage stage, int error) {
  mutex_lock(&data->ota.lock);
  data->ota.status.stage = stage;
  data->ota.status.error = error;
  if (stage == SSF_MEMS_OTA_LOADING) {
    data->ota.status.acknowledged = 0;
    data->ota.status.total = 0;
    data->ota.status.version = -1;
  }
  mutex_unlock(&data->ota.lock);
}

void ssf_mems_ota_get_status(struct ssf_mems_xyzs_data *data,
                             struct ssf_mems_ota_status *status) {
  mutex_lock(&data->ota.lock);
  *status = data->ota.status;
  mutex_unlock(&data->ota.lock);
}

const char *ssf_mems_ota_stage_name(enum ssf_mems_ota_stage stage) {
  static const char *const names[] = {
      "idle", "loading", "waiting", "checking", "sending", "rebooting",
      "verifying", "complete", "failed", "uncertain",
  };

  return (unsigned int)stage < ARRAY_SIZE(names) ? names[stage] : "unknown";
}

void ssf_mems_ota_receive_byte(struct ssf_mems_xyzs_data *data, u8 byte) {
  struct ssf_mems_ota_state *ota = &data->ota;
  bool complete = false;

  mutex_lock(&ota->lock);
  if (ota->pending == false || ota->stopping == true)
    goto out;
  ota->reply[ota->used++] = byte;
  /* 地址不合或功能码不合时滑动一字节，兼容碎片交付及前导噪声。 */
  while (ota->used != 0 &&
         (ota->reply[0] != data->slave_id ||
          (ota->used >= 2 && ota->reply[1] != SSF_MEMS_OTA_FUNCTION))) {
    ota->used--;
    memmove(ota->reply, ota->reply + 1, ota->used);
  }
  if (ota->used == sizeof(ota->reply)) {
    ota->next_packet = (u32)ota->reply[2] << 24 | (u32)ota->reply[3] << 16 |
                       (u32)ota->reply[4] << 8 | ota->reply[5];
    WRITE_ONCE(ota->pending, false);
    ota->used = 0;
    complete = true;
  }
out:
  mutex_unlock(&ota->lock);
  if (complete == true)
    wake_up_interruptible(&ota->waitq);
}

/* 不持应答锁等待；卸载会唤醒，信号只允许在提交末包之前中止。 */
static int ssf_mems_ota_pause(struct ssf_mems_xyzs_data *data, unsigned int ms) {
  long ret = wait_event_interruptible_timeout(
      data->ota.waitq, READ_ONCE(data->ota.stopping) == true,
      msecs_to_jiffies(ms));

  if (READ_ONCE(data->ota.stopping) == true)
    return -ENODEV;
  return ret < 0 ? (int)ret : 0;
}

/* 每包一次同步发送；写入不足时不能简单补发尾部，固件按 IDLE 分帧。 */
static int ssf_mems_ota_exchange(struct ssf_mems_xyzs_data *data,
                                 const u8 *frame, u32 *next) {
  struct ssf_mems_ota_state *ota = &data->ota;
  unsigned int baud = data->protocol.host_baudrate;
  unsigned int bits = data->protocol.parity == SSF_MEMS_PARITY_NONE ? 10U : 11U;
  unsigned int timeout = DIV_ROUND_UP(SSF_MEMS_OTA_FRAME_SIZE * bits * 1000U,
                                     baud) + 200U;
  ssize_t written;
  long waited;
  int ret;

  ret = ssf_mems_ota_pause(data, SSF_MEMS_OTA_GAP_MS);
  if (ret != 0)
    return ret;
  ssf_mems_modbus_receive_flush(data);
  mutex_lock(&ota->lock);
  ota->used = 0;
  WRITE_ONCE(ota->pending, true);
  mutex_unlock(&ota->lock);
  written = serdev_device_write(data->serdev, frame, SSF_MEMS_OTA_FRAME_SIZE,
                                msecs_to_jiffies(timeout));
  serdev_device_wait_until_sent(data->serdev, msecs_to_jiffies(timeout));
  if (written != SSF_MEMS_OTA_FRAME_SIZE) {
    ret = written < 0 ? (int)written : -EIO;
    goto out;
  }
  waited = wait_event_interruptible_timeout(
      ota->waitq, READ_ONCE(ota->pending) == false ||
                     READ_ONCE(ota->stopping) == true,
      msecs_to_jiffies(SSF_MEMS_OTA_ACK_MS));
  mutex_lock(&ota->lock);
  if (ota->stopping == true)
    ret = -ENODEV;
  else if (ota->pending == false) {
    *next = ota->next_packet;
    ret = 0;
  } else
    ret = waited < 0 ? (int)waited : -ETIMEDOUT;
  mutex_unlock(&ota->lock);
out:
  mutex_lock(&ota->lock);
  WRITE_ONCE(ota->pending, false);
  ota->used = 0;
  mutex_unlock(&ota->lock);
  return ret;
}

/* 普通包的重复序号不会再次写 Flash；末包在固件中不具备这个保护。 */
static int ssf_mems_ota_send(struct ssf_mems_xyzs_data *data, const u8 *frame,
                             u32 expected, bool final) {
  unsigned int attempt, attempts = final == true ? 1U : SSF_MEMS_OTA_RETRIES;
  u32 next = 0;
  int ret = -ETIMEDOUT;

  for (attempt = 0; attempt < attempts; attempt++) {
    ret = ssf_mems_ota_exchange(data, frame, &next);
    if (ret == 0) {
      if (next == expected)
        return 0;
      /* 收到旧包/拒收应答可以重试同一普通包；其他计数绝不能越过。 */
      if (final == true || expected == 0 || next != expected - 1U)
        return -EPROTO;
      ret = -ETIMEDOUT;
    }
    if (ret != -ETIMEDOUT)
      break;
  }
  return ret;
}

int ssf_mems_ota_transfer(struct ssf_mems_xyzs_data *data,
                          const u8 *image, size_t size) {
  u8 frame[SSF_MEMS_OTA_FRAME_SIZE];
  size_t offset, count;
  u32 sequence = 0;
  unsigned int attempt;
  u16 version;
  bool final = false;
  int ret, verify_ret = -ETIMEDOUT;

  if (data == NULL || data->serdev == NULL)
    return -EINVAL;
  ret = ssf_mems_ota_validate_image(image, size);
  if (ret != 0)
    goto done;
  if (data->slave_id == 0 || data->slave_id > 247) {
    ret = -EINVAL;
    goto done;
  }
  mutex_lock(&data->ota.lock);
  data->ota.status.total = size;
  mutex_unlock(&data->ota.lock);
  mutex_lock(&data->protocol.bus_lock);
  if (READ_ONCE(data->ota.stopping) == true ||
      READ_ONCE(data->modbus_req.shutting_down) == true) {
    ret = -ENODEV;
    goto unlock;
  }
  if (data->protocol.mode != SSF_MEMS_LINK_MODBUS ||
      data->protocol.host_baudrate < 2400U) {
    ret = -EBUSY;
    goto unlock;
  }
  ssf_mems_ota_set_stage(data, SSF_MEMS_OTA_CHECKING, 0);
  /* 普通请求的静默/超时隔离也在此生效，确认设备可通信后再进入私有协议。 */
  ret = ssf_mems_modbus_read_reg_by_name_locked(
      data->serdev, "firmware_version", &version, 1000);
  if (ret != 0)
    goto unlock;
  ssf_mems_modbus_receive_flush(data);
  WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_OTA);
  ssf_mems_ota_build_packet(data->slave_id, SSF_MEMS_OTA_QUERY, NULL, 0, frame);
  ret = ssf_mems_ota_send(data, frame, 0, false);
  if (ret != 0) {
    dev_err(&data->serdev->dev,
            "OTA preflight failed: %d; nonzero packet count requires a sensor restart\n",
            ret);
    goto restore;
  }
  ssf_mems_protocol_invalidate_features(data);
  ssf_mems_ota_set_stage(data, SSF_MEMS_OTA_SENDING, 0);
  for (offset = 0; offset < size; offset += count, sequence++) {
    count = min((size_t)SSF_MEMS_OTA_BLOCK_SIZE, size - offset);
    final = offset + count == size;
    ssf_mems_ota_build_packet(data->slave_id,
                               final == true ? SSF_MEMS_OTA_FINAL : sequence,
                               image + offset, count, frame);
    ret = ssf_mems_ota_send(data, frame, sequence + 1U, final);
    if (ret == 0) {
      mutex_lock(&data->ota.lock);
      data->ota.status.acknowledged = offset + count;
      mutex_unlock(&data->ota.lock);
    }
    if (final == true || ret != 0)
      break;
  }
  if (final == true) {
    /* 即使应答丢失或写入返回错误，末包也可能生效；禁止重发，静默等启动。
     * 该有界等待不被信号打断，卸载也必须等它结束才释放总线。 */
    ssf_mems_ota_set_stage(data, SSF_MEMS_OTA_REBOOTING, ret);
    msleep(SSF_MEMS_OTA_REBOOT_MS);
  } else if (ret != 0) {
    /* 信号或短写可能打断首包擦除等待，此时不立即恢复普通帧发送。 */
    msleep(SSF_MEMS_OTA_ACK_MS);
  }
restore:
  ssf_mems_modbus_receive_flush(data);
  WRITE_ONCE(data->protocol.mode, SSF_MEMS_LINK_MODBUS);
  if (final == true && READ_ONCE(data->ota.stopping) == false &&
      READ_ONCE(data->modbus_req.shutting_down) == false) {
    ssf_mems_ota_set_stage(data, SSF_MEMS_OTA_VERIFYING, ret);
    /* 使用原通信参数只读验证，不因升级异常再次写 40110 或改变速率。 */
    for (attempt = 0; attempt < 15; attempt++) {
      verify_ret = ssf_mems_modbus_read_reg_by_name_locked(
          data->serdev, "firmware_version", &version, 1000);
      if (verify_ret != -ETIMEDOUT)
        break;
      if (ssf_mems_ota_pause(data, 1000) != 0)
        break;
    }
    if (verify_ret == 0) {
      mutex_lock(&data->ota.lock);
      data->ota.status.version = version;
      mutex_unlock(&data->ota.lock);
    }
    if (ret == 0)
      ret = verify_ret;
  } else if (final == true && ret == 0) {
    ret = -ENODEV;
  }
unlock:
  mutex_unlock(&data->protocol.bus_lock);
done:
  /* sysfs write 不能被 SA_RESTART 自动重做，否则可能再次提交整幅镜像。 */
  if (ret == -ERESTARTSYS)
    ret = -EINTR;
  ssf_mems_ota_set_stage(data, ret == 0 ? SSF_MEMS_OTA_COMPLETE :
      (final == true ? SSF_MEMS_OTA_UNCERTAIN : SSF_MEMS_OTA_FAILED), ret);
  return ret;
}
