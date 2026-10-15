#include "modbus/modbus_receive.h"
#include "transport/serial_transport.h"
#include "core.h"
#include "modbus/modbus.h"
#include "model/modbus_table.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/kfifo.h>
#include <linux/string.h>

/* 取出接收队列中的字节并逐个解析。
 * 返回：0 成功，-EINVAL 设备无效，-ENODEV 接收不可用。
 * 只由接收工作项调用。
 */
static int ssf_mems_modbus_receive_drain(struct serdev_device *serdev);
/* 计算半帧的超时长度。
 * 返回：超时 tick 数。
 * 按最长帧传输时间加余量计算。
 */
static unsigned long ssf_mems_rx_expiry_ticks(struct ssf_mems_xyzs_data *data) {
  return msecs_to_jiffies(ssf_mems_serial_wire_ms(data,
                         SSF_MEMS_MODBUS_MAX_FRAME_LEN) + SSF_MEMS_RX_EXPIRY_MARGIN_MS);
}

/* 清理长时间没有收完整的候选帧。
 * 返回：无。
 * 队列非空时暂不清理。
 */
static void ssf_mems_rx_expiry_workfn(struct work_struct *work) {
  struct ssf_mems_modbus_receive_state *rx =
      container_of(work, struct ssf_mems_modbus_receive_state, expiry_work.work);
  struct ssf_mems_xyzs_data *data =
      container_of(rx, struct ssf_mems_xyzs_data, modbus_rx);
  unsigned long flags;
  unsigned long expiry = ssf_mems_rx_expiry_ticks(data);
  unsigned long now = jiffies;
  bool expired;

  mutex_lock(&rx->parse_lock);
  spin_lock_irqsave(&rx->fifo_lock, flags);
  if (rx->shutting_down == true || rx->flushing == true) {
    spin_unlock_irqrestore(&rx->fifo_lock, flags);
    mutex_unlock(&rx->parse_lock);
    return;
  }
  /* 已交付但尚未解析的字节应先被解析。push() 会重新启动空闲计时。 */
  expired = kfifo_is_empty(&rx->fifo) == true &&
            time_after_eq(now, rx->last_rx + expiry) == true;
  if (expired == false)
    mod_delayed_work(system_wq, &rx->expiry_work, expiry);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
  if (expired == true)
    rx->frame_used = 0;
  mutex_unlock(&rx->parse_lock);
}

/* 执行接收队列的解析任务。
 * 返回：无。
 */
static void ssf_mems_modbus_rx_workfn(struct work_struct *work) {
  struct ssf_mems_modbus_receive_state *rx =
      container_of(work, struct ssf_mems_modbus_receive_state, work);
  struct ssf_mems_xyzs_data *data =
      container_of(rx, struct ssf_mems_xyzs_data, modbus_rx);

  ssf_mems_modbus_receive_drain(data->serdev);
}

/* 初始化接收队列并注册完整帧回调。
 * 返回：0 成功，-EINVAL 参数无效，其他负数为分配错误。
 */
int ssf_mems_modbus_receive_init(struct ssf_mems_xyzs_data *data,
                               ssf_mems_frame_handler_t handler, void *context) {
  struct ssf_mems_modbus_receive_state *rx;
  int ret;

  if (data == NULL || handler == NULL)
    return -EINVAL;
  rx = &data->modbus_rx;
  spin_lock_init(&rx->fifo_lock);
  mutex_init(&rx->parse_lock);
  rx->last_rx = jiffies;
  rx->shutting_down = true;
  rx->flushing = false;
  rx->handler = NULL;
  rx->handler_context = NULL;
  INIT_WORK(&rx->work, ssf_mems_modbus_rx_workfn);
  INIT_DELAYED_WORK(&rx->expiry_work, ssf_mems_rx_expiry_workfn);
  rx->frame_used = 0;

  ret = kfifo_alloc(&rx->fifo, SSF_MEMS_RX_FIFO_SIZE, GFP_KERNEL);
  if (ret != 0)
    return ret;
  rx->handler = handler;
  rx->handler_context = context;
  rx->shutting_down = false;
  return 0;
}

/* 清空接收队列和未完成的解析窗口。
 * 返回：无。
 * 清理期间收到的字节会丢弃，调用时不能持请求锁。
 */
void ssf_mems_modbus_receive_flush(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_modbus_receive_state *rx;
  unsigned long flags;

  if (data == NULL)
    return;
  rx = &data->modbus_rx;

  spin_lock_irqsave(&rx->fifo_lock, flags);
  if (rx->shutting_down == true) {
    spin_unlock_irqrestore(&rx->fifo_lock, flags);
    return;
  }
  WRITE_ONCE(rx->flushing, true);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);

  cancel_work_sync(&rx->work);
  cancel_delayed_work_sync(&rx->expiry_work);

  mutex_lock(&rx->parse_lock);
  spin_lock_irqsave(&rx->fifo_lock, flags);
  kfifo_reset(&rx->fifo);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
  rx->frame_used = 0;
  mutex_unlock(&rx->parse_lock);

  spin_lock_irqsave(&rx->fifo_lock, flags);
  WRITE_ONCE(rx->flushing, false);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
}

/* 关闭接收、释放队列并清空解析窗口。
 * 返回：无。
 * 先停止接收工作项，再释放资源。
 */
void ssf_mems_modbus_receive_remove(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_modbus_receive_state *rx;
  unsigned long flags;

  if (data == NULL)
    return;
  rx = &data->modbus_rx;
  spin_lock_irqsave(&rx->fifo_lock, flags);
  WRITE_ONCE(rx->shutting_down, true);
  WRITE_ONCE(rx->flushing, false);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
  cancel_work_sync(&rx->work);
  cancel_delayed_work_sync(&rx->expiry_work);
  mutex_lock(&rx->parse_lock);
  rx->handler = NULL;
  rx->handler_context = NULL;
  rx->frame_used = 0;
  kfifo_free(&rx->fifo);
  mutex_unlock(&rx->parse_lock);
}

/* 读取最近收到串口字节的时间。
 * 返回：接收时间的 jiffies 值。
 * 包含清理期间丢弃的字节。
 */
unsigned long ssf_mems_modbus_receive_last_activity(struct ssf_mems_xyzs_data *data) {
  unsigned long flags;
  unsigned long last_rx;

  spin_lock_irqsave(&data->modbus_rx.fifo_lock, flags);
  last_rx = data->modbus_rx.last_rx;
  spin_unlock_irqrestore(&data->modbus_rx.fifo_lock, flags);
  return last_rx;
}

/* 安排接收队列的解析任务。
 * 返回：无。
 */
void ssf_mems_modbus_queue_parse(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_receive_state *rx;
  unsigned long flags;

  if (serdev == NULL)
    return;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return;
  rx = &data->modbus_rx;
  spin_lock_irqsave(&rx->fifo_lock, flags);
  if (rx->shutting_down == false && rx->flushing == false)
    queue_work(system_wq, &rx->work);
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
}

/* 判断窗口前缀的完整帧长度。
 * 返回：正数为帧长，0 等待帧头，负数为无效帧头。
 */
static int ssf_mems_rx_candidate_len(const u8 *buf, size_t available) {
  const struct ssf_modbus_frame_desc *desc;

  if (available < 2)
    return 0;
  desc = ssf_mems_modbus_find_rx_frame(buf[1]);
  if (desc == NULL)
    return -EOPNOTSUPP;
  if (desc->rx_bytes_per_reg != 0) {
    if (available <= desc->rx_byte_count_offset)
      return 0;
    return ssf_mems_modbus_rx_frame_len(desc, buf[desc->rx_byte_count_offset]);
  }
  return ssf_mems_modbus_rx_frame_len(desc, 0);
}

/* 校验完整候选帧并交给回调处理。
 * 返回：无。
 * 帧数据仅在回调期间有效。
 */
static void ssf_mems_rx_complete(struct ssf_mems_xyzs_data *data,
                                const u8 *buf, size_t len) {
  int ret = ssf_mems_modbus_check_crc(buf, len);

  if (ret != 0) {
    dev_err_ratelimited(&data->serdev->dev, "invalid frame CRC/length: %d\n", ret);
    return;
  }
  ret = data->modbus_rx.handler(data->modbus_rx.handler_context, buf, len);
  if (ret < 0)
    dev_err(&data->serdev->dev, "frame handler failed: %d\n", ret);
}

/* 解析普通 Modbus 字节并保留所有可能的帧头。
 * 返回：无。
 * 窗口最多一帧长；各帧头独立计时，伪帧头不会挤掉合法帧。
 */
static void ssf_mems_rx_process_modbus(struct ssf_mems_xyzs_data *data, u8 byte) {
  struct ssf_mems_modbus_receive_state *rx = &data->modbus_rx;
  size_t offset, keep;
  unsigned long expiry = ssf_mems_rx_expiry_ticks(data);
  int len;

  if (rx->frame_used == sizeof(rx->frame)) {
    rx->frame_used--;
    memmove(rx->frame, rx->frame + 1, rx->frame_used);
    memmove(rx->frame_started, rx->frame_started + 1,
            rx->frame_used * sizeof(rx->frame_started[0]));
  }
  rx->frame_started[rx->frame_used] = jiffies;
  rx->frame[rx->frame_used++] = byte;
  keep = rx->frame_used;
  for (offset = 0; offset < rx->frame_used; offset++) {
    size_t available = rx->frame_used - offset;

    if (rx->frame[offset] != data->slave_id ||
        time_after_eq(jiffies, rx->frame_started[offset] + expiry))
      continue;
    len = ssf_mems_rx_candidate_len(rx->frame + offset, available);
    if (len == 0 || (len > 0 && available < len)) {
      if (offset < keep)
        keep = offset;
    } else if (len > 0 && available == len) {
      ssf_mems_rx_complete(data, rx->frame + offset, available);
    }
  }
  /* 仅删除已完成、过期或不合法的前缀，保留仍有效的外层帧。 */
  if (keep != 0) {
    rx->frame_used -= keep;
    memmove(rx->frame, rx->frame + keep, rx->frame_used);
    memmove(rx->frame_started, rx->frame_started + keep,
            rx->frame_used * sizeof(rx->frame_started[0]));
  }
}

/* 按总线模式把一个字节交给对应解析器。
 * 返回：无。
 */
static void ssf_mems_rx_process_byte(struct ssf_mems_xyzs_data *data, u8 byte) {
  if (READ_ONCE(data->protocol.mode) == SSF_MEMS_LINK_OTA)
    ssf_mems_ota_receive_byte(data, byte);
  else if (READ_ONCE(data->raw.active) == true)
    ssf_mems_raw_receive_byte(data, byte);
  else
    ssf_mems_rx_process_modbus(data, byte);
}

/* 把串口字节放入接收队列。
 * 返回：已接收的字节数，负数为参数或接收状态错误。
 * 队列不足时允许部分入队，清理期间直接丢弃。
 */
ssize_t ssf_mems_rx_push(struct serdev_device *serdev,
                         const unsigned char *buf, size_t count) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_receive_state *rx;
  unsigned long flags;
  unsigned int ret;

  if (count == 0)
    return 0;
  if (serdev == NULL || buf == NULL)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;
  rx = &data->modbus_rx;
  spin_lock_irqsave(&rx->fifo_lock, flags);
  if (rx->shutting_down == true) {
    spin_unlock_irqrestore(&rx->fifo_lock, flags);
    return -ENODEV;
  }
  rx->last_rx = jiffies;
  if (rx->flushing == true) {
    spin_unlock_irqrestore(&rx->fifo_lock, flags);
    return (ssize_t)count;
  }
  ret = kfifo_in(&rx->fifo, buf, count);
  mod_delayed_work(system_wq, &rx->expiry_work, ssf_mems_rx_expiry_ticks(data));
  spin_unlock_irqrestore(&rx->fifo_lock, flags);
  ssf_mems_modbus_queue_parse(serdev);
  if (ret != count) {
    dev_err(&serdev->dev, "rx fifo overflow: received %zu bytes, stored %u bytes\n", count, ret);
  }
  return (ssize_t)ret;
}

/* 取出接收队列中的字节并逐个解析。
 * 返回：0 成功，-EINVAL 设备无效，-ENODEV 接收不可用。
 * 只由接收工作项调用。
 */
static int ssf_mems_modbus_receive_drain(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_receive_state *rx;
  u8 byte;

  if (serdev == NULL)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;
  rx = &data->modbus_rx;
  mutex_lock(&rx->parse_lock);
  if (READ_ONCE(rx->shutting_down) == true) {
    mutex_unlock(&rx->parse_lock);
    return -ENODEV;
  }
  while (READ_ONCE(rx->shutting_down) == false && READ_ONCE(rx->flushing) == false &&
         kfifo_out_spinlocked(&rx->fifo, &byte, sizeof(byte), &rx->fifo_lock) == sizeof(byte))
    ssf_mems_rx_process_byte(data, byte);
  mutex_unlock(&rx->parse_lock);
  return 0;
}
