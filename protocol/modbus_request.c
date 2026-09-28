#include "modbus_request.h"
#include "core.h"
#include "modbus_table.h"

#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

static u16 ssf_mems_modbus_crc16(const u8 *buf, size_t len) {
  u16 crc = 0xFFFF;
  while (len--) {
    int i;
    crc ^= *buf++;
    for (i = 0; i < 8; i++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

/* 根据寄存器号和寄存器数量，在 modbus_table.h 中找到对应的命令 */
static const struct ssf_cmd_desc *ssf_mems_modbus_find_read_cmd(u16 display_reg,
                                                                u16 reg_count) {
  size_t i;
  for (i = 0; i < SSF_CMD_TABLE_SIZE; i++) {
    const struct ssf_cmd_desc *cmd = &ssf_cmd_table[i];
    if (cmd->function == SSF_MEMS_MODBUS_FUNC_READ &&
        cmd->direction == SSF_CMD_READ && cmd->display_reg == display_reg &&
        cmd->reg_count == reg_count)
      return cmd;
  }
  return NULL;
}

/* 组装并发送 Modbus 0x03 读取请求帧 */  
static int ssf_mems_modbus_send_read_frame(struct serdev_device *serdev,
                                           u8 slave_id, u16 protocol_addr,
                                           u16 reg_count) {
  u8 frame[8];
  u16 crc;
  ssize_t ret;

  frame[0] = slave_id;
  frame[1] = SSF_MEMS_MODBUS_FUNC_READ;
  frame[2] = protocol_addr >> 8;
  frame[3] = protocol_addr & 0xff;
  frame[4] = reg_count >> 8;
  frame[5] = reg_count & 0xff;
  crc = ssf_mems_modbus_crc16(frame, 6);
  frame[6] = crc & 0xff;
  frame[7] = crc >> 8;

  ret = serdev_device_write(serdev, frame, sizeof(frame),
                            msecs_to_jiffies(SSF_MEMS_MODBUS_WRITE_TIMEOUT_MS));
  if (ret < 0)
    return ret;
  if (ret != sizeof(frame))
    return -EIO;

  serdev_device_wait_until_sent(
      serdev, msecs_to_jiffies(SSF_MEMS_MODBUS_WRITE_TIMEOUT_MS));
  return 0;
}

/* 工作队列真正执行的函数 */
static void ssf_mems_modbus_rx_workfn(struct work_struct *work) {
  struct ssf_mems_xyzs_data *data =
      container_of(work, struct ssf_mems_xyzs_data, rx_work);
  ssf_mems_modbus_parse_frame(data->serdev);
}

/* 把“解析接收数据”这个工作加入工作队列 */
void ssf_mems_modbus_queue_parse(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data = serdev_device_get_drvdata(serdev);
  if (data)
    queue_work(system_wq, &data->rx_work);
}

/* 初始化 Modbus 请求状态 */
int ssf_mems_modbus_request_init(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_modbus_request_state *req = &data->modbus_req;
  mutex_init(&req->lock);
  init_waitqueue_head(&req->waitq);
  req->pending = false;
  req->shutting_down = false;
  req->values = NULL;
  req->values_count = 0;
  req->status = 0;
  INIT_WORK(&data->rx_work, ssf_mems_modbus_rx_workfn);
  return 0;
}
/* 驱动卸载时清理请求状态 */
void ssf_mems_modbus_request_remove(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_modbus_request_state *req = &data->modbus_req;
  cancel_work_sync(&data->rx_work);
  mutex_lock(&req->lock);
  req->shutting_down = true;
  req->pending = false;
  req->values = NULL;
  req->values_count = 0;
  req->status = -ENODEV;
  mutex_unlock(&req->lock);
  wake_up_interruptible(&req->waitq);
}

/* 发送一个 0x03 读取请求，并等待对应的响应 */
int ssf_mems_modbus_read(struct serdev_device *serdev, u16 display_reg,
                         u16 reg_count, u16 *values, unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data = serdev_device_get_drvdata(serdev);
  struct ssf_mems_modbus_request_state *req;
  const struct ssf_cmd_desc *cmd;
  unsigned long timeout;
  long wait_ret;
  int ret;

  if (!data || !values || !reg_count){
    return -EINVAL;}
  if (!timeout_ms){
    timeout_ms = SSF_MEMS_MODBUS_DEFAULT_TIMEOUT_MS;}

  cmd = ssf_mems_modbus_find_read_cmd(display_reg, reg_count);
  if (!cmd){
    return -EINVAL;}
  if (cmd->frame_type != SSF_FRAME_FIXED){
    return -EOPNOTSUPP;}
  if (reg_count > 125){
    return -EINVAL;}

  req = &data->modbus_req;
  mutex_lock(&req->lock);

  if (req->shutting_down) {
    ret = -ENODEV;
    goto out_unlock;
  }
  if (req->pending) {
    ret = -EBUSY;
    goto out_unlock;
  }

  req->slave_id = SSF_MEMS_MODBUS_DEFAULT_SLAVE_ID;
  req->function = cmd->function;
  req->display_reg = display_reg;
  req->protocol_addr = cmd->protocol_addr;
  req->reg_count = reg_count;
  req->values = values;
  req->values_count = reg_count;
  req->status = -ETIMEDOUT;
  req->pending = true;

  ret = ssf_mems_modbus_send_read_frame(serdev, req->slave_id,
                                        req->protocol_addr, req->reg_count);
  if (ret) {
    req->pending = false;
    req->values = NULL;
    req->values_count = 0;
    goto out_unlock;
  }

  timeout = msecs_to_jiffies(timeout_ms);
  mutex_unlock(&req->lock);

  wait_ret = wait_event_interruptible_timeout(
      req->waitq, !READ_ONCE(req->pending), timeout);

  mutex_lock(&req->lock);
  if (!req->pending) {
    ret = req->status;
  } else if (wait_ret == 0) {
    req->pending = false;
    req->values = NULL;
    req->values_count = 0;
    req->status = -ETIMEDOUT;
    ret = -ETIMEDOUT;
  } else {
    req->pending = false;
    req->values = NULL;
    req->values_count = 0;
    req->status = -ERESTARTSYS;
    ret = -ERESTARTSYS;
  }
  mutex_unlock(&req->lock);
  return ret;

out_unlock:
  mutex_unlock(&req->lock);
  return ret;
}

/* 读取单个寄存器的简化接口 */
int ssf_mems_modbus_read_reg(struct serdev_device *serdev, u16 display_reg,
                             u16 *value, unsigned int timeout_ms) {
  return ssf_mems_modbus_read(serdev, display_reg, 1, value, timeout_ms);
}
/* 判断收到的完整帧是不是当前正在等待的请求，如果是就“认领”它 */
int ssf_mems_modbus_claim_frame(struct ssf_mems_xyzs_data *data,
                                struct ssf_mems_frame_slot *slot) {
  struct ssf_mems_modbus_request_state *req = &data->modbus_req;
  const u8 *frame = slot->data;
  u8 byte_count;
  u16 i;

  if (!frame || slot->data_len < 5)
    return 0;

  mutex_lock(&req->lock);
  if (!req->pending) {
    mutex_unlock(&req->lock);
    return 0;
  }

  if (frame[0] != req->slave_id || frame[1] != req->function) {
    mutex_unlock(&req->lock);
    return 0;
  }

  byte_count = frame[2];
  if (byte_count != req->reg_count * 2 ||
      slot->data_len != (size_t)byte_count + 5) {
    mutex_unlock(&req->lock);
    return 0;
  }

  if (!req->values || req->values_count < req->reg_count) {
    req->pending = false;
    req->values = NULL;
    req->values_count = 0;
    req->status = -EFAULT;
    mutex_unlock(&req->lock);
    wake_up_interruptible(&req->waitq);
    return 1;
  }

  for (i = 0; i < req->reg_count; i++)
    req->values[i] = ((u16)frame[3 + i * 2] << 8) | frame[4 + i * 2];

  req->pending = false;
  req->values = NULL;
  req->values_count = 0;
  req->status = 0;
  mutex_unlock(&req->lock);
  wake_up_interruptible(&req->waitq);
  return 1;
}
