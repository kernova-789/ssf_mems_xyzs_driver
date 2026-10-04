#include "modbus_request.h"
#include "core.h"
#include "modbus_table.h"

#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/slab.h>

/* 计算 Modbus RTU CRC16。 */
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

/* 在发送前检查整个范围，任何一个地址不满足条件都不发送。 */
static int ssf_mems_modbus_check_range(u16 display_reg, u16 reg_count,
                                     enum ssf_reg_access access,
                                     u16 *protocol_addr) {
  const struct ssf_reg_desc *first;
  const struct ssf_reg_desc *reg;
  u16 i;

  if (!reg_count || !protocol_addr ||
      (u32)display_reg + reg_count - 1 > 0xffffU)
    return -EINVAL;

  first = ssf_mems_modbus_find_reg(display_reg);
  if (!first)
    return -ENOENT;

  if ((u32)first->protocol_addr + reg_count - 1 > 0xffffU)
    return -EINVAL;

  for (i = 0; i < reg_count; i++) {
    reg = ssf_mems_modbus_find_reg(display_reg + i);
    if (!reg)
      return -ENOENT;

    if (!(reg->access & access))
      return -EACCES;

    if (reg->protocol_addr != (u32)first->protocol_addr + i)
      return -EINVAL;

    /* 特殊原始数据响应不能交给普通 0x03 寄存器解析器。 */
    if (access == SSF_REG_READ && reg->read_frame_type != SSF_FRAME_FIXED)
      return -EOPNOTSUPP;
  }

  *protocol_addr = first->protocol_addr;
  return 0;
}

/* 组装并发送 Modbus 0x03 读取请求帧。 */
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

/* 组装并发送 Modbus 0x06 单寄存器写请求帧。 */
static int ssf_mems_modbus_send_write_single_frame(struct serdev_device *serdev,
                                                   u8 slave_id,
                                                   u16 protocol_addr,
                                                   u16 value) {
  u8 frame[8];
  u16 crc;
  ssize_t ret;

  frame[0] = slave_id;
  frame[1] = SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE;
  frame[2] = protocol_addr >> 8;
  frame[3] = protocol_addr & 0xff;
  frame[4] = value >> 8;
  frame[5] = value & 0xff;

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

/* 组装并发送 Modbus 0x10 多寄存器写请求帧。 */
static int ssf_mems_modbus_send_write_multi_frame(struct serdev_device *serdev,
                                                  u8 slave_id,
                                                  u16 protocol_addr,
                                                  u16 reg_count,
                                                  const u16 *values) {
  u8 *frame;
  size_t frame_len;
  u16 crc;
  u16 i;
  ssize_t ret;

  frame_len = 9 + (size_t)reg_count * 2;
  frame = kmalloc(frame_len, GFP_KERNEL);
  if (!frame)
    return -ENOMEM;

  frame[0] = slave_id;
  frame[1] = SSF_MEMS_MODBUS_FUNC_WRITE_MULTI;
  frame[2] = protocol_addr >> 8;
  frame[3] = protocol_addr & 0xff;
  frame[4] = reg_count >> 8;
  frame[5] = reg_count & 0xff;
  frame[6] = reg_count * 2;

  for (i = 0; i < reg_count; i++) {
    frame[7 + i * 2] = values[i] >> 8;
    frame[8 + i * 2] = values[i] & 0xff;
  }

  crc = ssf_mems_modbus_crc16(frame, frame_len - 2);
  frame[frame_len - 2] = crc & 0xff;
  frame[frame_len - 1] = crc >> 8;

  ret = serdev_device_write(serdev, frame, frame_len,
                            msecs_to_jiffies(SSF_MEMS_MODBUS_WRITE_TIMEOUT_MS));
  if (ret < 0) {
    kfree(frame);
    return ret;
  }

  if (ret != frame_len) {
    kfree(frame);
    return -EIO;
  }

  serdev_device_wait_until_sent(
      serdev, msecs_to_jiffies(SSF_MEMS_MODBUS_WRITE_TIMEOUT_MS));

  kfree(frame);
  return 0;
}

/* 工作队列执行函数，负责调用帧解析函数。 */
static void ssf_mems_modbus_rx_workfn(struct work_struct *work) {
  struct ssf_mems_xyzs_data *data =
      container_of(work, struct ssf_mems_xyzs_data, rx_work);

  ssf_mems_modbus_parse_frame(data->serdev);
}

/* 将接收帧解析任务加入工作队列。 */
void ssf_mems_modbus_queue_parse(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data = serdev_device_get_drvdata(serdev);

  if (data)
    queue_work(system_wq, &data->rx_work);
}

/* 初始化 Modbus 请求状态和接收工作队列。 */
int ssf_mems_modbus_request_init(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_modbus_request_state *req = &data->modbus_req;

  mutex_init(&req->lock);
  init_waitqueue_head(&req->waitq);
  req->pending = false;
  req->shutting_down = false;
  req->slave_id = 0;
  req->function = 0;
  req->display_reg = 0;
  req->protocol_addr = 0;
  req->reg_count = 0;
  req->values = NULL;
  req->values_count = 0;
  req->write_value = 0;
  req->status = 0;
  INIT_WORK(&data->rx_work, ssf_mems_modbus_rx_workfn);

  return 0;
}

/* 驱动卸载时停止工作队列并结束正在等待的请求。 */
void ssf_mems_modbus_request_remove(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_modbus_request_state *req = &data->modbus_req;

  cancel_work_sync(&data->rx_work);

  mutex_lock(&req->lock);
  req->shutting_down = true;
  req->pending = false;
  req->values = NULL;
  req->values_count = 0;
  req->write_value = 0;
  req->status = -ENODEV;
  mutex_unlock(&req->lock);

  wake_up_interruptible(&req->waitq);
}

/* 结束当前请求并保存最终状态。 */
static void
ssf_mems_modbus_finish_request(struct ssf_mems_modbus_request_state *req,
                               int status) {
  req->pending = false;
  req->values = NULL;
  req->values_count = 0;
  req->write_value = 0;
  req->status = status;
}

/* 等待当前请求完成、超时或被信号打断。 */
static int
ssf_mems_modbus_wait_request(struct ssf_mems_modbus_request_state *req,
                             unsigned int timeout_ms) {
  unsigned long timeout;
  long wait_ret;
  int ret;

  timeout = msecs_to_jiffies(timeout_ms);
  mutex_unlock(&req->lock);

  wait_ret = wait_event_interruptible_timeout(
      req->waitq, !READ_ONCE(req->pending), timeout);

  mutex_lock(&req->lock);

  if (!req->pending) {
    ret = req->status;
  } else if (wait_ret == 0) {
    ssf_mems_modbus_finish_request(req, -ETIMEDOUT);
    ret = -ETIMEDOUT;
  } else {
    ssf_mems_modbus_finish_request(req, -ERESTARTSYS);
    ret = -ERESTARTSYS;
  }

  mutex_unlock(&req->lock);
  return ret;
}

/* 连续读取普通寄存器；整个范围通过可读权限检查后才发送 0x03。 */
int ssf_mems_modbus_read(struct serdev_device *serdev, u16 display_reg,
                         u16 reg_count, u16 *values, size_t values_count,
                         unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_request_state *req;
  u16 protocol_addr;
  int ret;

  if (!serdev || !values || !reg_count)
    return -EINVAL;

  if (values_count < reg_count)
    return -EINVAL;

  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;

  if (!timeout_ms)
    timeout_ms = SSF_MEMS_MODBUS_DEFAULT_TIMEOUT_MS;

  if (reg_count > SSF_MEMS_MODBUS_READ_MAX_REGS)
    return -EINVAL;

  ret = ssf_mems_modbus_check_range(display_reg, reg_count, SSF_REG_READ,
                                  &protocol_addr);
  if (ret)
    return ret;

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
  req->function = SSF_MEMS_MODBUS_FUNC_READ;
  req->display_reg = display_reg;
  req->protocol_addr = protocol_addr;
  req->reg_count = reg_count;
  req->values = values;
  req->values_count = values_count;
  req->write_value = 0;
  req->status = -ETIMEDOUT;
  req->pending = true;

  ret = ssf_mems_modbus_send_read_frame(serdev, req->slave_id,
                                        req->protocol_addr, req->reg_count);
  if (ret) {
    ssf_mems_modbus_finish_request(req, ret);
    goto out_unlock;
  }

  return ssf_mems_modbus_wait_request(req, timeout_ms);

out_unlock:
  mutex_unlock(&req->lock);
  return ret;
}

/* 读取单个寄存器的简化接口。 */
int ssf_mems_modbus_read_reg(struct serdev_device *serdev, u16 display_reg,
                             u16 *value, unsigned int timeout_ms) {
  return ssf_mems_modbus_read(serdev, display_reg, 1, value, 1, timeout_ms);
}

/* 发送一个 0x06 写请求，并等待对应的响应或超时。 */
int ssf_mems_modbus_write_reg(struct serdev_device *serdev, u16 display_reg,
                              u16 value, unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_request_state *req;
  const struct ssf_reg_desc *reg;
  int ret;

  if (!serdev)
    return -EINVAL;

  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;

  if (!timeout_ms)
    timeout_ms = SSF_MEMS_MODBUS_DEFAULT_TIMEOUT_MS;

  reg = ssf_mems_modbus_find_reg(display_reg);
  if (!reg)
    return -ENOENT;

  if (!(reg->access & SSF_REG_WRITE))
    return -EACCES;

  if (!reg->write_single)
    return -EOPNOTSUPP;

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
  req->function = SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE;
  req->display_reg = display_reg;
  req->protocol_addr = reg->protocol_addr;
  req->reg_count = 1;
  req->values = NULL;
  req->values_count = 0;
  req->write_value = value;
  req->status = -ETIMEDOUT;
  req->pending = true;

  ret = ssf_mems_modbus_send_write_single_frame(serdev, req->slave_id,
                                                req->protocol_addr, value);
  if (ret) {
    ssf_mems_modbus_finish_request(req, ret);
    goto out_unlock;
  }

  return ssf_mems_modbus_wait_request(req, timeout_ms);

out_unlock:
  mutex_unlock(&req->lock);
  return ret;
}

/* 连续写普通寄存器；整个范围通过可写权限检查后才发送 0x10。 */
int ssf_mems_modbus_write_regs(struct serdev_device *serdev, u16 display_reg,
                               u16 reg_count, const u16 *values,
                               size_t values_count, unsigned int timeout_ms) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_request_state *req;
  u16 protocol_addr;
  int ret;

  if (!serdev || !values || !reg_count)
    return -EINVAL;

  if (values_count < reg_count)
    return -EINVAL;

  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;

  if (!timeout_ms)
    timeout_ms = SSF_MEMS_MODBUS_DEFAULT_TIMEOUT_MS;

  if (reg_count > SSF_MEMS_MODBUS_WRITE_MAX_REGS)
    return -EINVAL;

  ret = ssf_mems_modbus_check_range(display_reg, reg_count, SSF_REG_WRITE,
                                  &protocol_addr);
  if (ret)
    return ret;

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
  req->function = SSF_MEMS_MODBUS_FUNC_WRITE_MULTI;
  req->display_reg = display_reg;
  req->protocol_addr = protocol_addr;
  req->reg_count = reg_count;
  req->values = NULL;
  req->values_count = 0;
  req->write_value = 0;
  req->status = -ETIMEDOUT;
  req->pending = true;

  ret = ssf_mems_modbus_send_write_multi_frame(
      serdev, req->slave_id, req->protocol_addr, req->reg_count, values);
  if (ret) {
    ssf_mems_modbus_finish_request(req, ret);
    goto out_unlock;
  }

  return ssf_mems_modbus_wait_request(req, timeout_ms);

out_unlock:
  mutex_unlock(&req->lock);
  return ret;
}

/* 手册规定的块写命令：固定写入 40061～40070 十个工作参数。 */
int ssf_mems_modbus_write_work_parameters(struct serdev_device *serdev,
                                         const u16 *values,
                                         size_t values_count,
                                         unsigned int timeout_ms) {
  const struct ssf_block_cmd_desc *cmd;

  cmd = ssf_mems_modbus_find_block_cmd(SSF_BLOCK_WRITE_WORK_PARAMETERS);
  if (!cmd)
    return -ENOENT;

  if (cmd->function != SSF_MEMS_MODBUS_FUNC_WRITE_MULTI ||
      cmd->direction != SSF_CMD_WRITE || cmd->frame_type != SSF_FRAME_FIXED)
    return -EOPNOTSUPP;

  return ssf_mems_modbus_write_regs(serdev, cmd->start_display_reg,
                                  cmd->reg_count, values, values_count,
                                  timeout_ms);
}

/* 判断完整响应帧是否属于当前请求，并在读取请求中保存寄存器数据。 */
bool ssf_mems_modbus_claim_frame(struct ssf_mems_xyzs_data *data,
                                 struct ssf_mems_frame_slot *slot) {
  struct ssf_mems_modbus_request_state *req = &data->modbus_req;
  const u8 *frame = slot->data;
  u8 byte_count;
  u16 addr;
  u16 count;
  u16 value;
  u16 i;

  if (!frame)
    return false;

  mutex_lock(&req->lock);

  if (!req->pending) {
    mutex_unlock(&req->lock);
    return false;
  }

  if (frame[0] != req->slave_id || frame[1] != req->function) {
    mutex_unlock(&req->lock);
    return false;
  }

  if (req->function == SSF_MEMS_MODBUS_FUNC_READ) {
    if (slot->data_len < 5) {
      mutex_unlock(&req->lock);
      return false;
    }

    byte_count = frame[2];
    if (byte_count != req->reg_count * 2 ||
        slot->data_len != (size_t)byte_count + 5) {
      mutex_unlock(&req->lock);
      return false;
    }

    if (!req->values || req->values_count < req->reg_count) {
      ssf_mems_modbus_finish_request(req, -EFAULT);
      mutex_unlock(&req->lock);
      wake_up_interruptible(&req->waitq);
      return true;
    }

    for (i = 0; i < req->reg_count; i++)
      req->values[i] = ((u16)frame[3 + i * 2] << 8) | frame[4 + i * 2];

    ssf_mems_modbus_finish_request(req, 0);
    mutex_unlock(&req->lock);
    wake_up_interruptible(&req->waitq);
    return true;
  }

  if (slot->data_len != 8) {
    mutex_unlock(&req->lock);
    return false;
  }

  addr = ((u16)frame[2] << 8) | frame[3];
  if (addr != req->protocol_addr) {
    mutex_unlock(&req->lock);
    return false;
  }

  if (req->function == SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE) {
    value = ((u16)frame[4] << 8) | frame[5];
    if (value != req->write_value) {
      mutex_unlock(&req->lock);
      return false;
    }

    ssf_mems_modbus_finish_request(req, 0);
    mutex_unlock(&req->lock);
    wake_up_interruptible(&req->waitq);
    return true;
  }

  if (req->function == SSF_MEMS_MODBUS_FUNC_WRITE_MULTI) {
    count = ((u16)frame[4] << 8) | frame[5];
    if (count != req->reg_count) {
      mutex_unlock(&req->lock);
      return false;
    }

    ssf_mems_modbus_finish_request(req, 0);
    mutex_unlock(&req->lock);
    wake_up_interruptible(&req->waitq);
    return true;
  }

  mutex_unlock(&req->lock);
  return false;
}
