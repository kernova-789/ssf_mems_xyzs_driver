#include "modbus_request.h"
#include "core.h"
#include "modbus.h"
#include "modbus_table.h"

#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

/* 返回 0 请求描述已生成；-EINVAL 参数/范围错误，-ENOENT 地址未入表，-EACCES 权限不足，-EOPNOTSUPP 帧或块命令不支持。 */
int ssf_mems_modbus_plan_request(u8 function, u16 display_reg, u16 reg_count,
                                const struct ssf_block_cmd_desc *block,
                                struct ssf_modbus_transfer *transfer) {
  const struct ssf_modbus_frame_desc *frame;
  const struct ssf_reg_desc *first;
  unsigned int access;
  u16 i;

  if (!transfer || !reg_count ||
      (u32)display_reg + reg_count > 0x10000U)
    return -EINVAL;

  frame = ssf_mems_modbus_find_frame(function);
  if (!frame)
    return -EOPNOTSUPP;
  if (reg_count > frame->max_reg_count)
    return -EINVAL;

  access = frame->tx_format == SSF_TX_READ_REGS ? SSF_REG_READ : SSF_REG_WRITE;
  if (block) {
    if (block->function != function || block->rx_format != frame->rx_format ||
        block->direction != (access == SSF_REG_READ ? SSF_CMD_READ : SSF_CMD_WRITE))
      return -EOPNOTSUPP;
    if (!block->reg_count || (u32)block->start_display_reg + block->reg_count > 0x10000U ||
        display_reg < block->start_display_reg ||
        (u32)display_reg + reg_count > (u32)block->start_display_reg + block->reg_count)
      return -EINVAL;
    if (!block->sparse_read &&
        (display_reg != block->start_display_reg || reg_count != block->reg_count))
      return -EINVAL;
  }

  first = ssf_mems_modbus_find_reg(display_reg);
  if (!first)
    return -ENOENT;
  if ((u32)first->protocol_addr + reg_count > 0x10000U)
    return -EINVAL;

  for (i = 0; i < reg_count; i++) {
    const struct ssf_reg_desc *reg = ssf_mems_modbus_find_reg(display_reg + i);

    if (!reg)
      return -ENOENT;
    if (!(reg->access & access))
      return -EACCES;
    if (reg->protocol_addr != (u32)first->protocol_addr + i)
      return -EINVAL;
    if (access == SSF_REG_READ && reg->read_format != frame->rx_format)
      return -EOPNOTSUPP;
    if (frame->tx_format == SSF_TX_WRITE_SINGLE && !reg->write_single)
      return -EOPNOTSUPP;
  }

  transfer->frame = frame;
  transfer->block = block;
  transfer->display_reg = display_reg;
  transfer->protocol_addr = first->protocol_addr;
  transfer->reg_count = reg_count;
  return 0;
}

/* 按实际串口格式估算线上发送时间，并预留调度/控制器余量。 */
static unsigned int
ssf_mems_modbus_write_timeout_ms(const struct ssf_mems_xyzs_data *data,
                                 size_t len) {
  unsigned int baudrate = READ_ONCE(data->protocol.host_baudrate);
  unsigned int bits_per_char =
      READ_ONCE(data->protocol.parity) == SSF_MEMS_PARITY_NONE ? 10U : 11U;
  unsigned int wire_ms;
  unsigned long long bit_ms;

  if (!baudrate)
    baudrate = 9600;
  bit_ms = (unsigned long long)len * bits_per_char * 1000U;
  wire_ms = (bit_ms + baudrate - 1) / baudrate;
  wire_ms += SSF_MEMS_MODBUS_WRITE_MARGIN_MS;
  if (wire_ms < SSF_MEMS_MODBUS_MIN_WRITE_TIMEOUT_MS)
    wire_ms = SSF_MEMS_MODBUS_MIN_WRITE_TIMEOUT_MS;
  return wire_ms;
}

/* 按请求描述组帧并发送；返回 0 完整发送，-ENOMEM 分配失败，-EIO 写入不足；其他负值来自组帧或串口。 */
static int ssf_mems_modbus_send_request(
    struct ssf_mems_xyzs_data *data,
    const struct ssf_modbus_transfer *transfer, u8 slave_id,
    const u16 *values, size_t values_count) {
  struct serdev_device *serdev = data->serdev;
  unsigned int write_timeout_ms;
  u8 *buf;
  int len;
  int ret;
  ssize_t written;

  len = ssf_mems_modbus_tx_frame_len(transfer->frame, transfer->reg_count);
  if (len < 0)
    return len;
  buf = kmalloc(len, GFP_KERNEL);
  if (!buf)
    return -ENOMEM;

  ret = ssf_mems_modbus_build_request(transfer, slave_id, values, values_count, buf, len);
  if (ret < 0)
    goto out;
  write_timeout_ms = ssf_mems_modbus_write_timeout_ms(data, len);
  written = serdev_device_write(serdev, buf, len,
                                msecs_to_jiffies(write_timeout_ms));
  if (written < 0) {
    ret = written;
    goto out;
  }
  if (written != len) {
    ret = -EIO;
    goto out;
  }

  serdev_device_wait_until_sent(serdev, msecs_to_jiffies(write_timeout_ms));
  ret = 0;
out:
  kfree(buf);
  return ret;
}

/* 初始化请求状态及其锁和等待队列；返回 0 成功，-EINVAL 驱动数据为空。 */
int ssf_mems_modbus_request_init(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_modbus_request_state *req;

  if (!data)
    return -EINVAL;
  req = &data->modbus_req;
  mutex_init(&req->lock);
  init_waitqueue_head(&req->waitq);
  req->busy = false;
  req->pending = false;
  req->shutting_down = false;
  req->slave_id = 0;
  memset(&req->transfer, 0, sizeof(req->transfer));
  req->values = NULL;
  req->values_count = 0;
  req->write_value = 0;
  req->status = 0;
  return 0;
}

/* 关闭请求模块并唤醒等待者；无返回值，当前请求状态设为 -ENODEV，空指针直接退出。 */
void ssf_mems_modbus_request_remove(struct ssf_mems_xyzs_data *data) {
  struct ssf_mems_modbus_request_state *req;

  if (!data)
    return;
  req = &data->modbus_req;
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

/* 完成响应并保存 status；无返回值，busy 保留到等待者取走结果，避免新请求覆盖旧结果。 */
static void ssf_mems_modbus_finish_request(struct ssf_mems_modbus_request_state *req,
                                          int status) {
  req->pending = false;
  req->values = NULL;
  req->values_count = 0;
  req->write_value = 0;
  req->status = status;
}

/* 返回请求的最终 status，-ETIMEDOUT 超时，-ERESTARTSYS 信号打断；取走结果后释放请求占用。 */
static int ssf_mems_modbus_wait_request(struct ssf_mems_modbus_request_state *req,
                                       unsigned int timeout_ms) {
  long wait_ret;
  int ret;

  mutex_unlock(&req->lock);
  wait_ret = wait_event_interruptible_timeout(req->waitq, !READ_ONCE(req->pending),
                                               msecs_to_jiffies(timeout_ms));
  mutex_lock(&req->lock);
  if (!req->pending) {
    ret = req->status;
  } else {
    ret = wait_ret == 0 ? -ETIMEDOUT : -ERESTARTSYS;
    ssf_mems_modbus_finish_request(req, ret);
  }
  req->busy = false;
  mutex_unlock(&req->lock);
  return ret;
}

/* 执行已校验请求；返回 0 成功，-EINVAL 空设备，-ENODEV 无驱动数据/关闭中，-EBUSY 请求占用；其他负值来自发送、响应或等待。 */
static int ssf_mems_modbus_execute(struct serdev_device *serdev,
                                  const struct ssf_modbus_transfer *transfer,
                                  const u16 *tx_values, size_t tx_values_count,
                                  u16 *rx_values, size_t rx_values_count,
                                  unsigned int timeout_ms,
                                  bool bus_locked) {
  struct ssf_mems_xyzs_data *data;
  struct ssf_mems_modbus_request_state *req;
  int ret;

  if (!serdev)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (!data)
    return -ENODEV;
  if (!timeout_ms)
    timeout_ms = SSF_MEMS_MODBUS_DEFAULT_TIMEOUT_MS;
  if (!bus_locked)
    mutex_lock(&data->protocol.bus_lock);

  req = &data->modbus_req;
  mutex_lock(&req->lock);
  if (req->shutting_down) {
    ret = -ENODEV;
    goto out_unlock;
  }
  if (req->busy) {
    ret = -EBUSY;
    goto out_unlock;
  }

  req->transfer = *transfer;
  req->slave_id = data->slave_id;
  req->values = rx_values;
  req->values_count = rx_values_count;
  req->write_value = transfer->frame->tx_format == SSF_TX_WRITE_SINGLE ? tx_values[0] : 0;
  req->status = -ETIMEDOUT;
  req->busy = true;
  req->pending = true;

  ret = ssf_mems_modbus_send_request(data, &req->transfer, req->slave_id,
                                     tx_values, tx_values_count);
  if (ret) {
    ssf_mems_modbus_finish_request(req, ret);
    req->busy = false;
    goto out_unlock;
  }
  ret = ssf_mems_modbus_wait_request(req, timeout_ms);
  goto out_bus_unlock;

out_unlock:
  mutex_unlock(&req->lock);
out_bus_unlock:
  if (!bus_locked)
    mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

/* 读取块内实际保留的连续段；返回 0 成功，-EINVAL 参数/范围错误，-ENOENT 地址缺失，-EACCES 不可读，-EOPNOTSUPP 格式不支持；其他负值同通用读取。 */
static int ssf_mems_modbus_read_block_range_common(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    u16 display_reg, u16 reg_count, u16 *values, size_t values_count,
    unsigned int timeout_ms, bool bus_locked) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (!serdev || !values || values_count < reg_count)
    return -EINVAL;
  ret = ssf_mems_modbus_plan_request(SSF_MEMS_MODBUS_FUNC_READ, display_reg,
                                     reg_count, block, &transfer);
  if (ret)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, NULL, 0, values,
                                 values_count, timeout_ms, bus_locked);
}

int ssf_mems_modbus_read_block_range(struct serdev_device *serdev,
                                     const struct ssf_block_cmd_desc *block,
                                     u16 display_reg, u16 reg_count,
                                     u16 *values, size_t values_count,
                                     unsigned int timeout_ms) {
  return ssf_mems_modbus_read_block_range_common(
      serdev, block, display_reg, reg_count, values, values_count, timeout_ms,
      false);
}

int ssf_mems_modbus_read_block_range_locked(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    u16 display_reg, u16 reg_count, u16 *values, size_t values_count,
    unsigned int timeout_ms) {
  return ssf_mems_modbus_read_block_range_common(
      serdev, block, display_reg, reg_count, values, values_count, timeout_ms,
      true);
}

/* 连续读取普通寄存器；返回 0 成功，-EINVAL 参数/范围错误，-ENOENT 地址缺失，-EACCES 不可读，-EOPNOTSUPP 特殊响应，-ENODEV 无设备/关闭中，-EBUSY 请求占用，-ENOMEM 分配失败，-EIO 发送不足，-EREMOTEIO 从机异常，-EFAULT 结果缓冲区异常，-ETIMEDOUT 超时，-ERESTARTSYS 信号打断；其他负值来自组帧或串口。 */
int ssf_mems_modbus_read(struct serdev_device *serdev, u16 display_reg,
                         u16 reg_count, u16 *values, size_t values_count,
                         unsigned int timeout_ms) {
  return ssf_mems_modbus_read_block_range(serdev, NULL, display_reg, reg_count,
                                          values, values_count, timeout_ms);
}

/* 返回 0 单寄存器读取成功，失败原因和负值同 ssf_mems_modbus_read()。 */
int ssf_mems_modbus_read_reg(struct serdev_device *serdev, u16 display_reg,
                             u16 *value, unsigned int timeout_ms) {
  return ssf_mems_modbus_read(serdev, display_reg, 1, value, 1, timeout_ms);
}

int ssf_mems_modbus_read_reg_locked(struct serdev_device *serdev,
                                    u16 display_reg, u16 *value,
                                    unsigned int timeout_ms) {
  return ssf_mems_modbus_read_block_range_common(
      serdev, NULL, display_reg, 1, value, 1, timeout_ms, true);
}

/* 返回 0 单寄存器写入成功，-EOPNOTSUPP 表项不支持 0x06；其余校验、发送、响应和等待错误同多寄存器写入。 */
static int ssf_mems_modbus_write_reg_common(struct serdev_device *serdev,
                                            u16 display_reg, u16 value,
                                            unsigned int timeout_ms,
                                            bool bus_locked) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (!serdev)
    return -EINVAL;
  ret = ssf_mems_modbus_plan_request(SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE,
                                     display_reg, 1, NULL, &transfer);
  if (ret)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, &value, 1, NULL, 0,
                                 timeout_ms, bus_locked);
}

int ssf_mems_modbus_write_reg(struct serdev_device *serdev, u16 display_reg,
                              u16 value, unsigned int timeout_ms) {
  return ssf_mems_modbus_write_reg_common(serdev, display_reg, value,
                                          timeout_ms, false);
}

int ssf_mems_modbus_write_reg_locked(struct serdev_device *serdev,
                                     u16 display_reg, u16 value,
                                     unsigned int timeout_ms) {
  return ssf_mems_modbus_write_reg_common(serdev, display_reg, value,
                                          timeout_ms, true);
}

/* 返回 0 连续写入成功，-EINVAL 参数/范围错误，-ENOENT 地址缺失，-EACCES 不可写，-EOPNOTSUPP 帧格式不支持，-ENODEV 无设备/关闭中，-EBUSY 请求占用，-ENOMEM 分配失败，-EIO 发送不足，-EREMOTEIO 从机异常，-ETIMEDOUT 超时，-ERESTARTSYS 信号打断；其他负值来自组帧或串口。 */
int ssf_mems_modbus_write_regs(struct serdev_device *serdev, u16 display_reg,
                               u16 reg_count, const u16 *values,
                               size_t values_count, unsigned int timeout_ms) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (!serdev || !values || values_count < reg_count)
    return -EINVAL;
  ret = ssf_mems_modbus_plan_request(SSF_MEMS_MODBUS_FUNC_WRITE_MULTI,
                                     display_reg, reg_count, NULL, &transfer);
  if (ret)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, values, values_count,
                                 NULL, 0, timeout_ms, false);
}

/* 按表中块描述执行写入；返回 0 成功，-EINVAL 参数/范围错误，-EOPNOTSUPP 不是可写块或格式不支持；其他负值同连续写入。 */
static int ssf_mems_modbus_write_block_common(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    const u16 *values, size_t values_count, unsigned int timeout_ms,
    bool bus_locked) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (!serdev || !block || !values || values_count < block->reg_count)
    return -EINVAL;
  if (block->direction != SSF_CMD_WRITE)
    return -EOPNOTSUPP;
  ret = ssf_mems_modbus_plan_request(block->function, block->start_display_reg,
                                     block->reg_count, block, &transfer);
  if (ret)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, values, values_count,
                                 NULL, 0, timeout_ms, bus_locked);
}

int ssf_mems_modbus_write_block(struct serdev_device *serdev,
                                const struct ssf_block_cmd_desc *block,
                                const u16 *values, size_t values_count,
                                unsigned int timeout_ms) {
  return ssf_mems_modbus_write_block_common(
      serdev, block, values, values_count, timeout_ms, false);
}

int ssf_mems_modbus_write_block_locked(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    const u16 *values, size_t values_count, unsigned int timeout_ms) {
  return ssf_mems_modbus_write_block_common(
      serdev, block, values, values_count, timeout_ms, true);
}

/* 返回 true 帧已消费（成功、从机异常或 -EFAULT），false 无等待请求、响应不匹配或帧损坏；true 不代表请求成功，不依赖接收槽位。 */
bool ssf_mems_modbus_claim_frame(struct ssf_mems_xyzs_data *data,
                                 const u8 *buf, size_t len) {
  struct ssf_mems_modbus_request_state *req;
  int ret;

  if (!data || !buf)
    return false;
  req = &data->modbus_req;
  mutex_lock(&req->lock);
  if (!req->pending) {
    mutex_unlock(&req->lock);
    return false;
  }

  ret = ssf_mems_modbus_parse_response(&req->transfer, req->slave_id,
                                       req->write_value, buf, len,
                                       req->values, req->values_count);
  if (ret == -ENOMSG || ret == -EMSGSIZE || ret == -EBADMSG || ret == -EINVAL) {
    mutex_unlock(&req->lock);
    return false;
  }

  ssf_mems_modbus_finish_request(req, ret);
  mutex_unlock(&req->lock);
  wake_up_interruptible(&req->waitq);
  return true;
}
