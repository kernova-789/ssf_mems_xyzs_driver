#include "modbus/modbus_request.h"
#include "transport/serial_transport.h"
#include "core.h"
#include "modbus/modbus.h"
#include "model/modbus_table.h"
#include "modbus/modbus_receive.h"

#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

/* 仅从当前实例取型号，不使用全局默认表。 */
static const struct ssf_mems_sensor *
ssf_mems_modbus_sensor(struct serdev_device *serdev) {
  struct ssf_mems_data *data;

  if (serdev == NULL)
    return NULL;
  data = serdev_device_get_drvdata(serdev);
  return data ? data->sensor : NULL;
}

/* 查表校验请求并生成传输描述。
 * 返回：0 成功，负数为参数、缺失表项、权限或格式错误。
 */
int ssf_mems_modbus_plan_request(const struct ssf_mems_sensor *sensor, u8 function, u16 display_reg, u16 reg_count,
                                const struct ssf_block_cmd_desc *block,
                                struct ssf_modbus_transfer *transfer) {
  const struct ssf_modbus_frame_desc *frame;
  const struct ssf_reg_desc *first;
  unsigned int access;
  u16 i;

  if (transfer == NULL || reg_count == 0 ||
      (u32)display_reg + reg_count > 0x10000U)
    return -EINVAL;

  if (sensor == NULL)
    return -ENODEV;
  frame = ssf_mems_modbus_find_frame(function);
  if (frame == NULL)
    return -EOPNOTSUPP;
  if (reg_count > frame->max_reg_count)
    return -EINVAL;

  access = frame->tx_format == SSF_TX_READ_REGS ? SSF_REG_READ : SSF_REG_WRITE;
  if (block != NULL) {
    if (block->function != function || block->rx_format != frame->rx_format ||
        block->direction != (access == SSF_REG_READ ? SSF_CMD_READ : SSF_CMD_WRITE))
      return -EOPNOTSUPP;
    if (block->reg_count == 0 || (u32)block->start_display_reg + block->reg_count > 0x10000U ||
        display_reg < block->start_display_reg ||
        (u32)display_reg + reg_count > (u32)block->start_display_reg + block->reg_count)
      return -EINVAL;
    if (block->sparse_read == false &&
        (display_reg != block->start_display_reg || reg_count != block->reg_count))
      return -EINVAL;
  }

  first = ssf_mems_sensor_find_reg(sensor, display_reg);
  if (first == NULL)
    return -ENOENT;
  if ((u32)first->protocol_addr + reg_count > 0x10000U)
    return -EINVAL;

  for (i = 0; i < reg_count; i++) {
    const struct ssf_reg_desc *reg = ssf_mems_sensor_find_reg(sensor, display_reg + i);

    if (reg == NULL)
      return -ENOENT;
    if ((reg->access & access) == 0)
      return -EACCES;
    if (reg->protocol_addr != (u32)first->protocol_addr + i)
      return -EINVAL;
    if (access == SSF_REG_READ && reg->read_format != frame->rx_format)
      return -EOPNOTSUPP;
    if (frame->tx_format == SSF_TX_WRITE_SINGLE && reg->write_single == false)
      return -EOPNOTSUPP;
  }

  transfer->frame = frame;
  transfer->block = block;
  transfer->display_reg = display_reg;
  transfer->protocol_addr = first->protocol_addr;
  transfer->reg_count = reg_count;
  return 0;
}

/* 计算串口发送的超时时间。
 * 返回：超时毫秒数，至少 100 ms。
 */
static unsigned int
ssf_mems_modbus_write_timeout_ms(const struct ssf_mems_data *data,
                                 size_t len) {
  return max(SSF_MEMS_MODBUS_MIN_WRITE_TIMEOUT_MS,
             ssf_mems_serial_wire_ms(data, len) + SSF_MEMS_MODBUS_WRITE_MARGIN_MS);
}

/* 组帧并发送一个 Modbus 请求。
 * 返回：0 完整发送，负数为组帧、分配或串口错误。
 */
static int ssf_mems_modbus_send_request(
    struct ssf_mems_data *data,
    const struct ssf_modbus_transfer *transfer, u8 slave_id,
    const u16 *values, size_t values_count) {
  unsigned int write_timeout_ms;
  u8 *buf;
  int len;
  int ret;

  len = ssf_mems_modbus_tx_frame_len(transfer->frame, transfer->reg_count);
  if (len < 0)
    return len;
  buf = kmalloc(len, GFP_KERNEL);
  if (buf == NULL)
    return -ENOMEM;

  ret = ssf_mems_modbus_build_request(transfer, slave_id, values, values_count, buf, len);
  if (ret < 0)
    goto out;
  write_timeout_ms = ssf_mems_modbus_write_timeout_ms(data, len);
  ret = ssf_mems_serial_write(data, buf, len, write_timeout_ms);
out:
  kfree(buf);
  return ret;
}

/* 初始化请求状态和等待队列。
 * 返回：0 成功，-EINVAL 设备无效。
 */
int ssf_mems_modbus_request_init(struct ssf_mems_data *data) {
  struct ssf_mems_modbus_request_state *req;

  if (data == NULL)
    return -EINVAL;
  req = &data->modbus_req;
  mutex_init(&req->lock);
  init_waitqueue_head(&req->waitq);
  req->busy = false;
  req->pending = false;
  req->shutting_down = false;
  req->quarantine_until = jiffies;
  req->slave_id = 0;
  memset(&req->transfer, 0, sizeof(req->transfer));
  req->values = NULL;
  req->values_count = 0;
  req->write_value = 0;
  req->status = 0;
  return 0;
}

/* 关闭请求模块并唤醒等待者。
 * 返回：无。
 * 在途请求以 -ENODEV 结束。
 */
void ssf_mems_modbus_request_remove(struct ssf_mems_data *data) {
  struct ssf_mems_modbus_request_state *req;

  if (data == NULL)
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

/* 保存请求结果并解除应答数据绑定。
 * 返回：无。
 * 调用者负责唤醒；等待者取走结果后才释放请求占用。
 */
static void ssf_mems_modbus_finish_request(struct ssf_mems_modbus_request_state *req,
                                          int status) {
  req->pending = false;
  req->values = NULL;
  req->values_count = 0;
  req->write_value = 0;
  req->status = status;
}

/* 等待应答并取走请求结果。
 * 返回：0 成功，负数为应答错误、停止、超时或信号中断。
 */
static int ssf_mems_modbus_wait_request(struct ssf_mems_modbus_request_state *req,
                                       unsigned int timeout_ms) {
  long wait_ret;
  int ret;

  mutex_unlock(&req->lock);
  wait_ret = wait_event_interruptible_timeout(req->waitq, READ_ONCE(req->pending) == false,
                                               msecs_to_jiffies(timeout_ms));
  mutex_lock(&req->lock);
  if (req->pending == false) {
    ret = req->status;
  } else {
    ret = wait_ret == 0 ? -ETIMEDOUT : -ERESTARTSYS;
    ssf_mems_modbus_finish_request(req, ret);
  }
  req->busy = false;
  mutex_unlock(&req->lock);
  return ret;
}

/* 计算发送前需要的静默时间。
 * 返回：静默 tick 数。
 * 按 3.5 个字符时间估算，至少按 2 ms 换算。
 */
static unsigned long ssf_mems_modbus_silent_ticks(struct ssf_mems_data *data) {
  return msecs_to_jiffies(ssf_mems_serial_gap_ms(data));
}

/* 等待旧应答隔离期结束和总线静默。
 * 返回：0 可以发送，-ENODEV 停止，-ETIMEDOUT 超时，负数也可能是信号中断。
 * 调用者持总线锁，不能持请求锁。
 */
static int ssf_mems_modbus_prepare_request(struct ssf_mems_data *data,
                                           bool recovery_read) {
  struct ssf_mems_modbus_request_state *req = &data->modbus_req;
  unsigned long silent = ssf_mems_modbus_silent_ticks(data);
  unsigned long now = jiffies;
  unsigned long deadline = time_after(req->quarantine_until, now) == true ?
                               req->quarantine_until : now;

  deadline += msecs_to_jiffies(SSF_MEMS_MODBUS_IDLE_WAIT_MS);
  for (;;) {
    unsigned long quiet_until;
    unsigned long remaining = 0;
    long wait_ret;

    if (READ_ONCE(req->shutting_down) == true && recovery_read == false)
      return -ENODEV;
    now = jiffies;
    quiet_until = ssf_mems_modbus_receive_last_activity(data) + silent;
    if (time_before(now, req->quarantine_until) == true)
      remaining = req->quarantine_until - now;
    if (time_before(now, quiet_until) == true)
      remaining = max(remaining, quiet_until - now);
    if (remaining == 0) {
      /* 绝不能持 req.lock 同步取消接收工作项，回调会获取该锁。 */
      ssf_mems_modbus_receive_flush(data);
      if (time_after_eq(jiffies,
                        ssf_mems_modbus_receive_last_activity(data) + silent) == true)
        return READ_ONCE(req->shutting_down) == true && recovery_read == false ?
                   -ENODEV : 0;
    }
    now = jiffies;
    if (time_after_eq(now, deadline) == true)
      return -ETIMEDOUT;
    if (remaining == 0)
      remaining = silent;
    remaining = min(remaining, deadline - now);
    wait_ret = wait_event_interruptible_timeout(
        req->waitq, READ_ONCE(req->shutting_down) == true &&
                        recovery_read == false, remaining);
    if (wait_ret < 0)
      return -ERESTARTSYS;
  }
}

/* 发送一个请求并等待匹配应答。
 * 返回：0 成功，负数为状态、发送或应答错误。
 * 错误帧和不匹配帧丢弃，继续等待有效应答。
 */
static int ssf_mems_modbus_execute(struct serdev_device *serdev,
                                  const struct ssf_modbus_transfer *transfer,
                                  const u16 *tx_values, size_t tx_values_count,
                                  u16 *rx_values, size_t rx_values_count,
                                  unsigned int timeout_ms,
                                  bool bus_locked, bool recovery_read) {
  struct ssf_mems_data *data;
  struct ssf_mems_modbus_request_state *req;
  int ret;

  if (serdev == NULL)
    return -EINVAL;
  data = serdev_device_get_drvdata(serdev);
  if (data == NULL)
    return -ENODEV;
  if (timeout_ms == 0)
    timeout_ms = SSF_MEMS_MODBUS_DEFAULT_TIMEOUT_MS;
  if (bus_locked == false)
    mutex_lock(&data->protocol.bus_lock);

  req = &data->modbus_req;
  mutex_lock(&req->lock);
  if (req->shutting_down == true && recovery_read == false) {
    ret = -ENODEV;
    goto out_unlock;
  }
  /* 未确认退出私有流时，只有专用只读恢复接口可以使用总线。
   * 关闭优先返回 -ENODEV，不能让卸载期间的普通调用误以为只需稍后重试。 */
  if (data->protocol.mode == SSF_MEMS_LINK_OTA ||
      (data->protocol.mode != SSF_MEMS_LINK_MODBUS && recovery_read == false)) {
    ret = -EAGAIN;
    goto out_unlock;
  }
  if (req->busy == true) {
    ret = -EBUSY;
    goto out_unlock;
  }

  req->busy = true;
  /* 保留总线所有权，但先放开请求锁，让旧帧回调和 flush 可以完成。 */
  mutex_unlock(&req->lock);
  ret = ssf_mems_modbus_prepare_request(data, recovery_read);
  mutex_lock(&req->lock);
  if (ret != 0 || (req->shutting_down == true && recovery_read == false)) {
    if (ret == 0)
      ret = -ENODEV;
    req->busy = false;
    goto out_unlock;
  }

  req->transfer = *transfer;
  req->slave_id = data->slave_id;
  req->values = rx_values;
  req->values_count = rx_values_count;
  req->write_value = transfer->frame->tx_format == SSF_TX_WRITE_SINGLE ? tx_values[0] : 0;
  /* 先发布等待状态再发送，避免快速应答先到达而被当作无主帧丢弃。 */
  req->status = -ETIMEDOUT;
  req->pending = true;

  ret = ssf_mems_modbus_send_request(data, &req->transfer, req->slave_id,
                                     tx_values, tx_values_count);
  if (ret != 0) {
    ssf_mems_modbus_finish_request(req, ret);
    req->busy = false;
    mutex_unlock(&req->lock);
  } else {
    ret = ssf_mems_modbus_wait_request(req, timeout_ms);
  }
  /* bus_lock 保证这里及下一次 prepare 的 guard 不会被其他请求覆盖。 */
  if (ret != 0 && ret != -EREMOTEIO && ret != -ENOMEM && ret != -ENODEV)
    req->quarantine_until = jiffies + msecs_to_jiffies(
        max(SSF_MEMS_MODBUS_RECOVERY_GUARD_MS, timeout_ms));
  goto out_bus_unlock;

out_unlock:
  mutex_unlock(&req->lock);
out_bus_unlock:
  if (bus_locked == false)
    mutex_unlock(&data->protocol.bus_lock);
  return ret;
}

/* 读取寄存器块中的一段数据。
 * 返回：0 成功，负数为校验或通信错误。
 * 是否已有总线锁由 bus_locked 指定。
 */
static int ssf_mems_modbus_read_block_range_common(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    u16 display_reg, u16 reg_count, u16 *values, size_t values_count,
    unsigned int timeout_ms, bool bus_locked) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (serdev == NULL || values == NULL || values_count < reg_count)
    return -EINVAL;
  ret = ssf_mems_modbus_plan_request(
      ssf_mems_modbus_sensor(serdev), SSF_MEMS_MODBUS_FUNC_READ, display_reg,
                                     reg_count, block, &transfer);
  if (ret != 0)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, NULL, 0, values,
                                 values_count, timeout_ms, bus_locked, false);
}

/* 读取寄存器块中的一段数据。
 * 返回：0 成功，负数为校验或通信错误。
 */
int ssf_mems_modbus_read_block_range(struct serdev_device *serdev,
                                     const struct ssf_block_cmd_desc *block,
                                     u16 display_reg, u16 reg_count,
                                     u16 *values, size_t values_count,
                                     unsigned int timeout_ms) {
  return ssf_mems_modbus_read_block_range_common(
      serdev, block, display_reg, reg_count, values, values_count, timeout_ms,
      false);
}

/* 在已加锁的总线上读取块内数据。
 * 返回：0 成功，负数为校验或通信错误。
 * 调用者须持 bus_lock。
 */
int ssf_mems_modbus_read_block_range_locked(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    u16 display_reg, u16 reg_count, u16 *values, size_t values_count,
    unsigned int timeout_ms) {
  return ssf_mems_modbus_read_block_range_common(
      serdev, block, display_reg, reg_count, values, values_count, timeout_ms,
      true);
}

/* 读取一段普通寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 私有数据响应不能通过此接口读取。
 */
int ssf_mems_modbus_read(struct serdev_device *serdev, u16 display_reg,
                         u16 reg_count, u16 *values, size_t values_count,
                         unsigned int timeout_ms) {
  return ssf_mems_modbus_read_block_range(serdev, NULL, display_reg, reg_count,
                                          values, values_count, timeout_ms);
}

/* 读取一个普通寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 */
int ssf_mems_modbus_read_reg(struct serdev_device *serdev, u16 display_reg,
                             u16 *value, unsigned int timeout_ms) {
  return ssf_mems_modbus_read(serdev, display_reg, 1, value, 1, timeout_ms);
}

/* 在已加锁的总线上读取一个寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 调用者须持 bus_lock。
 */
int ssf_mems_modbus_read_reg_locked(struct serdev_device *serdev,
                                    u16 display_reg, u16 *value,
                                    unsigned int timeout_ms) {
  return ssf_mems_modbus_read_block_range_common(
      serdev, NULL, display_reg, 1, value, 1, timeout_ms, true);
}

/* 只读确认定长采集后的普通通信。
 * 返回：0 成功，负数为校验或通信错误。
 * 调用者须持 bus_lock，恢复和关闭期间也允许读取。
 */
int ssf_mems_modbus_read_reg_recovery_locked(struct serdev_device *serdev,
                                             u16 display_reg, u16 *value,
                                             unsigned int timeout_ms) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (serdev == NULL || value == NULL)
    return -EINVAL;
  ret = ssf_mems_modbus_plan_request(
      ssf_mems_modbus_sensor(serdev), SSF_MEMS_MODBUS_FUNC_READ, display_reg,
                                     1, NULL, &transfer);
  if (ret != 0)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, NULL, 0, value, 1,
                                 timeout_ms, true, true);
}

/* 用 0x06 写入一个寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 是否已有总线锁由 bus_locked 指定。
 */
static int ssf_mems_modbus_write_reg_common(struct serdev_device *serdev,
                                            u16 display_reg, u16 value,
                                            unsigned int timeout_ms,
                                            bool bus_locked) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (serdev == NULL)
    return -EINVAL;
  ret = ssf_mems_modbus_plan_request(
      ssf_mems_modbus_sensor(serdev), SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE,
                                     display_reg, 1, NULL, &transfer);
  if (ret != 0)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, &value, 1, NULL, 0,
                                 timeout_ms, bus_locked, false);
}

/* 用 0x06 写入一个寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 表项须允许单寄存器写入。
 */
int ssf_mems_modbus_write_reg(struct serdev_device *serdev, u16 display_reg,
                              u16 value, unsigned int timeout_ms) {
  return ssf_mems_modbus_write_reg_common(serdev, display_reg, value,
                                          timeout_ms, false);
}

/* 在已加锁的总线上写入一个寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 调用者须持 bus_lock。
 */
int ssf_mems_modbus_write_reg_locked(struct serdev_device *serdev,
                                     u16 display_reg, u16 value,
                                     unsigned int timeout_ms) {
  return ssf_mems_modbus_write_reg_common(serdev, display_reg, value,
                                          timeout_ms, true);
}

/* 按名称查表并读取一个寄存器。
 * 返回：0 成功，负数为名称、校验或通信错误。
 * 是否已有总线锁由 bus_locked 指定。
 */
static int ssf_mems_modbus_read_name(struct serdev_device *serdev,
                                    const char *name, u16 *value,
                                    unsigned int timeout_ms, bool bus_locked) {
  const struct ssf_reg_desc *reg;

  if (serdev == NULL || name == NULL || name[0] == '\0' || value == NULL)
    return -EINVAL;
  if (ssf_mems_modbus_sensor(serdev) == NULL)
    return -ENODEV;
  reg = ssf_mems_sensor_find_reg_by_name(
      ssf_mems_modbus_sensor(serdev), name);
  if (reg == NULL)
    return -ENOENT;
  if (bus_locked == true)
    return ssf_mems_modbus_read_reg_locked(serdev, reg->display_reg, value,
                                          timeout_ms);
  return ssf_mems_modbus_read_reg(serdev, reg->display_reg, value, timeout_ms);
}

/* 按名称读取一个寄存器。
 * 返回：0 成功，负数为名称、校验或通信错误。
 */
int ssf_mems_modbus_read_reg_by_name(struct serdev_device *serdev,
                                    const char *name, u16 *value,
                                    unsigned int timeout_ms) {
  return ssf_mems_modbus_read_name(serdev, name, value, timeout_ms, false);
}

/* 在已加锁的总线上按名称读取寄存器。
 * 返回：0 成功，负数为名称、校验或通信错误。
 * 调用者须持 bus_lock。
 */
int ssf_mems_modbus_read_reg_by_name_locked(struct serdev_device *serdev,
                                           const char *name, u16 *value,
                                           unsigned int timeout_ms) {
  return ssf_mems_modbus_read_name(serdev, name, value, timeout_ms, true);
}

/* 按名称查表并写入一个寄存器。
 * 返回：0 成功，负数为名称、校验或通信错误。
 * 是否已有总线锁由 bus_locked 指定。
 */
static int ssf_mems_modbus_write_name(struct serdev_device *serdev,
                                     const char *name, u16 value,
                                     unsigned int timeout_ms, bool bus_locked) {
  const struct ssf_reg_desc *reg;

  if (serdev == NULL || name == NULL || name[0] == '\0')
    return -EINVAL;
  if (ssf_mems_modbus_sensor(serdev) == NULL)
    return -ENODEV;
  reg = ssf_mems_sensor_find_reg_by_name(
      ssf_mems_modbus_sensor(serdev), name);
  if (reg == NULL)
    return -ENOENT;
  return ssf_mems_modbus_write_reg_common(serdev, reg->display_reg, value,
                                          timeout_ms, bus_locked);
}

/* 按名称用 0x06 写入一个寄存器。
 * 返回：0 成功，负数为名称、校验或通信错误。
 * 仅写寄存器，不代替波特率保存和切换流程。
 */
int ssf_mems_modbus_write_reg_by_name(struct serdev_device *serdev,
                                     const char *name, u16 value,
                                     unsigned int timeout_ms) {
  return ssf_mems_modbus_write_name(serdev, name, value, timeout_ms, false);
}

/* 在已加锁的总线上按名称写入寄存器。
 * 返回：0 成功，负数为名称、校验或通信错误。
 * 调用者须持 bus_lock，不额外发送保存命令。
 */
int ssf_mems_modbus_write_reg_by_name_locked(struct serdev_device *serdev,
                                            const char *name, u16 value,
                                            unsigned int timeout_ms) {
  return ssf_mems_modbus_write_name(serdev, name, value, timeout_ms, true);
}

/* 用 0x10 写入一段寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 是否已有总线锁由 bus_locked 指定。
 */
static int ssf_mems_modbus_write_regs_common(
    struct serdev_device *serdev, u16 display_reg, u16 reg_count,
    const u16 *values, size_t values_count, unsigned int timeout_ms,
    bool bus_locked) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (serdev == NULL || values == NULL || values_count < reg_count)
    return -EINVAL;
  ret = ssf_mems_modbus_plan_request(
      ssf_mems_modbus_sensor(serdev), SSF_MEMS_MODBUS_FUNC_WRITE_MULTI,
                                     display_reg, reg_count, NULL, &transfer);
  if (ret != 0)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, values, values_count,
                                 NULL, 0, timeout_ms, bus_locked, false);
}

/* 用 0x10 写入一段寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 */
int ssf_mems_modbus_write_regs(struct serdev_device *serdev, u16 display_reg,
                               u16 reg_count, const u16 *values,
                               size_t values_count, unsigned int timeout_ms) {
  return ssf_mems_modbus_write_regs_common(serdev, display_reg, reg_count,
                                          values, values_count, timeout_ms, false);
}

/* 在已加锁的总线上写入一段寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 调用者须持 bus_lock。
 */
int ssf_mems_modbus_write_regs_locked(
    struct serdev_device *serdev, u16 display_reg, u16 reg_count,
    const u16 *values, size_t values_count, unsigned int timeout_ms) {
  return ssf_mems_modbus_write_regs_common(serdev, display_reg, reg_count,
                                          values, values_count, timeout_ms, true);
}

/* 按块命令描述写入寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 * 是否已有总线锁由 bus_locked 指定。
 */
static int ssf_mems_modbus_write_block_common(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    const u16 *values, size_t values_count, unsigned int timeout_ms,
    bool bus_locked) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (serdev == NULL || block == NULL || values == NULL || values_count < block->reg_count)
    return -EINVAL;
  if (block->direction != SSF_CMD_WRITE)
    return -EOPNOTSUPP;
  ret = ssf_mems_modbus_plan_request(
      ssf_mems_modbus_sensor(serdev), block->function, block->start_display_reg,
                                     block->reg_count, block, &transfer);
  if (ret != 0)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, values, values_count,
                                 NULL, 0, timeout_ms, bus_locked, false);
}

/* 按块命令描述写入寄存器。
 * 返回：0 成功，负数为校验或通信错误。
 */
int ssf_mems_modbus_write_block(struct serdev_device *serdev,
                                const struct ssf_block_cmd_desc *block,
                                const u16 *values, size_t values_count,
                                unsigned int timeout_ms) {
  return ssf_mems_modbus_write_block_common(
      serdev, block, values, values_count, timeout_ms, false);
}

/* 在已加锁的总线上执行块写入。
 * 返回：0 成功，负数为校验或通信错误。
 * 调用者须持 bus_lock。
 */
int ssf_mems_modbus_write_block_locked(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    const u16 *values, size_t values_count, unsigned int timeout_ms) {
  return ssf_mems_modbus_write_block_common(
      serdev, block, values, values_count, timeout_ms, true);
}

/* 尝试把完整帧匹配给当前请求。
 * 返回：true 已认领，false 未认领。
 * 认领从机异常帧也返回 true，不表示请求成功。
 */
bool ssf_mems_modbus_claim_frame(struct ssf_mems_data *data,
                                 const u8 *buf, size_t len) {
  struct ssf_mems_modbus_request_state *req;
  int ret;

  if (data == NULL || buf == NULL)
    return false;
  req = &data->modbus_req;
  mutex_lock(&req->lock);
  if (req->pending == false) {
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
