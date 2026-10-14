#include "modbus_request.h"
#include "core.h"
#include "modbus.h"
#include "modbus_table.h"
#include "modbus_receive.h"

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

  if (transfer == NULL || reg_count == 0 ||
      (u32)display_reg + reg_count > 0x10000U)
    return -EINVAL;

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

  first = ssf_mems_modbus_find_reg(display_reg);
  if (first == NULL)
    return -ENOENT;
  if ((u32)first->protocol_addr + reg_count > 0x10000U)
    return -EINVAL;

  for (i = 0; i < reg_count; i++) {
    const struct ssf_reg_desc *reg = ssf_mems_modbus_find_reg(display_reg + i);

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

/* 按实际串口格式估算线上发送时间，并预留调度/控制器余量。
 * 返回完整发送的超时毫秒数，含线上时间与调度余量，至少 100 ms。 */
static unsigned int
ssf_mems_modbus_write_timeout_ms(const struct ssf_mems_xyzs_data *data,
                                 size_t len) {
  unsigned int baudrate = READ_ONCE(data->protocol.host_baudrate);
  unsigned int bits_per_char =
      READ_ONCE(data->protocol.parity) == SSF_MEMS_PARITY_NONE ? 10U : 11U;
  unsigned int wire_ms;
  unsigned long long bit_ms;

  if (baudrate == 0)
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
  if (buf == NULL)
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

/* 关闭请求模块并唤醒等待者；无返回值，当前请求状态设为 -ENODEV，空指针直接退出。 */
void ssf_mems_modbus_request_remove(struct ssf_mems_xyzs_data *data) {
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

/* 完成响应并保存 status；无返回值，busy 保留到等待者取走结果，避免新请求覆盖旧结果。 */
static void ssf_mems_modbus_finish_request(struct ssf_mems_modbus_request_state *req,
                                          int status) {
  req->pending = false;
  req->values = NULL;
  req->values_count = 0;
  req->write_value = 0;
  req->status = status;
}

/* 返回 0 请求成功，-EREMOTEIO 从机异常，-EFAULT 输出缓冲区异常，
 * -ENODEV 被卸载中断，-ETIMEDOUT 超时，-ERESTARTSYS 信号打断；取走后释放占用。 */
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

/* 返回 3.5 字符时间对应的帧间静默 jiffies，至少按 2 ms 换算。 */
static unsigned long ssf_mems_modbus_silent_ticks(struct ssf_mems_xyzs_data *data) {
  unsigned int baudrate = READ_ONCE(data->protocol.host_baudrate);
  unsigned int bits = READ_ONCE(data->protocol.parity) == SSF_MEMS_PARITY_NONE ? 35U : 39U;

  if (baudrate == 0)
    baudrate = 9600;
  return msecs_to_jiffies(max(2U, DIV_ROUND_UP(bits * 1000U, baudrate)));
}

/* 调用者持 bus_lock、保留 busy，但不持 req.lock。旧应答在 pending=false
 * 期间只会被丢弃。等待可由卸载中断，连续噪声也不会无限阻塞。
 * 返回 0 静默和隔离条件满足，-ENODEV 关闭中，-ETIMEDOUT 总线持续忙，
 * -ERESTARTSYS 被信号打断；调用者持总线锁，但不能持请求锁。 */
static int ssf_mems_modbus_prepare_request(struct ssf_mems_xyzs_data *data,
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

/* 执行已校验请求；返回 0 成功，-EINVAL 空设备，-ENODEV 无设备/关闭中，
 * -EBUSY 请求占用，-EAGAIN 当前总线模式不允许；其他负值及含义见请求头文件。
 * 校验/CRC 不匹配的候选帧会丢弃，等待正确应答或最终超时。 */
static int ssf_mems_modbus_execute(struct serdev_device *serdev,
                                  const struct ssf_modbus_transfer *transfer,
                                  const u16 *tx_values, size_t tx_values_count,
                                  u16 *rx_values, size_t rx_values_count,
                                  unsigned int timeout_ms,
                                  bool bus_locked, bool recovery_read) {
  struct ssf_mems_xyzs_data *data;
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

/* 读取块内实际保留的连续段；返回 0 成功，-EINVAL 参数/范围错误，-ENOENT 地址缺失，-EACCES 不可读，-EOPNOTSUPP 格式不支持；其他负值同通用读取。 */
static int ssf_mems_modbus_read_block_range_common(
    struct serdev_device *serdev, const struct ssf_block_cmd_desc *block,
    u16 display_reg, u16 reg_count, u16 *values, size_t values_count,
    unsigned int timeout_ms, bool bus_locked) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (serdev == NULL || values == NULL || values_count < reg_count)
    return -EINVAL;
  ret = ssf_mems_modbus_plan_request(SSF_MEMS_MODBUS_FUNC_READ, display_reg,
                                     reg_count, block, &transfer);
  if (ret != 0)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, NULL, 0, values,
                                 values_count, timeout_ms, bus_locked, false);
}

/* 读取块命令中的实际连续段；返回 0 成功，-EINVAL 参数/范围错误，-ENOENT 地址缺失，-EACCES 不可读，-EOPNOTSUPP 格式不支持；其他负值同通用读取。 */
int ssf_mems_modbus_read_block_range(struct serdev_device *serdev,
                                     const struct ssf_block_cmd_desc *block,
                                     u16 display_reg, u16 reg_count,
                                     u16 *values, size_t values_count,
                                     unsigned int timeout_ms) {
  return ssf_mems_modbus_read_block_range_common(
      serdev, block, display_reg, reg_count, values, values_count, timeout_ms,
      false);
}

/* 调用者已持 bus_lock 的块读取；返回值及含义同 read_block_range()。 */
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

/* 调用者已持 bus_lock 的单寄存器读取；返回值及含义同 read_reg()。 */
int ssf_mems_modbus_read_reg_locked(struct serdev_device *serdev,
                                    u16 display_reg, u16 *value,
                                    unsigned int timeout_ms) {
  return ssf_mems_modbus_read_block_range_common(
      serdev, NULL, display_reg, 1, value, 1, timeout_ms, true);
}

/* 退出原始流时仍需确认普通应答，不能因 shutting_down 而省略确认。
 * 返回 0 成功；其余错误同普通读取，OTA 模式仍返回 -EAGAIN。 */
int ssf_mems_modbus_read_reg_recovery_locked(struct serdev_device *serdev,
                                             u16 display_reg, u16 *value,
                                             unsigned int timeout_ms) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (serdev == NULL || value == NULL)
    return -EINVAL;
  ret = ssf_mems_modbus_plan_request(SSF_MEMS_MODBUS_FUNC_READ, display_reg,
                                     1, NULL, &transfer);
  if (ret != 0)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, NULL, 0, value, 1,
                                 timeout_ms, true, true);
}

/* 返回 0 单寄存器写入成功，-EOPNOTSUPP 表项不支持 0x06；其余校验、发送、响应和等待错误同多寄存器写入。 */
static int ssf_mems_modbus_write_reg_common(struct serdev_device *serdev,
                                            u16 display_reg, u16 value,
                                            unsigned int timeout_ms,
                                            bool bus_locked) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (serdev == NULL)
    return -EINVAL;
  ret = ssf_mems_modbus_plan_request(SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE,
                                     display_reg, 1, NULL, &transfer);
  if (ret != 0)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, &value, 1, NULL, 0,
                                 timeout_ms, bus_locked, false);
}

/* 以 0x06 写入 write_single 为真的可写寄存器；返回 0 成功，-EOPNOTSUPP 表项不支持 0x06；其余校验、发送、响应和等待错误同 ssf_mems_modbus_write_regs()。 */
int ssf_mems_modbus_write_reg(struct serdev_device *serdev, u16 display_reg,
                              u16 value, unsigned int timeout_ms) {
  return ssf_mems_modbus_write_reg_common(serdev, display_reg, value,
                                          timeout_ms, false);
}

/* 调用者已持 bus_lock 的单寄存器写入；返回值及含义同 write_reg()。 */
int ssf_mems_modbus_write_reg_locked(struct serdev_device *serdev,
                                     u16 display_reg, u16 value,
                                     unsigned int timeout_ms) {
  return ssf_mems_modbus_write_reg_common(serdev, display_reg, value,
                                          timeout_ms, true);
}

/* 名称只用于定位表项；收发、权限和总线锁沿用已有单寄存器读取。
 * 返回 0 成功，-EINVAL 空参数/空名称，-ENOENT 名称未入表；
 * 其余负值及含义同 ssf_mems_modbus_read_reg()。 */
static int ssf_mems_modbus_read_name(struct serdev_device *serdev,
                                    const char *name, u16 *value,
                                    unsigned int timeout_ms, bool bus_locked) {
  const struct ssf_reg_desc *reg;

  if (serdev == NULL || name == NULL || name[0] == '\0' || value == NULL)
    return -EINVAL;
  reg = ssf_mems_modbus_find_reg_by_name(name);
  if (reg == NULL)
    return -ENOENT;
  if (bus_locked == true)
    return ssf_mems_modbus_read_reg_locked(serdev, reg->display_reg, value,
                                          timeout_ms);
  return ssf_mems_modbus_read_reg(serdev, reg->display_reg, value, timeout_ms);
}

/* 普通按名称读取，内部获取总线锁。
 * 返回值同 read_reg；不做单位换算，不触发高层配置流程。 */
int ssf_mems_modbus_read_reg_by_name(struct serdev_device *serdev,
                                    const char *name, u16 *value,
                                    unsigned int timeout_ms) {
  return ssf_mems_modbus_read_name(serdev, name, value, timeout_ms, false);
}

/* 已持总线锁的按名称读取，不重复加锁。
 * 返回值及含义同 read_reg_by_name()。 */
int ssf_mems_modbus_read_reg_by_name_locked(struct serdev_device *serdev,
                                           const char *name, u16 *value,
                                           unsigned int timeout_ms) {
  return ssf_mems_modbus_read_name(serdev, name, value, timeout_ms, true);
}

/* 查表后复用原有写路径，不额外发送保存或重启命令。
 * 返回 0 成功，-EINVAL 空参数/空名称，-ENOENT 名称未入表；
 * 其余负值及含义同 ssf_mems_modbus_write_reg()。 */
static int ssf_mems_modbus_write_name(struct serdev_device *serdev,
                                     const char *name, u16 value,
                                     unsigned int timeout_ms, bool bus_locked) {
  const struct ssf_reg_desc *reg;

  if (serdev == NULL || name == NULL || name[0] == '\0')
    return -EINVAL;
  reg = ssf_mems_modbus_find_reg_by_name(name);
  if (reg == NULL)
    return -ENOENT;
  return ssf_mems_modbus_write_reg_common(serdev, reg->display_reg, value,
                                          timeout_ms, bus_locked);
}

/* 普通按名称写入，内部获取总线锁。
 * 返回 0 成功，-EINVAL 名称/设备为空，-ENOENT 名称缺失；其余错误同 write_reg()。 */
int ssf_mems_modbus_write_reg_by_name(struct serdev_device *serdev,
                                     const char *name, u16 value,
                                     unsigned int timeout_ms) {
  return ssf_mems_modbus_write_name(serdev, name, value, timeout_ms, false);
}

/* 已持总线锁的按名称写入，不重复加锁。
 * 返回值及含义同 write_reg_by_name()。 */
int ssf_mems_modbus_write_reg_by_name_locked(struct serdev_device *serdev,
                                            const char *name, u16 value,
                                            unsigned int timeout_ms) {
  return ssf_mems_modbus_write_name(serdev, name, value, timeout_ms, true);
}

/* 返回 0 连续写入成功，-EINVAL 参数/范围错误，-ENOENT 地址缺失，-EACCES 不可写，-EOPNOTSUPP 帧格式不支持，-ENODEV 无设备/关闭中，-EBUSY 请求占用，-ENOMEM 分配失败，-EIO 发送不足，-EREMOTEIO 从机异常，-ETIMEDOUT 超时，-ERESTARTSYS 信号打断；其他负值来自组帧或串口。 */
static int ssf_mems_modbus_write_regs_common(
    struct serdev_device *serdev, u16 display_reg, u16 reg_count,
    const u16 *values, size_t values_count, unsigned int timeout_ms,
    bool bus_locked) {
  struct ssf_modbus_transfer transfer;
  int ret;

  if (serdev == NULL || values == NULL || values_count < reg_count)
    return -EINVAL;
  ret = ssf_mems_modbus_plan_request(SSF_MEMS_MODBUS_FUNC_WRITE_MULTI,
                                     display_reg, reg_count, NULL, &transfer);
  if (ret != 0)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, values, values_count,
                                 NULL, 0, timeout_ms, bus_locked, false);
}

/* 普通连续写，由执行器取得总线锁；返回 0 成功，负值及含义同 write_regs_common()。 */
int ssf_mems_modbus_write_regs(struct serdev_device *serdev, u16 display_reg,
                               u16 reg_count, const u16 *values,
                               size_t values_count, unsigned int timeout_ms) {
  return ssf_mems_modbus_write_regs_common(serdev, display_reg, reg_count,
                                          values, values_count, timeout_ms, false);
}

/* 已持总线锁的连续写：用于原始批采集前配置长度，不重复加锁。
 * 返回值及含义同 write_regs()，仅配置指定范围。 */
int ssf_mems_modbus_write_regs_locked(
    struct serdev_device *serdev, u16 display_reg, u16 reg_count,
    const u16 *values, size_t values_count, unsigned int timeout_ms) {
  return ssf_mems_modbus_write_regs_common(serdev, display_reg, reg_count,
                                          values, values_count, timeout_ms, true);
}

/* 按表中块描述执行写入；返回 0 成功，-EINVAL 参数/范围错误，-EOPNOTSUPP 不是可写块或格式不支持；其他负值同连续写入。 */
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
  ret = ssf_mems_modbus_plan_request(block->function, block->start_display_reg,
                                     block->reg_count, block, &transfer);
  if (ret != 0)
    return ret;
  return ssf_mems_modbus_execute(serdev, &transfer, values, values_count,
                                 NULL, 0, timeout_ms, bus_locked, false);
}

/* 按块描述执行写入，保留功能码/响应格式/方向和固定范围校验；返回 0 成功，-EINVAL 参数错误，-EOPNOTSUPP 不是可写块或格式不支持；其余负值同 ssf_mems_modbus_write_regs()。 */
int ssf_mems_modbus_write_block(struct serdev_device *serdev,
                                const struct ssf_block_cmd_desc *block,
                                const u16 *values, size_t values_count,
                                unsigned int timeout_ms) {
  return ssf_mems_modbus_write_block_common(
      serdev, block, values, values_count, timeout_ms, false);
}

/* 已持 bus_lock 的块写入；返回值及含义同 write_block()。 */
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
