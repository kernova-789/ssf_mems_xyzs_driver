// SPDX-License-Identifier: GPL-2.0-only
#include "transport/serial_transport.h"

#include "core.h"

#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/serdev.h>

/* 估算当前 UART 发送指定字节数的时间。
 * 返回：向上取整的毫秒数，未配置速率时按 9600 计算。
 */
unsigned int ssf_mems_serial_wire_ms(const struct ssf_mems_data *data,
                                     size_t bytes) {
  unsigned int baud = READ_ONCE(data->protocol.host_baudrate);
  unsigned int bits = READ_ONCE(data->protocol.parity) == SSF_MEMS_PARITY_NONE ?
                          10U : 11U;
  u64 bit_ms = (u64)bytes * bits * 1000U;

  if (baud == 0)
    baud = data->sensor->profile.baudrates[SSF_MEMS_BAUDRATE_DEFAULT];
  return (bit_ms + baud - 1U) / baud;
}

/* 计算 RTU 的最短帧间隔。
 * 返回：至少 2 ms 的 3.5 字符时间。
 */
unsigned int ssf_mems_serial_gap_ms(const struct ssf_mems_data *data) {
  unsigned int baud = READ_ONCE(data->protocol.host_baudrate);
  unsigned int bits = READ_ONCE(data->protocol.parity) == SSF_MEMS_PARITY_NONE ?
                          35U : 39U;

  if (baud == 0)
    baud = data->sensor->profile.baudrates[SSF_MEMS_BAUDRATE_DEFAULT];
  return max(2U, DIV_ROUND_UP(bits * 1000U, baud));
}

/* 完整发送一帧并等待 UART 发完。
 * 返回：0 成功，负数为串口错误，-EIO 表示只发送了部分字节。
 */
int ssf_mems_serial_write(struct ssf_mems_data *data, const u8 *frame,
                           size_t len, unsigned int timeout_ms) {
  ssize_t written = serdev_device_write(data->serdev, frame, len,
                                        msecs_to_jiffies(timeout_ms));

  if (written < 0)
    return written;
  /* 短写也先排空已提交的字节，再由调用者隔离旧请求。 */
  serdev_device_wait_until_sent(data->serdev, msecs_to_jiffies(timeout_ms));
  return written == len ? 0 : -EIO;
}
