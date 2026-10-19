/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once

#include <linux/types.h>

struct ssf_mems_data;

/* 估算当前 UART 发送指定字节数的时间。
 * 返回：向上取整的毫秒数，未配置速率时按 9600 计算。
 */
unsigned int ssf_mems_serial_wire_ms(const struct ssf_mems_data *data,
                                     size_t bytes);
/* 计算 RTU 的最短帧间隔。
 * 返回：至少 2 ms 的 3.5 字符时间。
 */
unsigned int ssf_mems_serial_gap_ms(const struct ssf_mems_data *data);
/* 完整发送一帧并等待 UART 发完。
 * 返回：0 成功，负数为串口错误，-EIO 表示只发送了部分字节。
 * 调用者独占总线；应答等待和重试由各协议负责。
 */
int ssf_mems_serial_write(struct ssf_mems_data *data, const u8 *frame,
                           size_t len, unsigned int timeout_ms);
