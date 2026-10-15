#pragma once

#include "modbus/modbus_types.h"

/* 本模块只提供无设备状态的帧编解码；请求归属信息由调用者传入。 */
/* 计算一段数据的 Modbus CRC16。
 * 返回：CRC16 校验值。
 */
u16 ssf_mems_modbus_crc16(const u8 *buf, size_t len);

/* 计算请求帧长度。
 * 返回：帧字节数，负数为参数、格式或长度错误。
 */
int ssf_mems_modbus_tx_frame_len(const struct ssf_modbus_frame_desc *frame, u16 reg_count);

/* 计算响应帧长度。
 * 返回：帧字节数，负数为描述或长度错误。
 */
int ssf_mems_modbus_rx_frame_len(const struct ssf_modbus_frame_desc *frame, u8 byte_count);

/* 组装一个 Modbus 请求帧。
 * 返回：帧字节数，负数为参数、格式或空间错误。
 */
int ssf_mems_modbus_build_request(const struct ssf_modbus_transfer *transfer,
                                 u8 slave_id, const u16 *values, size_t values_count,
                                 u8 *buf, size_t capacity);

/* 检查 Modbus 帧的 CRC。
 * 返回：0 正确，-EINVAL 空帧，-EMSGSIZE 帧过短，-EBADMSG 校验失败。
 */
int ssf_mems_modbus_check_crc(const u8 *buf, size_t len);

/* 解析当前请求对应的 Modbus 应答。
 * 返回：0 成功，-ENOMSG 不匹配，-EREMOTEIO 从机异常，其他负数为帧或参数错误。
 */
int ssf_mems_modbus_parse_response(const struct ssf_modbus_transfer *transfer,
                                   u8 slave_id, u16 write_value, const u8 *buf,
                                   size_t len, u16 *values, size_t values_count);
