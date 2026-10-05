#pragma once

#include "modbus_types.h"

/* 本模块只提供无设备状态的帧编解码；请求归属信息由调用者传入。 */
/* 返回输入字节序列的 Modbus CRC16。 */
u16 ssf_mems_modbus_crc16(const u8 *buf, size_t len);

/* 返回请求帧长度；-EINVAL 数量/描述错误，-EOPNOTSUPP 无发送格式，-EMSGSIZE 帧超限。 */
int ssf_mems_modbus_tx_frame_len(const struct ssf_modbus_frame_desc *frame, u16 reg_count);

/* 返回响应帧长度；-EINVAL 无描述，-EMSGSIZE 普通读取字节数为零、非整寄存器或超限。 */
int ssf_mems_modbus_rx_frame_len(const struct ssf_modbus_frame_desc *frame, u8 byte_count);

/* 返回组装帧长度；-EINVAL 参数/布局错误，-ENOSPC 容量不足，其他负值来自帧长度检查。 */
int ssf_mems_modbus_build_request(const struct ssf_modbus_transfer *transfer,
                                 u8 slave_id, const u16 *values, size_t values_count,
                                 u8 *buf, size_t capacity);

/* 返回 0 CRC 正确，-EINVAL 空帧，-EMSGSIZE 帧过短，-EBADMSG CRC 不匹配。 */
int ssf_mems_modbus_check_crc(const u8 *buf, size_t len);

/* 返回 0 响应成功，-ENOMSG 不匹配，-EREMOTEIO 从机异常，-EFAULT 结果缓冲区异常，-EINVAL 参数错误，-EMSGSIZE 帧长错误，-EBADMSG CRC 错误。 */
int ssf_mems_modbus_parse_response(const struct ssf_modbus_transfer *transfer,
                                   u8 slave_id, u16 write_value, const u8 *buf,
                                   size_t len, u16 *values, size_t values_count);
