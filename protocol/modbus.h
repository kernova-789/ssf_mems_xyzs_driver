#pragma once

#include <linux/types.h>

struct serdev_device;
struct ssf_mems_xyzs_data;
struct ssf_modbus_frame_desc;
struct ssf_block_cmd_desc;
struct ssf_modbus_transfer;
#define SSF_MEMS_FEATURE_SCALE 100U

/*
 * 所有特征值均以定点整数形式保存。以 _x100 结尾的字段所存储的数值，
 * 是传感器手册中所述物理量的 100 倍。保留原始定点表示既可避免在
 * 内核中进行浮点运算，也能完整保留寄存器中的精确数值。
 */
struct ssf_mems_axis_features {
  u16 high_freq_acc_rms_x100; /* mm/s，沿用寄存器表中的表述 */
  u16 low_freq_velocity_rms_x100; /* mm/s */
  u16 acc_peak_to_peak_x100; /* g */
  u16 acc_peak_x100;         /* g */
  u16 acc_rms_x100;          /* g */
  u16 kurtosis_x100;
  u16 velocity_rms_x100; /* mm/s */
};

struct ssf_mems_sound_features {
  u16 rms_x100;          /* Pa */
  u16 peak_x100;         /* Pa */
  u16 peak_to_peak_x100; /* Pa */
};

struct ssf_mems_sensor_data {
  struct ssf_mems_axis_features x;
  struct ssf_mems_axis_features y;
  struct ssf_mems_axis_features z;

  u16 temperature_x100; /* 摄氏度 */
  struct ssf_mems_sound_features sound;
  u16 zero_crossing_rate_x100; /* 百分比 */
  u16 spectral_centroid_x100;  /* Hz */
  u16 spectral_flux_x100;      /* Pa^2 */

  /* 位 0、位 1 和位 2 分别表示 X、Y 和 Z 轴的启动状态。 */
  u8 startup_flags;
};

/* 返回输入字节序列的 Modbus CRC16。 */
u16 ssf_mems_modbus_crc16(const u8 *buf, size_t len);

/* 返回请求帧长度；-EINVAL 数量/描述错误，-EOPNOTSUPP 无发送格式，-EMSGSIZE 帧超限。 */
int ssf_mems_modbus_tx_frame_len(const struct ssf_modbus_frame_desc *frame, u16 reg_count);

/* 返回响应帧长度；-EINVAL 无描述，-EMSGSIZE 普通读取字节数为零、非整寄存器或超限。 */
int ssf_mems_modbus_rx_frame_len(const struct ssf_modbus_frame_desc *frame, u8 byte_count);

/* 返回 0 请求描述已生成，-EINVAL 参数/范围错误，-ENOENT 地址未入表，-EACCES 权限不足，-EOPNOTSUPP 格式不支持。 */
int ssf_mems_modbus_plan_request(u8 function, u16 display_reg, u16 reg_count,
                                const struct ssf_block_cmd_desc *block,
                                struct ssf_modbus_transfer *transfer);

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

/* 按块表解码完整连续特征块；返回 0 成功，-ENOENT 块命令缺失，-EINVAL 参数/数量错误；删除表项对应字段置零。 */
int ssf_mems_modbus_decode_features(const u16 *registers, size_t count,
                                    struct ssf_mems_sensor_data *result);

/* 按保留表项分段读取特征；返回 0 成功（无特征时缓存全零），-EINVAL 参数/范围错误，-ENODEV 无驱动数据，-ENOENT 块命令缺失，-EOPNOTSUPP 块格式不支持；其他负值来自读请求或解码。 */
int ssf_mems_modbus_read_features(struct serdev_device *serdev,
                                  unsigned int timeout_ms);

/* 复制特征值缓存；返回 0 成功，-EINVAL 设备/输出指针为空，-ENODEV 未绑定驱动数据，-ENODATA 缓存尚无有效数据。 */
int ssf_mems_modbus_get_features(struct serdev_device *serdev,
                                 struct ssf_mems_sensor_data *result);
