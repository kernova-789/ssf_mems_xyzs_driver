#pragma once

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/types.h>

#include "modbus.h"

/* 所有 RTU 帧共用的线格式；发送、接收和解析均引用这里。 */
#define SSF_MEMS_MODBUS_CRC_LEN 2U
#define SSF_MEMS_MODBUS_CRC_INIT 0xffffU
#define SSF_MEMS_MODBUS_CRC_POLY 0xa001U
#define SSF_MEMS_MODBUS_EXCEPTION_FLAG 0x80U
#define SSF_MEMS_MODBUS_DEFAULT_SLAVE_ID 0x01U
#define SSF_MEMS_MODBUS_READ_MAX_REGS 125U
#define SSF_MEMS_MODBUS_WRITE_MAX_REGS 123U
#define SSF_MEMS_MODBUS_MAX_FRAME_LEN 256U

enum ssf_mems_modbus_func {
  SSF_MEMS_MODBUS_FUNC_READ = 0x03,
  SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE = 0x06,
  SSF_MEMS_MODBUS_FUNC_WRITE_MULTI = 0x10,
};

enum ssf_cmd_dir {
  SSF_CMD_READ,
  SSF_CMD_WRITE,
};

enum ssf_reg_access {
  SSF_REG_READ = BIT(0),
  SSF_REG_WRITE = BIT(1),
};

enum ssf_reg_type {
  SSF_REG_U16,
  SSF_REG_U32_HIGH,
  SSF_REG_U32_LOW,
};

enum ssf_tx_format {
  SSF_TX_READ_REGS,
  SSF_TX_WRITE_SINGLE,
  SSF_TX_WRITE_MULTI,
  SSF_TX_NONE,
};

enum ssf_rx_format {
  SSF_RX_HOLDING_REGS,
  SSF_RX_WRITE_SINGLE_ECHO,
  SSF_RX_WRITE_MULTI_ACK,
  SSF_RX_EXCEPTION,
  SSF_RX_RAW_AXIS, /* 私有原始轴数据，通用读取接口不支持。 */
  SSF_RX_RAW_XYZ, /* 私有 0x15 数据，接收策略仍待实现。 */
};

enum ssf_decode_kind {
  SSF_DECODE_REGISTERS,
  SSF_DECODE_FEATURES,
};

/* 字段位置和长度规则由帧表提供，帧算法统一放在 modbus.c。 */
struct ssf_modbus_frame_desc {
  u8 function;
  enum ssf_tx_format tx_format;
  enum ssf_rx_format rx_format;
  u16 max_reg_count;
  u8 tx_base_len;
  u8 tx_bytes_per_reg;
  u8 tx_data_offset;
  u8 address_offset;
  u8 quantity_offset;
  u8 tx_byte_count_offset;
  u8 rx_base_len;
  u8 rx_bytes_per_reg;
  u8 rx_byte_count_offset;
  u8 rx_data_offset;
};

/* 一个表项对应一个寄存器；删除特征表项后自动跳过该地址，绝不重编号。 */
struct ssf_reg_desc {
  u16 display_reg;
  u16 protocol_addr;
  unsigned int access;
  enum ssf_reg_type type;
  enum ssf_rx_format read_format;
  bool write_single;
  size_t feature_offset;
  u8 feature_width; /* 0 表示该寄存器不映射到特征缓存。 */
  u16 value_mask;
  const char *name;
};

enum ssf_block_cmd_id {
  SSF_BLOCK_READ_ALL_FEATURES,
  SSF_BLOCK_WRITE_WORK_PARAMETERS,
};

/* sparse_read 为真时，起点和数量描述地址范围，实际请求按保留表项分段。 */
struct ssf_block_cmd_desc {
  enum ssf_block_cmd_id id;
  u8 function;
  enum ssf_cmd_dir direction;
  enum ssf_rx_format rx_format;
  enum ssf_decode_kind decode_kind;
  u16 start_display_reg;
  u16 reg_count;
  bool sparse_read;
  const char *name;
};

/* 发送时保存实际请求描述，响应本身不携带读取起始地址。 */
struct ssf_modbus_transfer {
  const struct ssf_modbus_frame_desc *frame;
  const struct ssf_block_cmd_desc *block;
  u16 display_reg;
  u16 protocol_addr;
  u16 reg_count;
};

static const struct ssf_modbus_frame_desc ssf_frame_table[] = {
    {
        .function = SSF_MEMS_MODBUS_FUNC_READ,
        .tx_format = SSF_TX_READ_REGS,
        .rx_format = SSF_RX_HOLDING_REGS,
        .max_reg_count = SSF_MEMS_MODBUS_READ_MAX_REGS,
        .tx_base_len = 8,
        .address_offset = 2,
        .quantity_offset = 4,
        .rx_base_len = 5,
        .rx_bytes_per_reg = 2,
        .rx_byte_count_offset = 2,
        .rx_data_offset = 3,
    },
    {
        .function = SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE,
        .tx_format = SSF_TX_WRITE_SINGLE,
        .rx_format = SSF_RX_WRITE_SINGLE_ECHO,
        .max_reg_count = 1,
        .tx_base_len = 8,
        .address_offset = 2,
        .quantity_offset = 4,
        .rx_base_len = 8,
    },
    {
        .function = SSF_MEMS_MODBUS_FUNC_WRITE_MULTI,
        .tx_format = SSF_TX_WRITE_MULTI,
        .rx_format = SSF_RX_WRITE_MULTI_ACK,
        .max_reg_count = SSF_MEMS_MODBUS_WRITE_MAX_REGS,
        .tx_base_len = 9,
        .tx_bytes_per_reg = 2,
        .tx_data_offset = 7,
        .address_offset = 2,
        .quantity_offset = 4,
        .tx_byte_count_offset = 6,
        .rx_base_len = 8,
    },
    {
        .tx_format = SSF_TX_NONE,
        .rx_format = SSF_RX_EXCEPTION,
        .rx_base_len = 5,
        .rx_data_offset = 2,
    },
};

static const struct ssf_reg_desc ssf_reg_table[] = {
    /* 40001～40029：只读特征值。 */
    {
        .display_reg = 40001,
        .protocol_addr = 0x0000,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, x.high_freq_acc_rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->x.high_freq_acc_rms_x100),
        .value_mask = 0xffff,
        .name = "x_high_freq_acc_rms",
    },
    {
        .display_reg = 40002,
        .protocol_addr = 0x0001,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, x.low_freq_velocity_rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->x.low_freq_velocity_rms_x100),
        .value_mask = 0xffff,
        .name = "x_low_freq_velocity_rms",
    },
    {
        .display_reg = 40003,
        .protocol_addr = 0x0002,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, y.high_freq_acc_rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->y.high_freq_acc_rms_x100),
        .value_mask = 0xffff,
        .name = "y_high_freq_acc_rms",
    },
    {
        .display_reg = 40004,
        .protocol_addr = 0x0003,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, y.low_freq_velocity_rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->y.low_freq_velocity_rms_x100),
        .value_mask = 0xffff,
        .name = "y_low_freq_velocity_rms",
    },
    {
        .display_reg = 40005,
        .protocol_addr = 0x0004,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, z.high_freq_acc_rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->z.high_freq_acc_rms_x100),
        .value_mask = 0xffff,
        .name = "z_high_freq_acc_rms",
    },
    {
        .display_reg = 40006,
        .protocol_addr = 0x0005,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, z.low_freq_velocity_rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->z.low_freq_velocity_rms_x100),
        .value_mask = 0xffff,
        .name = "z_low_freq_velocity_rms",
    },
    {
        .display_reg = 40007,
        .protocol_addr = 0x0006,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, temperature_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->temperature_x100),
        .value_mask = 0xffff,
        .name = "temperature",
    },
    {
        .display_reg = 40008,
        .protocol_addr = 0x0007,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, x.acc_peak_to_peak_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->x.acc_peak_to_peak_x100),
        .value_mask = 0xffff,
        .name = "x_acc_peak_to_peak",
    },
    {
        .display_reg = 40009,
        .protocol_addr = 0x0008,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, y.acc_peak_to_peak_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->y.acc_peak_to_peak_x100),
        .value_mask = 0xffff,
        .name = "y_acc_peak_to_peak",
    },
    {
        .display_reg = 40010,
        .protocol_addr = 0x0009,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, z.acc_peak_to_peak_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->z.acc_peak_to_peak_x100),
        .value_mask = 0xffff,
        .name = "z_acc_peak_to_peak",
    },
    {
        .display_reg = 40011,
        .protocol_addr = 0x000a,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, x.acc_peak_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->x.acc_peak_x100),
        .value_mask = 0xffff,
        .name = "x_acc_peak",
    },
    {
        .display_reg = 40012,
        .protocol_addr = 0x000b,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, y.acc_peak_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->y.acc_peak_x100),
        .value_mask = 0xffff,
        .name = "y_acc_peak",
    },
    {
        .display_reg = 40013,
        .protocol_addr = 0x000c,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, z.acc_peak_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->z.acc_peak_x100),
        .value_mask = 0xffff,
        .name = "z_acc_peak",
    },
    {
        .display_reg = 40014,
        .protocol_addr = 0x000d,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, x.acc_rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->x.acc_rms_x100),
        .value_mask = 0xffff,
        .name = "x_acc_rms",
    },
    {
        .display_reg = 40015,
        .protocol_addr = 0x000e,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, y.acc_rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->y.acc_rms_x100),
        .value_mask = 0xffff,
        .name = "y_acc_rms",
    },
    {
        .display_reg = 40016,
        .protocol_addr = 0x000f,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, z.acc_rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->z.acc_rms_x100),
        .value_mask = 0xffff,
        .name = "z_acc_rms",
    },
    {
        .display_reg = 40017,
        .protocol_addr = 0x0010,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, x.kurtosis_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->x.kurtosis_x100),
        .value_mask = 0xffff,
        .name = "x_kurtosis",
    },
    {
        .display_reg = 40018,
        .protocol_addr = 0x0011,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, y.kurtosis_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->y.kurtosis_x100),
        .value_mask = 0xffff,
        .name = "y_kurtosis",
    },
    {
        .display_reg = 40019,
        .protocol_addr = 0x0012,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, z.kurtosis_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->z.kurtosis_x100),
        .value_mask = 0xffff,
        .name = "z_kurtosis",
    },
    {
        .display_reg = 40020,
        .protocol_addr = 0x0013,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, x.velocity_rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->x.velocity_rms_x100),
        .value_mask = 0xffff,
        .name = "x_velocity_rms",
    },
    {
        .display_reg = 40021,
        .protocol_addr = 0x0014,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, y.velocity_rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->y.velocity_rms_x100),
        .value_mask = 0xffff,
        .name = "y_velocity_rms",
    },
    {
        .display_reg = 40022,
        .protocol_addr = 0x0015,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, z.velocity_rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->z.velocity_rms_x100),
        .value_mask = 0xffff,
        .name = "z_velocity_rms",
    },
    {
        .display_reg = 40023,
        .protocol_addr = 0x0016,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, sound.rms_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->sound.rms_x100),
        .value_mask = 0xffff,
        .name = "sound_rms",
    },
    {
        .display_reg = 40024,
        .protocol_addr = 0x0017,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, sound.peak_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->sound.peak_x100),
        .value_mask = 0xffff,
        .name = "sound_peak",
    },
    {
        .display_reg = 40025,
        .protocol_addr = 0x0018,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, sound.peak_to_peak_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->sound.peak_to_peak_x100),
        .value_mask = 0xffff,
        .name = "sound_peak_to_peak",
    },
    {
        .display_reg = 40026,
        .protocol_addr = 0x0019,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, zero_crossing_rate_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->zero_crossing_rate_x100),
        .value_mask = 0xffff,
        .name = "zero_crossing_rate",
    },
    {
        .display_reg = 40027,
        .protocol_addr = 0x001a,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, spectral_centroid_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->spectral_centroid_x100),
        .value_mask = 0xffff,
        .name = "spectral_centroid",
    },
    {
        .display_reg = 40028,
        .protocol_addr = 0x001b,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, spectral_flux_x100),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->spectral_flux_x100),
        .value_mask = 0xffff,
        .name = "spectral_flux",
    },
    {
        .display_reg = 40029,
        .protocol_addr = 0x001c,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_offset = offsetof(struct ssf_mems_sensor_data, startup_flags),
        .feature_width = sizeof(((struct ssf_mems_sensor_data *)0)->startup_flags),
        .value_mask = 0x0007,
        .name = "startup_flag",
    },

    /* 40050～40053：配置寄存器；保留原有 0x06 支持范围。 */
    {
        .display_reg = 40050,
        .protocol_addr = 0x0031,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .write_single = true,
        .name = "auto_report_enable",
    },
    {
        .display_reg = 40051,
        .protocol_addr = 0x0032,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .write_single = true,
        .name = "auto_report_time",
    },
    {
        .display_reg = 40052,
        .protocol_addr = 0x0033,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .write_single = true,
        .name = "sampling_rate",
    },
    {
        .display_reg = 40053,
        .protocol_addr = 0x0034,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .write_single = true,
        .name = "sampling_length",
    },

    /* 这些地址触发特殊数据响应，不能按普通 0x03 寄存器响应读取。 */
    {
        .display_reg = 40054,
        .protocol_addr = 0x0035,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_RAW_AXIS,
        .name = "x_raw_data",
    },
    {
        .display_reg = 40055,
        .protocol_addr = 0x0036,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_RAW_AXIS,
        .name = "y_raw_data",
    },
    {
        .display_reg = 40056,
        .protocol_addr = 0x0037,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_RAW_AXIS,
        .name = "z_raw_data",
    },
    {
        .display_reg = 40058,
        .protocol_addr = 0x0039,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_RAW_AXIS,
        .name = "start_xyz_continuous",
    },
    {
        .display_reg = 40059,
        .protocol_addr = 0x003a,
        .access = SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .write_single = true,
        .name = "stop_xyz_continuous",
    },
    {
        .display_reg = 40060,
        .protocol_addr = 0x003b,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_RAW_XYZ,
        .name = "start_custom_length",
    },

    /* 40061～40071：手册标记为 RW，40061/40062 合成一个 U32 字段。 */
    {
        .display_reg = 40061,
        .protocol_addr = 0x003c,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U32_HIGH,
        .read_format = SSF_RX_HOLDING_REGS,
        .name = "custom_length_high",
    },
    {
        .display_reg = 40062,
        .protocol_addr = 0x003d,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U32_LOW,
        .read_format = SSF_RX_HOLDING_REGS,
        .name = "custom_length_low",
    },
    {
        .display_reg = 40063,
        .protocol_addr = 0x003e,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .name = "raw_sound_4096_enable",
    },
    {
        .display_reg = 40064,
        .protocol_addr = 0x003f,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .name = "x_acc_threshold",
    },
    {
        .display_reg = 40065,
        .protocol_addr = 0x0040,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .name = "y_acc_threshold",
    },
    {
        .display_reg = 40066,
        .protocol_addr = 0x0041,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .name = "z_acc_threshold",
    },
    {
        .display_reg = 40067,
        .protocol_addr = 0x0042,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .name = "x_velocity_threshold",
    },
    {
        .display_reg = 40068,
        .protocol_addr = 0x0043,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .name = "y_velocity_threshold",
    },
    {
        .display_reg = 40069,
        .protocol_addr = 0x0044,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .name = "z_velocity_threshold",
    },
    {
        .display_reg = 40070,
        .protocol_addr = 0x0045,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .name = "parameter_switch",
    },
    {
        .display_reg = 40071,
        .protocol_addr = 0x0046,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .name = "continuous_raw_sound",
    },

    /* 通信参数与固件版本。 */
    {
        .display_reg = 40101,
        .protocol_addr = 0x0064,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .write_single = true,
        .name = "slave_addr",
    },
    {
        .display_reg = 40102,
        .protocol_addr = 0x0065,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .write_single = true,
        .name = "baudrate",
    },
    {
        .display_reg = 40103,
        .protocol_addr = 0x0066,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .write_single = true,
        .name = "parity",
    },
    {
        .display_reg = 40121,
        .protocol_addr = 0x0078,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .name = "firmware_version",
    },
};

static const struct ssf_block_cmd_desc ssf_block_cmd_table[] = {
    {
        .id = SSF_BLOCK_READ_ALL_FEATURES,
        .function = SSF_MEMS_MODBUS_FUNC_READ,
        .direction = SSF_CMD_READ,
        .rx_format = SSF_RX_HOLDING_REGS,
        .decode_kind = SSF_DECODE_FEATURES,
        .start_display_reg = 40001,
        .reg_count = 29, /* 原始地址跨度；删除特征表项后无需调整。 */
        .sparse_read = true,
        .name = "read_all_features",
    },
    {
        .id = SSF_BLOCK_WRITE_WORK_PARAMETERS,
        .function = SSF_MEMS_MODBUS_FUNC_WRITE_MULTI,
        .direction = SSF_CMD_WRITE,
        .rx_format = SSF_RX_WRITE_MULTI_ACK,
        .decode_kind = SSF_DECODE_REGISTERS,
        .start_display_reg = 40061,
        .reg_count = 10,
        .name = "write_work_parameters",
    },
};

#define SSF_FRAME_TABLE_SIZE ARRAY_SIZE(ssf_frame_table)
#define SSF_REG_TABLE_SIZE ARRAY_SIZE(ssf_reg_table)
#define SSF_BLOCK_CMD_TABLE_SIZE ARRAY_SIZE(ssf_block_cmd_table)

/* 返回普通请求的帧描述；功能码未入表返回 NULL。 */
static inline const struct ssf_modbus_frame_desc *
ssf_mems_modbus_find_frame(u8 function) {
  size_t i;

  for (i = 0; i < SSF_FRAME_TABLE_SIZE; i++) {
    if (ssf_frame_table[i].tx_format != SSF_TX_NONE &&
        ssf_frame_table[i].function == function)
      return &ssf_frame_table[i];
  }
  return NULL;
}

/* 返回正常/异常响应帧描述；对应请求功能码不支持时返回 NULL。 */
static inline const struct ssf_modbus_frame_desc *
ssf_mems_modbus_find_rx_frame(u8 function) {
  const struct ssf_modbus_frame_desc *normal;
  size_t i;

  normal = ssf_mems_modbus_find_frame(function & ~SSF_MEMS_MODBUS_EXCEPTION_FLAG);
  if (!normal || !(function & SSF_MEMS_MODBUS_EXCEPTION_FLAG))
    return normal;

  for (i = 0; i < SSF_FRAME_TABLE_SIZE; i++) {
    if (ssf_frame_table[i].rx_format == SSF_RX_EXCEPTION)
      return &ssf_frame_table[i];
  }
  return NULL;
}

/* 返回匹配的寄存器表项指针；地址未入表返回 NULL。 */
static inline const struct ssf_reg_desc *
ssf_mems_modbus_find_reg(u16 display_reg) {
  size_t i;

  for (i = 0; i < SSF_REG_TABLE_SIZE; i++) {
    if (ssf_reg_table[i].display_reg == display_reg)
      return &ssf_reg_table[i];
  }
  return NULL;
}

/* 返回匹配的块命令表项指针；命令 ID 未入表返回 NULL。 */
static inline const struct ssf_block_cmd_desc *
ssf_mems_modbus_find_block_cmd(enum ssf_block_cmd_id id) {
  size_t i;

  for (i = 0; i < SSF_BLOCK_CMD_TABLE_SIZE; i++) {
    if (ssf_block_cmd_table[i].id == id)
      return &ssf_block_cmd_table[i];
  }
  return NULL;
}
