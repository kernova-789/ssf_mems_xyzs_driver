#pragma once

#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/string.h>

#include "modbus/modbus_types.h"
#include "model/sensor_data.h"

/* 只有 modbus_table.c 定义表；其他文件共享同一份只读数据。 */
extern const struct ssf_modbus_frame_desc ssf_frame_table[];
extern const struct ssf_reg_desc ssf_reg_table[];
extern const struct ssf_block_cmd_desc ssf_block_cmd_table[];
extern const size_t ssf_frame_table_size;
extern const size_t ssf_reg_table_size;
extern const size_t ssf_block_cmd_table_size;
extern const struct ssf_mems_sensor_profile ssf_mems_sensor_profile;

#define SSF_FRAME_TABLE_SIZE ssf_frame_table_size
#define SSF_REG_TABLE_SIZE ssf_reg_table_size
#define SSF_BLOCK_CMD_TABLE_SIZE ssf_block_cmd_table_size
#define SSF_MEMS_SAMPLING_RATE_MAX_INDEX \
  (ssf_mems_sensor_profile.sampling_frequency_count - 1U)
#define SSF_MEMS_RAW_SAMPLING_RATE_MAX_INDEX \
  (ssf_mems_sensor_profile.raw_max_sampling_rate_index)

/* 按功能码查找请求帧描述。
 * 返回：描述指针，未找到时为 NULL。
 */
const struct ssf_modbus_frame_desc *ssf_mems_modbus_find_frame(u8 function);
/* 按功能码查找响应帧描述。
 * 返回：描述指针，未找到时为 NULL。
 */
const struct ssf_modbus_frame_desc *ssf_mems_modbus_find_rx_frame(u8 function);
/* 按手册地址查找寄存器。
 * 返回：表项指针，未找到时为 NULL。
 */
const struct ssf_reg_desc *ssf_mems_modbus_find_reg(u16 display_reg);
/* 按稳定名称查找寄存器。
 * 返回：表项指针，名称无效或未找到时为 NULL。
 */
const struct ssf_reg_desc *ssf_mems_modbus_find_reg_by_name(const char *name);
/* 按命令编号查找寄存器块。
 * 返回：块描述指针，未找到时为 NULL。
 */
const struct ssf_block_cmd_desc *
ssf_mems_modbus_find_block_cmd(enum ssf_block_cmd_id id);
/* 查找地址范围内的下一个可读特征。
 * 返回：表项指针，没有剩余特征时为 NULL。
 */
const struct ssf_reg_desc *ssf_mems_modbus_next_feature(u32 cursor, u32 end);
/* 检查型号表的编码、布局和名称是否有效。
 * 返回：0 有效，-EINVAL 无效；bad_name 指向出错配置或表项名称。
 */
int ssf_mems_modbus_validate_tables(const char **bad_name);


#ifdef SSF_MEMS_DEFINE_MODBUS_TABLE
/* 型号相关的编码、默认配置和位布局集中在这里，不在 IIO 中另建副本。 */
static const char *const ssf_sampling_frequencies[] = {
    "533.34", "888.9", "1066.68", "1333.35", "2666.7",
    "2963", "5333.4", "8889", "13333.5", "26667",
};
static const unsigned int ssf_baudrates[] = {
    9600, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 128000,
    230400, 256000, 460800, 500000, 512000, 600000, 750000, 921600, 1000000,
};
const struct ssf_mems_sensor_profile ssf_mems_sensor_profile = {
    .sampling_frequencies = ssf_sampling_frequencies,
    .sampling_frequency_count = ARRAY_SIZE(ssf_sampling_frequencies),
    .baudrates = ssf_baudrates,
    .baudrate_count = ARRAY_SIZE(ssf_baudrates),
    .default_work_baudrate = SSF_MEMS_BAUDRATE_128000,
    .default_sampling_rate_index = 6, /* 5333.4 Hz。 */
    .raw_max_sampling_rate_index = 9, /* 手册允许索引 0..9。 */
    .raw_accel_scale_nano = 4788403, /* ±16 g，转换为 m/s²。 */
    .raw_axis_order = {0, 1, 2}, /* XYZ 顺序与手册一致。 */
    .feature_enable_mask = 0xfc00,
    .feature_enable_shift = 10,
};

/* 帧格式表：描述 0x03/0x06/0x10 和异常响应的长度、字段位置及解析方式。 */
const struct ssf_modbus_frame_desc ssf_frame_table[] = {
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

/*
 * 寄存器表：把手册显示地址映射到线上地址，并记录权限、类型和解码位置。
 * 表项删除后不重编其他地址，稀疏特征由块读取逻辑自动跳过。
 */
const struct ssf_reg_desc ssf_reg_table[] = {
    /* 40001～40029：只读特征值。 */
    {
        .display_reg = 40001,
        .protocol_addr = 0x0000,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_HIGH_FREQ_ACC_RMS,
        .value_mask = 0xffff,
        .name = "x_high_freq_acc_rms",
    },
    {
        .display_reg = 40002,
        .protocol_addr = 0x0001,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_LOW_FREQ_VELOCITY_RMS,
        .value_mask = 0xffff,
        .name = "x_low_freq_velocity_rms",
    },
    {
        .display_reg = 40003,
        .protocol_addr = 0x0002,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_HIGH_FREQ_ACC_RMS,
        .value_mask = 0xffff,
        .name = "y_high_freq_acc_rms",
    },
    {
        .display_reg = 40004,
        .protocol_addr = 0x0003,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_LOW_FREQ_VELOCITY_RMS,
        .value_mask = 0xffff,
        .name = "y_low_freq_velocity_rms",
    },
    {
        .display_reg = 40005,
        .protocol_addr = 0x0004,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_HIGH_FREQ_ACC_RMS,
        .value_mask = 0xffff,
        .name = "z_high_freq_acc_rms",
    },
    {
        .display_reg = 40006,
        .protocol_addr = 0x0005,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_LOW_FREQ_VELOCITY_RMS,
        .value_mask = 0xffff,
        .name = "z_low_freq_velocity_rms",
    },
    {
        .display_reg = 40007,
        .protocol_addr = 0x0006,
        .access = SSF_REG_READ,
        .type = SSF_REG_S16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_TEMPERATURE,
        .value_mask = 0xffff,
        .name = "temperature",
    },
    {
        .display_reg = 40008,
        .protocol_addr = 0x0007,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_ACC_PEAK_TO_PEAK,
        .value_mask = 0xffff,
        .name = "x_acc_peak_to_peak",
    },
    {
        .display_reg = 40009,
        .protocol_addr = 0x0008,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_ACC_PEAK_TO_PEAK,
        .value_mask = 0xffff,
        .name = "y_acc_peak_to_peak",
    },
    {
        .display_reg = 40010,
        .protocol_addr = 0x0009,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_ACC_PEAK_TO_PEAK,
        .value_mask = 0xffff,
        .name = "z_acc_peak_to_peak",
    },
    {
        .display_reg = 40011,
        .protocol_addr = 0x000a,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_ACC_PEAK,
        .value_mask = 0xffff,
        .name = "x_acc_peak",
    },
    {
        .display_reg = 40012,
        .protocol_addr = 0x000b,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_ACC_PEAK,
        .value_mask = 0xffff,
        .name = "y_acc_peak",
    },
    {
        .display_reg = 40013,
        .protocol_addr = 0x000c,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_ACC_PEAK,
        .value_mask = 0xffff,
        .name = "z_acc_peak",
    },
    {
        .display_reg = 40014,
        .protocol_addr = 0x000d,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_ACC_RMS,
        .value_mask = 0xffff,
        .name = "x_acc_rms",
    },
    {
        .display_reg = 40015,
        .protocol_addr = 0x000e,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_ACC_RMS,
        .value_mask = 0xffff,
        .name = "y_acc_rms",
    },
    {
        .display_reg = 40016,
        .protocol_addr = 0x000f,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_ACC_RMS,
        .value_mask = 0xffff,
        .name = "z_acc_rms",
    },
    {
        .display_reg = 40017,
        .protocol_addr = 0x0010,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_KURTOSIS,
        .value_mask = 0xffff,
        .name = "x_kurtosis",
    },
    {
        .display_reg = 40018,
        .protocol_addr = 0x0011,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_KURTOSIS,
        .value_mask = 0xffff,
        .name = "y_kurtosis",
    },
    {
        .display_reg = 40019,
        .protocol_addr = 0x0012,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_KURTOSIS,
        .value_mask = 0xffff,
        .name = "z_kurtosis",
    },
    {
        .display_reg = 40020,
        .protocol_addr = 0x0013,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_VELOCITY_RMS,
        .value_mask = 0xffff,
        .name = "x_velocity_rms",
    },
    {
        .display_reg = 40021,
        .protocol_addr = 0x0014,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_VELOCITY_RMS,
        .value_mask = 0xffff,
        .name = "y_velocity_rms",
    },
    {
        .display_reg = 40022,
        .protocol_addr = 0x0015,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_VELOCITY_RMS,
        .value_mask = 0xffff,
        .name = "z_velocity_rms",
    },
    {
        .display_reg = 40023,
        .protocol_addr = 0x0016,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_SOUND_RMS,
        .value_mask = 0xffff,
        .name = "sound_rms_db",
    },
    {
        .display_reg = 40024,
        .protocol_addr = 0x0017,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_SOUND_PEAK,
        .value_mask = 0xffff,
        .name = "sound_peak_db",
    },
    {
        .display_reg = 40025,
        .protocol_addr = 0x0018,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_SOUND_PEAK_TO_PEAK,
        .value_mask = 0xffff,
        .name = "sound_peak_to_peak_db",
    },
    {
        .display_reg = 40026,
        .protocol_addr = 0x0019,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_ZERO_CROSSING_RATE,
        .value_mask = 0xffff,
        .name = "zero_crossing_rate_percent",
    },
    {
        .display_reg = 40027,
        .protocol_addr = 0x001a,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_SPECTRAL_CENTROID,
        .value_mask = 0xffff,
        .name = "spectral_centroid_hz",
    },
    {
        .display_reg = 40028,
        .protocol_addr = 0x001b,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_SPECTRAL_FLUX,
        .value_mask = 0xffff,
        .name = "spectral_flux",
    },
    {
        .display_reg = 40029,
        .protocol_addr = 0x001c,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_STARTUP_FLAGS,
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
    /* 按手册保留描述；当前采集策略不使用连续模式。 */
    {
        .display_reg = 40058,
        .protocol_addr = 0x0039,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_RAW_XYZ,
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
        /* 支持单寄存器写入，无需重写整个工作块。 */
        .write_single = true,
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
        /* 写 1 保存通信参数并重启传感器。 */
        .display_reg = 40110,
        .protocol_addr = 0x006D,
        .access = SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .write_single = true,
        .name = "save_config_and_reboot",
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

/* 块命令表：把“读全部特征”、“写工作参数”这类高层操作组织成固定地址范围。 */
const struct ssf_block_cmd_desc ssf_block_cmd_table[] = {
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

const size_t ssf_frame_table_size = ARRAY_SIZE(ssf_frame_table);
const size_t ssf_reg_table_size = ARRAY_SIZE(ssf_reg_table);
const size_t ssf_block_cmd_table_size = ARRAY_SIZE(ssf_block_cmd_table);
#endif /* SSF_MEMS_DEFINE_MODBUS_TABLE */
