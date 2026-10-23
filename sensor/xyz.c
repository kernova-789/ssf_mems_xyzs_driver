// SPDX-License-Identifier: GPL-2.0-only
#include "sensor.h"
#include "raw/raw_capture.h"
#include "raw/raw_stream.h"
#include <linux/kernel.h>

static const char *const ssf_sampling_frequencies[] = {
    "533.34", "888.9", "1066.68", "1333.35", "2666.7",
    "2963", "5333.4", "8889", "13333.5", "26667",
};
static const unsigned int ssf_baudrates[] = {
    9600, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 128000,
    230400, 256000, 460800, 500000, 512000, 600000, 750000, 921600, 1000000,
};

/* 实机 XYZ 在采样期间发送；不能按 XYZS 的缓冲后上传行为使用。
 * 392 字节/64 组、8N1，再预留 20% 余量（向上取整）。
 * 表项按采样率编码索引；高采样率仍可用于特征读取。
 */
#define XYZ_RAW_MIN_BAUDRATE(millihz) \
    DIV_ROUND_UP((millihz) * (u64)SSF_MEMS_RAW_PACKET_BYTES * 10U * 12U, \
                 SSF_MEMS_RAW_SAMPLES_PER_PACKET * 1000U * 10U)
static const unsigned int xyz_raw_min_baudrates[] = {
    XYZ_RAW_MIN_BAUDRATE(533340),  XYZ_RAW_MIN_BAUDRATE(888900),
    XYZ_RAW_MIN_BAUDRATE(1066680), XYZ_RAW_MIN_BAUDRATE(1333350),
    XYZ_RAW_MIN_BAUDRATE(2666700), XYZ_RAW_MIN_BAUDRATE(2963000),
    XYZ_RAW_MIN_BAUDRATE(5333400), XYZ_RAW_MIN_BAUDRATE(8889000),
    XYZ_RAW_MIN_BAUDRATE(13333500), XYZ_RAW_MIN_BAUDRATE(26667000),
};
#undef XYZ_RAW_MIN_BAUDRATE

static const struct ssf_reg_desc ssf_reg_table[] = {
    {
        .display_reg = 40001,
        .protocol_addr = 0x0000,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_ACC_PEAK,
        SSF_MEMS_FEATURE_MEMBER(xyz.x.acc_peak_x100, ACCEL),
        .value_mask = 0xffff,
        .name = "x_acc_peak",
    },
    {
        .display_reg = 40002,
        .protocol_addr = 0x0001,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_ACC_PEAK_TO_PEAK,
        SSF_MEMS_FEATURE_MEMBER(xyz.x.acc_peak_to_peak_x100, ACCEL),
        .value_mask = 0xffff,
        .name = "x_acc_peak_to_peak",
    },
    {
        .display_reg = 40003,
        .protocol_addr = 0x0002,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_ACC_RMS,
        SSF_MEMS_FEATURE_MEMBER(xyz.x.acc_rms_x100, ACCEL),
        .value_mask = 0xffff,
        .name = "x_acc_rms",
    },
    {
        .display_reg = 40004,
        .protocol_addr = 0x0003,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_KURTOSIS,
        SSF_MEMS_FEATURE_MEMBER(xyz.x.kurtosis_x100, CENTI),
        .value_mask = 0xffff,
        .name = "x_kurtosis",
    },
    {
        .display_reg = 40005,
        .protocol_addr = 0x0004,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_VELOCITY_RMS,
        SSF_MEMS_FEATURE_MEMBER(xyz.x.velocity_rms_x100, VELOCITY),
        .value_mask = 0xffff,
        .name = "x_velocity_rms",
    },
    {
        .display_reg = 40006,
        .protocol_addr = 0x0005,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_X_DISPLACEMENT_PEAK_TO_PEAK,
        SSF_MEMS_FEATURE_MEMBER(xyz.x.displacement_peak_to_peak_um, DISPLACEMENT),
        .value_mask = 0xffff,
        .name = "x_displacement_peak_to_peak",
    },
    {
        .display_reg = 40007,
        .protocol_addr = 0x0006,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_ACC_PEAK,
        SSF_MEMS_FEATURE_MEMBER(xyz.y.acc_peak_x100, ACCEL),
        .value_mask = 0xffff,
        .name = "y_acc_peak",
    },
    {
        .display_reg = 40008,
        .protocol_addr = 0x0007,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_ACC_PEAK_TO_PEAK,
        SSF_MEMS_FEATURE_MEMBER(xyz.y.acc_peak_to_peak_x100, ACCEL),
        .value_mask = 0xffff,
        .name = "y_acc_peak_to_peak",
    },
    {
        .display_reg = 40009,
        .protocol_addr = 0x0008,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_ACC_RMS,
        SSF_MEMS_FEATURE_MEMBER(xyz.y.acc_rms_x100, ACCEL),
        .value_mask = 0xffff,
        .name = "y_acc_rms",
    },
    {
        .display_reg = 40010,
        .protocol_addr = 0x0009,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_KURTOSIS,
        SSF_MEMS_FEATURE_MEMBER(xyz.y.kurtosis_x100, CENTI),
        .value_mask = 0xffff,
        .name = "y_kurtosis",
    },
    {
        .display_reg = 40011,
        .protocol_addr = 0x000a,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_VELOCITY_RMS,
        SSF_MEMS_FEATURE_MEMBER(xyz.y.velocity_rms_x100, VELOCITY),
        .value_mask = 0xffff,
        .name = "y_velocity_rms",
    },
    {
        .display_reg = 40012,
        .protocol_addr = 0x000b,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Y_DISPLACEMENT_PEAK_TO_PEAK,
        SSF_MEMS_FEATURE_MEMBER(xyz.y.displacement_peak_to_peak_um, DISPLACEMENT),
        .value_mask = 0xffff,
        .name = "y_displacement_peak_to_peak",
    },
    {
        .display_reg = 40013,
        .protocol_addr = 0x000c,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_ACC_PEAK,
        SSF_MEMS_FEATURE_MEMBER(xyz.z.acc_peak_x100, ACCEL),
        .value_mask = 0xffff,
        .name = "z_acc_peak",
    },
    {
        .display_reg = 40014,
        .protocol_addr = 0x000d,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_ACC_PEAK_TO_PEAK,
        SSF_MEMS_FEATURE_MEMBER(xyz.z.acc_peak_to_peak_x100, ACCEL),
        .value_mask = 0xffff,
        .name = "z_acc_peak_to_peak",
    },
    {
        .display_reg = 40015,
        .protocol_addr = 0x000e,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_ACC_RMS,
        SSF_MEMS_FEATURE_MEMBER(xyz.z.acc_rms_x100, ACCEL),
        .value_mask = 0xffff,
        .name = "z_acc_rms",
    },
    {
        .display_reg = 40016,
        .protocol_addr = 0x000f,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_KURTOSIS,
        SSF_MEMS_FEATURE_MEMBER(xyz.z.kurtosis_x100, CENTI),
        .value_mask = 0xffff,
        .name = "z_kurtosis",
    },
    {
        .display_reg = 40017,
        .protocol_addr = 0x0010,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_VELOCITY_RMS,
        SSF_MEMS_FEATURE_MEMBER(xyz.z.velocity_rms_x100, VELOCITY),
        .value_mask = 0xffff,
        .name = "z_velocity_rms",
    },
    {
        .display_reg = 40018,
        .protocol_addr = 0x0011,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_Z_DISPLACEMENT_PEAK_TO_PEAK,
        SSF_MEMS_FEATURE_MEMBER(xyz.z.displacement_peak_to_peak_um, DISPLACEMENT),
        .value_mask = 0xffff,
        .name = "z_displacement_peak_to_peak",
    },
    {
        .display_reg = 40019,
        .protocol_addr = 0x0012,
        .access = SSF_REG_READ,
        .type = SSF_REG_S16,
        .read_format = SSF_RX_HOLDING_REGS,
        .feature_field = SSF_MEMS_FIELD_TEMPERATURE,
        SSF_MEMS_FEATURE_MEMBER(xyz.temperature_x100, TEMPERATURE),
        .value_mask = 0xffff,
        .feature_multiplier = 10, /* 手册 ×10 -> 缓存 ×100。 */
        .name = "temperature",
    },
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
};

static const struct ssf_block_cmd_desc ssf_block_cmd_table[] = {
    {
        .id = SSF_BLOCK_READ_ALL_FEATURES,
        .function = SSF_MEMS_MODBUS_FUNC_READ,
        .direction = SSF_CMD_READ,
        .rx_format = SSF_RX_HOLDING_REGS,
        .decode_kind = SSF_DECODE_FEATURES,
        .start_display_reg = 40001,
        .reg_count = 19, /* 原始地址跨度；删除特征表项后无需调整。 */
        .sparse_read = true,
        .name = "read_all_features",
    },
};

static int xyz_read_features(struct ssf_mems_data *data,
                                unsigned int timeout_ms) {
  return ssf_mems_sensor_read_features(data, timeout_ms);
}
static int xyz_collect_raw(struct ssf_mems_data *data,
                              unsigned int duration_ms, u32 samples) {
  return ssf_mems_raw_collect_finite(data, duration_ms, samples);
}
static const struct ssf_mems_sensor_ops xyz_ops = {
  .read_features = xyz_read_features,
  .collect_raw = xyz_collect_raw,
};

const struct ssf_mems_sensor ssf_mems_xyz = {
  .name = "ssf_mems_xyz",
  .raw_name = "ssf_mems_xyz_raw",
  .feature_size = sizeof(struct ssf_mems_xyz_features),
  .regs = ssf_reg_table,
  .reg_count = ARRAY_SIZE(ssf_reg_table),
  .blocks = ssf_block_cmd_table,
  .block_count = ARRAY_SIZE(ssf_block_cmd_table),
  .profile = {
    .sampling_frequencies = ssf_sampling_frequencies,
    .sampling_frequency_count = ARRAY_SIZE(ssf_sampling_frequencies),
    .sampling_length_count = 6, /* 1024、2048、4096、8192、16384、32768。 */
    .baudrates = ssf_baudrates,
    .baudrate_count = ARRAY_SIZE(ssf_baudrates),
    /* 推荐默认运行组合：128000 bit/s / 1333.35 Hz，兼顾特征和定长采集。
     * 双路实机已验证；更高波特率未通过稳定性复测，不作为默认值。
     */
    .default_work_baudrate = SSF_MEMS_BAUDRATE_128000,
    .default_sampling_rate_index = 3, /* 1333.35 Hz。 */
    .raw_max_sampling_rate_index = 9, /* 手册允许索引 0..9。 */
    .raw_batch_max_samples = 1024, /* XYZ 暂保持已验证的长度。 */
    .raw_min_baudrates = xyz_raw_min_baudrates,
    .raw_min_baudrate_count = ARRAY_SIZE(xyz_raw_min_baudrates),
    .raw_accel_scale_nano = 4788403, /* ±16 g，转换为 m/s²。 */
    .raw_axis_order = {0, 1, 2}, /* XYZ 顺序与手册一致。 */
    .feature_enable_mask = 0,
    .feature_enable_shift = 0,
  },
  .ops = &xyz_ops,
  .supports_ota = false,
};
