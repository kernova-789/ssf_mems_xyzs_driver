#pragma once

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/types.h>

enum ssf_cmd_dir {
  SSF_CMD_READ,
  SSF_CMD_WRITE,
};

enum ssf_frame_type {
  SSF_FRAME_FIXED,
  SSF_FRAME_VARIABLE,
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

/* 一个表项对应一个 16 位寄存器，不再为读写请求重复建项。 */
struct ssf_reg_desc {
  u16 display_reg;
  u16 protocol_addr;
  unsigned int access;
  enum ssf_reg_type type;
  enum ssf_frame_type read_frame_type;
  bool write_single; /* 是否已明确支持 0x06；与可写权限分别记录。 */
  const char *name;
};

enum ssf_block_cmd_id {
  SSF_BLOCK_READ_ALL_FEATURES,
  SSF_BLOCK_WRITE_WORK_PARAMETERS,
};

/*
 * 手册明确规定的整块命令。起始协议地址从寄存器属性表获取，
 * 避免两张表分别维护同一个地址映射。
 */
struct ssf_block_cmd_desc {
  enum ssf_block_cmd_id id;
  u8 function;
  enum ssf_cmd_dir direction;
  enum ssf_frame_type frame_type;
  u16 start_display_reg;
  u16 reg_count;
  const char *name;
};

static const struct ssf_reg_desc ssf_reg_table[] = {
    /* 40001～40029：只读特征值。 */
    {
        .display_reg = 40001,
        .protocol_addr = 0x0000,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "x_high_freq_acc_rms",
    },
    {
        .display_reg = 40002,
        .protocol_addr = 0x0001,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "x_low_freq_velocity_rms",
    },
    {
        .display_reg = 40003,
        .protocol_addr = 0x0002,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "y_high_freq_acc_rms",
    },
    {
        .display_reg = 40004,
        .protocol_addr = 0x0003,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "y_low_freq_velocity_rms",
    },
    {
        .display_reg = 40005,
        .protocol_addr = 0x0004,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "z_high_freq_acc_rms",
    },
    {
        .display_reg = 40006,
        .protocol_addr = 0x0005,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "z_low_freq_velocity_rms",
    },
    {
        .display_reg = 40007,
        .protocol_addr = 0x0006,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "temperature",
    },
    {
        .display_reg = 40008,
        .protocol_addr = 0x0007,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "x_acc_peak_to_peak",
    },
    {
        .display_reg = 40009,
        .protocol_addr = 0x0008,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "y_acc_peak_to_peak",
    },
    {
        .display_reg = 40010,
        .protocol_addr = 0x0009,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "z_acc_peak_to_peak",
    },
    {
        .display_reg = 40011,
        .protocol_addr = 0x000a,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "x_acc_peak",
    },
    {
        .display_reg = 40012,
        .protocol_addr = 0x000b,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "y_acc_peak",
    },
    {
        .display_reg = 40013,
        .protocol_addr = 0x000c,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "z_acc_peak",
    },
    {
        .display_reg = 40014,
        .protocol_addr = 0x000d,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "x_acc_rms",
    },
    {
        .display_reg = 40015,
        .protocol_addr = 0x000e,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "y_acc_rms",
    },
    {
        .display_reg = 40016,
        .protocol_addr = 0x000f,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "z_acc_rms",
    },
    {
        .display_reg = 40017,
        .protocol_addr = 0x0010,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "x_kurtosis",
    },
    {
        .display_reg = 40018,
        .protocol_addr = 0x0011,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "y_kurtosis",
    },
    {
        .display_reg = 40019,
        .protocol_addr = 0x0012,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "z_kurtosis",
    },
    {
        .display_reg = 40020,
        .protocol_addr = 0x0013,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "x_velocity_rms",
    },
    {
        .display_reg = 40021,
        .protocol_addr = 0x0014,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "y_velocity_rms",
    },
    {
        .display_reg = 40022,
        .protocol_addr = 0x0015,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "z_velocity_rms",
    },
    {
        .display_reg = 40023,
        .protocol_addr = 0x0016,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "sound_rms",
    },
    {
        .display_reg = 40024,
        .protocol_addr = 0x0017,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "sound_peak",
    },
    {
        .display_reg = 40025,
        .protocol_addr = 0x0018,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "sound_peak_to_peak",
    },
    {
        .display_reg = 40026,
        .protocol_addr = 0x0019,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "zero_crossing_rate",
    },
    {
        .display_reg = 40027,
        .protocol_addr = 0x001a,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "spectral_centroid",
    },
    {
        .display_reg = 40028,
        .protocol_addr = 0x001b,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "spectral_flux",
    },
    {
        .display_reg = 40029,
        .protocol_addr = 0x001c,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "startup_flag",
    },

    /* 40050～40053：配置寄存器；保留原有 0x06 支持范围。 */
    {
        .display_reg = 40050,
        .protocol_addr = 0x0031,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .write_single = true,
        .name = "auto_report_enable",
    },
    {
        .display_reg = 40051,
        .protocol_addr = 0x0032,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .write_single = true,
        .name = "auto_report_time",
    },
    {
        .display_reg = 40052,
        .protocol_addr = 0x0033,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .write_single = true,
        .name = "sampling_rate",
    },
    {
        .display_reg = 40053,
        .protocol_addr = 0x0034,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .write_single = true,
        .name = "sampling_length",
    },

    /* 这些地址触发特殊数据响应，不能按普通 0x03 寄存器响应读取。 */
    {
        .display_reg = 40054,
        .protocol_addr = 0x0035,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_VARIABLE,
        .name = "x_raw_data",
    },
    {
        .display_reg = 40055,
        .protocol_addr = 0x0036,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_VARIABLE,
        .name = "y_raw_data",
    },
    {
        .display_reg = 40056,
        .protocol_addr = 0x0037,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_VARIABLE,
        .name = "z_raw_data",
    },
    {
        .display_reg = 40058,
        .protocol_addr = 0x0039,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_VARIABLE,
        .name = "start_xyz_continuous",
    },
    {
        .display_reg = 40059,
        .protocol_addr = 0x003a,
        .access = SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .write_single = true,
        .name = "stop_xyz_continuous",
    },
    {
        .display_reg = 40060,
        .protocol_addr = 0x003b,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_VARIABLE,
        .name = "start_custom_length",
    },

    /* 40061～40071：手册标记为 RW，40061/40062 合成一个 U32 字段。 */
    {
        .display_reg = 40061,
        .protocol_addr = 0x003c,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U32_HIGH,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "custom_length_high",
    },
    {
        .display_reg = 40062,
        .protocol_addr = 0x003d,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U32_LOW,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "custom_length_low",
    },
    {
        .display_reg = 40063,
        .protocol_addr = 0x003e,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "raw_sound_4096_enable",
    },
    {
        .display_reg = 40064,
        .protocol_addr = 0x003f,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "x_acc_threshold",
    },
    {
        .display_reg = 40065,
        .protocol_addr = 0x0040,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "y_acc_threshold",
    },
    {
        .display_reg = 40066,
        .protocol_addr = 0x0041,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "z_acc_threshold",
    },
    {
        .display_reg = 40067,
        .protocol_addr = 0x0042,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "x_velocity_threshold",
    },
    {
        .display_reg = 40068,
        .protocol_addr = 0x0043,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "y_velocity_threshold",
    },
    {
        .display_reg = 40069,
        .protocol_addr = 0x0044,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "z_velocity_threshold",
    },
    {
        .display_reg = 40070,
        .protocol_addr = 0x0045,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "parameter_switch",
    },
    {
        .display_reg = 40071,
        .protocol_addr = 0x0046,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "continuous_raw_sound",
    },

    /* 通信参数与固件版本。 */
    {
        .display_reg = 40101,
        .protocol_addr = 0x0064,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .write_single = true,
        .name = "slave_addr",
    },
    {
        .display_reg = 40102,
        .protocol_addr = 0x0065,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .write_single = true,
        .name = "baudrate",
    },
    {
        .display_reg = 40103,
        .protocol_addr = 0x0066,
        .access = SSF_REG_READ | SSF_REG_WRITE,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .write_single = true,
        .name = "parity",
    },
    {
        .display_reg = 40121,
        .protocol_addr = 0x0078,
        .access = SSF_REG_READ,
        .type = SSF_REG_U16,
        .read_frame_type = SSF_FRAME_FIXED,
        .name = "firmware_version",
    },
};

static const struct ssf_block_cmd_desc ssf_block_cmd_table[] = {
    {
        .id = SSF_BLOCK_READ_ALL_FEATURES,
        .function = 0x03,
        .direction = SSF_CMD_READ,
        .frame_type = SSF_FRAME_FIXED,
        .start_display_reg = 40001,
        .reg_count = 29,
        .name = "read_all_features",
    },
    {
        .id = SSF_BLOCK_WRITE_WORK_PARAMETERS,
        .function = 0x10,
        .direction = SSF_CMD_WRITE,
        .frame_type = SSF_FRAME_FIXED,
        .start_display_reg = 40061,
        .reg_count = 10,
        .name = "write_work_parameters",
    },
};

#define SSF_REG_TABLE_SIZE ARRAY_SIZE(ssf_reg_table)
#define SSF_BLOCK_CMD_TABLE_SIZE ARRAY_SIZE(ssf_block_cmd_table)

/* 公共查表入口，后续 modbus.c 也可使用同一份寄存器/块命令定义。 */
static inline const struct ssf_reg_desc *
ssf_mems_modbus_find_reg(u16 display_reg) {
  size_t i;

  for (i = 0; i < SSF_REG_TABLE_SIZE; i++) {
    if (ssf_reg_table[i].display_reg == display_reg)
      return &ssf_reg_table[i];
  }

  return NULL;
}

static inline const struct ssf_block_cmd_desc *
ssf_mems_modbus_find_block_cmd(enum ssf_block_cmd_id id) {
  size_t i;

  for (i = 0; i < SSF_BLOCK_CMD_TABLE_SIZE; i++) {
    if (ssf_block_cmd_table[i].id == id)
      return &ssf_block_cmd_table[i];
  }

  return NULL;
}
