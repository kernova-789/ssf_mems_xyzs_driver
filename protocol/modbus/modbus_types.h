#pragma once

#include <linux/bitops.h>
#include <linux/types.h>
#include "model/sensor_fields.h"

/*
 * Modbus 共用类型：定义功能码、收发格式和各种“描述表项”。
 * 这里只描述协议规则，不保存某个具体设备的运行状态。
 */

/* RTU 公共参数：CRC 规则、异常标志、默认地址及帧/寄存器上限。 */
#define SSF_MEMS_MODBUS_CRC_LEN 2U /* 普通 RTU 帧末尾 CRC 字节数。 */
#define SSF_MEMS_MODBUS_CRC_INIT 0xffffU /* CRC16 初值。 */
#define SSF_MEMS_MODBUS_CRC_POLY 0xa001U /* 反射形式 CRC 多项式。 */
#define SSF_MEMS_MODBUS_EXCEPTION_FLAG 0x80U /* 异常应答功能码最高位。 */
#define SSF_MEMS_MODBUS_DEFAULT_SLAVE_ID 0x01U /* 手册默认从站地址。 */
#define SSF_MEMS_MODBUS_READ_MAX_REGS 125U /* 一次普通读取的最大寄存器数。 */
#define SSF_MEMS_MODBUS_WRITE_MAX_REGS 123U /* 一次 0x10 写入的最大寄存器数。 */
#define SSF_MEMS_MODBUS_MAX_FRAME_LEN 256U /* 标准 RTU 帧容量，不含私有原始/OTA 帧。 */

/* 40102 的波特率编码，枚举值不是每秒位数。 */
enum ssf_mems_baudrate {
  SSF_MEMS_BAUDRATE_DEFAULT = 0, /* 默认速率编码，对应 9600 bit/s。 */
  SSF_MEMS_BAUDRATE_2400 = 1, /* 2400 bit/s。 */
  SSF_MEMS_BAUDRATE_4800 = 2, /* 4800 bit/s。 */
  SSF_MEMS_BAUDRATE_9600 = 3, /* 9600 bit/s，显式编码。 */
  SSF_MEMS_BAUDRATE_19200 = 4, /* 19200 bit/s。 */
  SSF_MEMS_BAUDRATE_38400 = 5, /* 38400 bit/s。 */
  SSF_MEMS_BAUDRATE_57600 = 6, /* 57600 bit/s。 */
  SSF_MEMS_BAUDRATE_115200 = 7, /* 115200 bit/s。 */
  SSF_MEMS_BAUDRATE_128000 = 8, /* 128000 bit/s，当前正式工作速率。 */
  SSF_MEMS_BAUDRATE_230400 = 9, /* 230400 bit/s。 */
  SSF_MEMS_BAUDRATE_256000 = 10, /* 256000 bit/s。 */
  SSF_MEMS_BAUDRATE_460800 = 11, /* 460800 bit/s。 */
  SSF_MEMS_BAUDRATE_500000 = 12, /* 500000 bit/s。 */
  SSF_MEMS_BAUDRATE_512000 = 13, /* 512000 bit/s。 */
  SSF_MEMS_BAUDRATE_600000 = 14, /* 600000 bit/s。 */
  SSF_MEMS_BAUDRATE_750000 = 15, /* 750000 bit/s。 */
  SSF_MEMS_BAUDRATE_921600 = 16, /* 921600 bit/s。 */
  SSF_MEMS_BAUDRATE_1000000 = 17, /* 1000000 bit/s，当前型号最高编码速率。 */
  SSF_MEMS_BAUDRATE_MAX, /* 编码数量/非法值边界，不能写入传感器。 */
};

/* 标准寄存器读写功能码，数值直接出现在 RTU 帧中。 */
enum ssf_mems_modbus_func {
  SSF_MEMS_MODBUS_FUNC_READ = 0x03,         // 读保持寄存器
  SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE = 0x06, // 写单个寄存器
  SSF_MEMS_MODBUS_FUNC_WRITE_MULTI = 0x10,  // 写多个寄存器
};

/* 块命令的操作方向。 */
enum ssf_cmd_dir {
  SSF_CMD_READ, /* 从传感器读取。 */
  SSF_CMD_WRITE, /* 向传感器写入。 */
};

/* 寄存器权限位，可用 SSF_REG_READ | SSF_REG_WRITE 表示可读写。 */
enum ssf_reg_access {
  SSF_REG_READ = BIT(0), /* 允许读取。 */
  SSF_REG_WRITE = BIT(1), /* 允许写入。 */
};

/* 一个寄存器是 U16/S16，或是某个 U32 的高/低 16 位。 */
enum ssf_reg_type {
  SSF_REG_U16, /* 无符号 16 位值。 */
  SSF_REG_S16, /* 二补码有符号 16 位值。 */
  SSF_REG_U32_HIGH, /* 无符号 32 位值的高 16 位。 */
  SSF_REG_U32_LOW, /* 无符号 32 位值的低 16 位。 */
};

/* 请求帧的组帧方式；NONE 表示该描述不用于发送。 */
enum ssf_tx_format {
  SSF_TX_READ_REGS, /* 0x03：起始地址和寄存器数量。 */
  SSF_TX_WRITE_SINGLE, /* 0x06：地址和一个写入值。 */
  SSF_TX_WRITE_MULTI, /* 0x10：地址、数量及连续写入值。 */
  SSF_TX_NONE, /* 只描述接收，不允许构造请求。 */
};

/* 响应帧的解析方式，包括标准响应、异常响应和设备私有数据。 */
enum ssf_rx_format {
  SSF_RX_HOLDING_REGS, /* 0x03：字节数和寄存器数据。 */
  SSF_RX_WRITE_SINGLE_ECHO, /* 0x06：回显地址和写入值。 */
  SSF_RX_WRITE_MULTI_ACK, /* 0x10：确认地址和写入数量。 */
  SSF_RX_EXCEPTION, /* 功能码最高位置位，并携带异常码。 */
  SSF_RX_RAW_AXIS, /* 私有原始轴数据，通用读取接口不支持。 */
  SSF_RX_RAW_XYZ, /* 私有 0x15 XYZ 流，由独立流解析器处理。 */
};

/* 块数据是作为普通寄存器数组，还是映射到传感器特征结构体。 */
enum ssf_decode_kind {
  SSF_DECODE_REGISTERS, //当作普通寄存器数组
  SSF_DECODE_FEATURES,  //根据寄存器表映射到 ssf_mems_sensor_data 特征结构体中
};

/* 描述一种功能码的帧布局，实际组帧/解析算法在 modbus.c。 */
struct ssf_modbus_frame_desc {
  u8 function;                   // Modbus 功能码
  enum ssf_tx_format tx_format;  // 请求组帧方式
  enum ssf_rx_format rx_format;  // 响应解析方式
  u16 max_reg_count;             // 一帧最多操作的寄存器数
  u8 tx_base_len;                // 请求帧固定长度
  u8 tx_bytes_per_reg;           // 每个寄存器增加的请求字节数
  u8 tx_data_offset;             // 请求数据起始位置
  u8 address_offset;             // 寄存器地址位置
  u8 quantity_offset;            // 数量位置；0x06 中用于放写入值
  u8 tx_byte_count_offset;       // 多寄存器写的字节数位置
  u8 rx_base_len;                // 响应帧固定长度
  u8 rx_bytes_per_reg;           // 每个寄存器占用的响应字节数
  u8 rx_byte_count_offset;       // 响应字节数位置
  u8 rx_data_offset;             // 响应数据起始位置
};

/* 描述一个寄存器；显示地址与线上协议地址分开保存。 */
struct ssf_reg_desc {
  u16 display_reg;                // 手册显示地址，如 40001
  u16 protocol_addr;              // 线上地址，如 40001 对应 0x0000
  unsigned int access;            // 读/写权限位
  enum ssf_reg_type type;         // U16/S16 或 U32 的高/低半字
  enum ssf_rx_format read_format; // 读取时预期的响应格式
  bool write_single;              // 是否允许用 0x06 单独写入
  enum ssf_mems_sensor_field feature_field; /* NONE 表示不映射特征。 */
  u16 value_mask;                 // 写入特征缓存前保留的位
  u16 feature_multiplier; /* 特征单位换算的分子；0 按 1 处理。 */
  u16 feature_divisor; /* 特征单位换算的分母；0 按 1 处理。 */
  const char *name;               // 表内唯一、稳定的查询名称，同时用于调试
};

/* 当前型号的配置含义；上层只使用规范化后的特征单位和配置编号。 */
struct ssf_mems_sensor_profile {
  const char *const *sampling_frequencies; /* Hz 文本，按采样率编码索引。 */
  size_t sampling_frequency_count;
  const unsigned int *baudrates; /* bit/s，按传感器波特率编码索引。 */
  size_t baudrate_count;
  enum ssf_mems_baudrate default_work_baudrate; /* 驱动首次连接的目标编码。 */
  u16 default_sampling_rate_index;
  u16 raw_max_sampling_rate_index; /* 型号支持的定长采样率上限。 */
  u32 raw_accel_scale_nano; /* 原始计数换算为 m/s² 的纳米比例。 */
  u8 raw_axis_order[3]; /* 手册XYZ对应线上三轴的位置，取值0..2。 */
  u16 feature_enable_mask;
  u8 feature_enable_shift;
};

/* 上层批量操作的逻辑编号，不是 Modbus 功能码。 */
enum ssf_block_cmd_id {
  SSF_BLOCK_READ_ALL_FEATURES, /* 分段读取全部保留的特征表项。 */
  SSF_BLOCK_WRITE_WORK_PARAMETERS, /* 批量写入 40061～40070 工作参数。 */
};

/* 描述一个高层批量命令；sparse_read 表示允许跳过缺失表项分段读取。 */
struct ssf_block_cmd_desc {
  enum ssf_block_cmd_id id;       // 块命令编号
  u8 function;                    // 使用的 Modbus 功能码
  enum ssf_cmd_dir direction;     // 读或写
  enum ssf_rx_format rx_format;   // 预期响应格式
  enum ssf_decode_kind decode_kind; // 数据的最终解码方式
  u16 start_display_reg;          // 起始显示寄存器
  u16 reg_count;                  // 命令覆盖的寄存器数
  bool sparse_read;               // 是否允许稀疏分段读取
  const char *name;               // 命令名称
};

/* 一次实际请求的快照；解析响应时用它找回功能码、起始地址和数量。 */
struct ssf_modbus_transfer {
  const struct ssf_modbus_frame_desc *frame; // 本次使用的帧描述
  const struct ssf_block_cmd_desc *block;    // 所属块命令，普通请求为 NULL
  u16 display_reg;                           // 上层使用的显示地址
  u16 protocol_addr;                         // 实际发送的协议地址
  u16 reg_count;                             // 本次操作的寄存器数
};
