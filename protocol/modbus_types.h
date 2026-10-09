#pragma once

#include <linux/bitops.h>
#include <linux/types.h>

/*
 * Modbus 共用类型：定义功能码、收发格式和各种“描述表项”。
 * 这里只描述协议规则，不保存某个具体设备的运行状态。
 */

/* RTU 公共参数：CRC 规则、异常标志、默认地址及帧/寄存器上限。 */
#define SSF_MEMS_MODBUS_CRC_LEN 2U
#define SSF_MEMS_MODBUS_CRC_INIT 0xffffU
#define SSF_MEMS_MODBUS_CRC_POLY 0xa001U
#define SSF_MEMS_MODBUS_EXCEPTION_FLAG 0x80U
#define SSF_MEMS_MODBUS_DEFAULT_SLAVE_ID 0x01U
#define SSF_MEMS_MODBUS_READ_MAX_REGS 125U
#define SSF_MEMS_MODBUS_WRITE_MAX_REGS 123U
#define SSF_MEMS_MODBUS_MAX_FRAME_LEN 256U

enum ssf_mems_modbus_func {
  SSF_MEMS_MODBUS_FUNC_READ = 0x03,         // 读保持寄存器
  SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE = 0x06, // 写单个寄存器
  SSF_MEMS_MODBUS_FUNC_WRITE_MULTI = 0x10,  // 写多个寄存器
};

/* 块命令的操作方向。 */
enum ssf_cmd_dir {
  SSF_CMD_READ,
  SSF_CMD_WRITE,
};

/* 寄存器权限位，可用 SSF_REG_READ | SSF_REG_WRITE 表示可读写。 */
enum ssf_reg_access {
  SSF_REG_READ = BIT(0),
  SSF_REG_WRITE = BIT(1),
};

/* 一个寄存器是 U16/S16，或是某个 U32 的高/低 16 位。 */
enum ssf_reg_type {
  SSF_REG_U16,
  SSF_REG_S16,
  SSF_REG_U32_HIGH,
  SSF_REG_U32_LOW,
};

/* 请求帧的组帧方式；NONE 表示该描述不用于发送。 */
enum ssf_tx_format {
  SSF_TX_READ_REGS,
  SSF_TX_WRITE_SINGLE,
  SSF_TX_WRITE_MULTI,
  SSF_TX_NONE,
};

/* 响应帧的解析方式，包括标准响应、异常响应和设备私有数据。 */
enum ssf_rx_format {
  SSF_RX_HOLDING_REGS,
  SSF_RX_WRITE_SINGLE_ECHO,
  SSF_RX_WRITE_MULTI_ACK,
  SSF_RX_EXCEPTION,
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
  size_t feature_offset;          // 在 ssf_mems_sensor_data 中的字段偏移
  u8 feature_width; /* 0 表示该寄存器不映射到特征缓存。 */
  u16 value_mask;                 // 写入特征缓存前保留的位
  const char *name;               // 便于阅读和调试的名称
};

/* 上层批量操作的逻辑编号，不是 Modbus 功能码。 */
enum ssf_block_cmd_id {
  SSF_BLOCK_READ_ALL_FEATURES,
  SSF_BLOCK_WRITE_WORK_PARAMETERS,
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
