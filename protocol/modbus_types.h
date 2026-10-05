#pragma once

#include <linux/bitops.h>
#include <linux/types.h>

/* 共用线格式与请求描述类型，不包含设备状态或上层模块接口。 */
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

