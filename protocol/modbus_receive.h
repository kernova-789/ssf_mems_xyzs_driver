#pragma once

#include <linux/atomic.h>
#include <linux/serdev.h>
#include <linux/types.h>

enum ssf_mems_frame_type {
  SSF_MEMS_FRAME_MODBUS,
  SSF_MEMS_FRAME_RAW_VIB,
};

struct ssf_mems_frame_desc {
  enum ssf_mems_frame_type type;
  u8 slave_id;
};

enum ssf_mems_modbus_func {
  SSF_MEMS_MODBUS_FUNC_READ = 0x03,
  SSF_MEMS_MODBUS_FUNC_WRITE_SINGLE = 0x06,
  SSF_MEMS_MODBUS_FUNC_WRITE_MULTI = 0x10,
};

enum ssf_mems_rx_state {
  SSF_MEMS_RX_IDLE,
  SSF_MEMS_RX_FUNC,
  SSF_MEMS_RX_DATA,
  SSF_MEMS_RX_CRC,
  SSF_MEMS_RX_DONE,
  SSF_MEMS_RX_ERROR,
};

/*
 * 一个 frame_slot 就是一个独立的“候选帧解析上下文”。
 *
 * 多个 slot 可以同时存在，分别对同一串输入数据
 * 做不同的帧边界假设。
 */
struct ssf_mems_frame_slot {
  u8 *data;         //实际分配的帧缓冲区

  size_t data_len;  //data 缓冲区的容量
  size_t data_pos;  //当前已经接收了多少字节
  size_t frame_len; //根据帧头判断出来的“这一帧应该有多少字节”

  enum ssf_mems_rx_state state;

  u8 slave_id;
  u8 function;

  atomic_t in_use;
};

int ssf_mems_rx_push(struct serdev_device *serdev, const unsigned char *buf,
                     size_t count);

int ssf_mems_modbus_parse_frame(struct serdev_device *serdev);