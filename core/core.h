#pragma once
#include <linux/types.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>

#define SSF_MEMS_FRAME_SLOT_NUM    4

#define SSF_MEMS_FRAME_SLOT_FREE    0
#define SSF_MEMS_FRAME_SLOT_USED    1

struct ssf_mems_frame_slot {
	u8 *data;               //存储临时的帧数据
  u32 data_len;           //数据的长度
  u32 data_pos;           //当前数据被索引的位置
	atomic_t in_use;        //0 → 未使用，可以占用。1 → 正在使用，不能再被其他执行者占用
};

struct ssf_mems_xyzs_data {
  struct serdev_device *serdev;
  struct kfifo *rx_fifo;    //接收FIFO
  spinlock_t rx_fifo_lock;
  int parse_frame_state;    //用于帧接收状态机

  struct ssf_mems_frame_slot frame[SSF_MEMS_FRAME_SLOT_NUM];
};
