#pragma once

#include <linux/kfifo.h>
#include <linux/serdev.h>
#include <linux/spinlock.h>

#include "modbus_receive.h"

#define SSF_MEMS_FRAME_SLOT_NUM 4

#define SSF_MEMS_FRAME_SLOT_FREE 0
#define SSF_MEMS_FRAME_SLOT_USED 1

struct ssf_mems_xyzs_data {
  struct serdev_device *serdev;

  struct kfifo rx_fifo;
  spinlock_t rx_fifo_lock;

  struct ssf_mems_frame_slot frame[SSF_MEMS_FRAME_SLOT_NUM];
};