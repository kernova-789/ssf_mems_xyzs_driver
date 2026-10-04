#pragma once

#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/serdev.h>
#include <linux/spinlock.h>

#include "modbus.h"
#include "modbus_receive.h"
#include "modbus_request.h"

#define SSF_MEMS_FRAME_SLOT_NUM 4

#define SSF_MEMS_FRAME_SLOT_FREE 0
#define SSF_MEMS_FRAME_SLOT_USED 1

struct ssf_mems_xyzs_data {
  struct serdev_device *serdev;
  struct kfifo rx_fifo;       //收到的字节流放进这里
  spinlock_t rx_fifo_lock;    //读写rx_fifo时的锁

  struct work_struct rx_work; //从fifo中分离完整帧的工作
  struct ssf_mems_modbus_request_state modbus_req;

  /* 最近一次完整解析的 40001 ~ 40029 传感器特征值。 */
  struct mutex sensor_data_lock;
  struct ssf_mems_sensor_data sensor_data;
  bool sensor_data_valid;

  struct ssf_mems_frame_slot frame[SSF_MEMS_FRAME_SLOT_NUM];  //用于放置完整的帧的空间的描述结构体数组
};
