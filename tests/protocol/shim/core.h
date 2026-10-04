#pragma once
#include "common.h"
#include "modbus.h"
#include "modbus_receive.h"
#include "modbus_request.h"

#define SSF_MEMS_FRAME_SLOT_NUM 4
#define SSF_MEMS_FRAME_SLOT_FREE 0
#define SSF_MEMS_FRAME_SLOT_USED 1

struct ssf_mems_xyzs_data {
  struct serdev_device *serdev;
  struct kfifo rx_fifo;
  spinlock_t rx_fifo_lock;
  struct work_struct rx_work;
  struct ssf_mems_modbus_request_state modbus_req;
  struct mutex sensor_data_lock;
  struct ssf_mems_sensor_data sensor_data;
  bool sensor_data_valid;
  struct ssf_mems_frame_slot frame[SSF_MEMS_FRAME_SLOT_NUM];
};
