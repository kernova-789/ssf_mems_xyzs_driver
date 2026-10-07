#pragma once

#include <linux/serdev.h>
#include <linux/types.h>

#include "protocol.h"
#include "modbus_receive.h"
#include "modbus_request.h"

struct iio_dev;

/* core 只组合设备和各模块状态，模块内部的变量由各自负责初始化和管理。 */
struct ssf_mems_xyzs_data {
  struct serdev_device *serdev;
  u8 slave_id; // 当前设备实例对应的 Modbus 从机地址
  struct ssf_mems_modbus_receive_state modbus_rx;
  struct ssf_mems_modbus_request_state modbus_req;
  struct ssf_mems_protocol_state protocol;
  struct iio_dev *indio_dev; // IIO 对象的创建/释放由 iio 目录代码负责
};
