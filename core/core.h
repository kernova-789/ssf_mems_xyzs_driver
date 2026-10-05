#pragma once

#include <linux/serdev.h>

#include "protocol.h"
#include "modbus_receive.h"
#include "modbus_request.h"

/* core 只组合设备和各模块状态，模块内部的变量由各自负责初始化和管理。 */
struct ssf_mems_xyzs_data {
  struct serdev_device *serdev;
  struct ssf_mems_modbus_receive_state modbus_rx;
  struct ssf_mems_modbus_request_state modbus_req;
  struct ssf_mems_protocol_state protocol;
};
