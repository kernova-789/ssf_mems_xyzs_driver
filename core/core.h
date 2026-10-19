#pragma once

#include <linux/serdev.h>
#include <linux/types.h>

#include "sensor.h"
#include "protocol.h"
#include "modbus/modbus_receive.h"
#include "modbus/modbus_request.h"
#include "ssf_mems_acquisition.h"
#include "raw/raw_stream.h"
#include "firmware/ota.h"

struct iio_dev;

/* core 只组合设备和各模块状态，模块内部的变量由各自负责初始化和管理。 */
struct ssf_mems_data {
  const struct ssf_mems_sensor *sensor; /* probe 按 compatible 选择，只读。 */
  const struct ssf_mems_sensor_ops *sensor_ops; /* probe 绑定的型号采集接口。 */
  struct serdev_device *serdev; /* 与板载 UART 绑定的串口设备。 */
  u8 slave_id; // 当前设备实例对应的 Modbus 从机地址
  struct ssf_mems_modbus_receive_state modbus_rx; /* 字节 FIFO 和接收拼帧状态。 */
  struct ssf_mems_modbus_request_state modbus_req; /* 当前请求和响应等待状态。 */
  struct ssf_mems_protocol_state protocol; /* 总线模式、串口配置和特征缓存。 */
  struct ssf_mems_acquisition_state acquisition; /* 采集线程、调度和恢复状态。 */
  struct iio_dev *indio_dev; // IIO 对象的创建/释放由 iio 目录代码负责
  struct iio_dev *raw_indio_dev; /* 独立的 XYZ 原始采样 IIO 设备。 */
  struct ssf_mems_raw_state raw; /* 私有定长数据的拼包和丢包统计状态。 */
  struct ssf_mems_ota_state ota; /* bin 升级的应答、进度和生命周期。 */
};
