#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "core.h"
#include "ssf_mems_iio.h"
#include "protocol.h"
#include "modbus/modbus_receive.h"
#include "modbus/modbus_request.h"
#include "ssf_mems_acquisition.h"

/* 把串口收到的字节放入接收队列。
 * 返回：已接收的字节数，失败时为 0。
 * 未入队的剩余字节由 serdev 重送。
 */
static int ssf_mems_serdev_ops_receive_buf(struct serdev_device *serdev,
                              const unsigned char *buf, size_t count) {
  ssize_t ret;

  ret = ssf_mems_rx_push(serdev, buf, count);
  if (ret < 0) {
    dev_err(&serdev->dev, "failed to push received data into rx fifo: %zd\n",
            ret);
    return 0;
  }

  return (int)ret;
}

/* 唤醒等待发送串口数据的线程。
 * 返回：无。
 */
static void ssf_mems_serdev_ops_write_wakeup(struct serdev_device *serdev) {
  serdev_device_write_wakeup(serdev);
}

/* serdev 回调表：接收只入队，发送唤醒交给 serdev 通用实现。 */
static const struct serdev_device_ops ssf_mems_serdev_ops = {
    .receive_buf = ssf_mems_serdev_ops_receive_buf,
    .write_wakeup = ssf_mems_serdev_ops_write_wakeup,
};

/* 初始化驱动并启动后台采集。
 * 返回：0 成功，负数为初始化错误。
 * 传感器通信由后台线程执行。
 */
static int ssf_mems_probe(struct serdev_device *serdev) {
  struct ssf_mems_data *data;
  int ret;

  data = devm_kzalloc(&serdev->dev, sizeof(*data), GFP_KERNEL);
  if (data == NULL)
    return -ENOMEM;

  data->serdev = serdev;
  data->sensor = device_get_match_data(&serdev->dev);
  if (data->sensor == NULL)
    return -ENODEV;
  data->sensor_ops = data->sensor->ops;

  /* probe 只初始化主机状态；真正匹配和 Modbus 读写由后台线程执行。 */
  ret = ssf_mems_protocol_init(data);
  if (ret != 0) {
    dev_err(&serdev->dev, "failed to initialize protocol configuration: %d\n",
            ret);
    return ret;
  }

  ret = ssf_mems_modbus_request_init(data);
  if (ret != 0)
    return ret;

  ret = ssf_mems_modbus_receive_init(data, ssf_mems_protocol_handle_frame, data);
  if (ret != 0) {
    dev_err(&serdev->dev, "failed to initialize Modbus receive state: %d\n", ret);
    goto err_request;
  }

  serdev_device_set_drvdata(serdev, data);
  serdev_device_set_client_ops(serdev, &ssf_mems_serdev_ops);

  ret = devm_serdev_device_open(&serdev->dev, serdev);
  if (ret != 0)
    goto err_receive;

  ret = ssf_mems_protocol_configure_serial(data);
  if (ret != 0) {
    dev_err(&serdev->dev, "failed to configure serial parameters: %d\n", ret);
    goto err_receive;
  }

  ret = ssf_mems_acquisition_init(data);
  if (ret != 0)
    goto err_receive;

  ret = ssf_mems_iio_register(data);
  if (ret != 0) {
    dev_err(&serdev->dev, "failed to register IIO device: %d\n", ret);
    goto err_receive;
  }

  data->raw.sample_handler = ssf_mems_iio_publish_raw;
  ret = ssf_mems_acquisition_start(data);
  if (ret != 0) {
    dev_err(&serdev->dev, "failed to start sensor acquisition: %d\n", ret);
    goto err_iio;
  }

  return 0;

err_iio:
  /* 与成功初始化的顺序相反回滚；先停止所有使用 IIO/接收状态的生产者。 */
  ssf_mems_ota_shutdown(data);
  ssf_mems_modbus_request_remove(data);
  ssf_mems_acquisition_stop(data);
  ssf_mems_iio_unregister(data);
err_receive:
  ssf_mems_modbus_receive_remove(data);
err_request:
  ssf_mems_modbus_request_remove(data);
  return ret;
}

/* 停止驱动并释放设备资源。
 * 返回：无。
 * 先停止数据生产者，再释放 IIO 和接收资源。
 */
static void ssf_mems_remove(struct serdev_device *serdev) {
  struct ssf_mems_data *data;
  data = serdev_device_get_drvdata(serdev);
  ssf_mems_ota_shutdown(data);
  /* 先拒绝新请求并唤醒在途请求，再等待生产者退出，最后释放其使用的资源。 */
  ssf_mems_modbus_request_remove(data);
  ssf_mems_acquisition_stop(data);
  ssf_mems_iio_unregister(data);
  ssf_mems_modbus_receive_remove(data);
}

/* 按设备树选择型号；最后一个空项是 OF 匹配表的结束标记。 */
static const struct of_device_id ssf_mems_of_matchs[] = {
    {
        .compatible = "sange-cbm,ssf-mems-xyzs",
        .data = &ssf_mems_xyzs,
    },
    { .compatible = "sange-cbm,ssf-mems-xyz", .data = &ssf_mems_xyz },
    {}};

MODULE_DEVICE_TABLE(of, ssf_mems_of_matchs);

/* 将设备树匹配、绑定/解绑入口及模块所有者交给 serdev 总线。 */
static struct serdev_device_driver ssf_mems_driver = {
    .driver =
        {
            .name = "ssf_mems_xyzs_driver",
            .of_match_table = ssf_mems_of_matchs,
            .owner = THIS_MODULE,
        },

    .probe = ssf_mems_probe,
    .remove = ssf_mems_remove,
};

module_serdev_device_driver(ssf_mems_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("kernova-789");
MODULE_DESCRIPTION("SSF MEMS sensor driver");
