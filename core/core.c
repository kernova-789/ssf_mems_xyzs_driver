#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "core.h"
#include "protocol.h"
#include "modbus_receive.h"
#include "modbus_request.h"

static int ssf_mems_xyzs_ops_receive_buf(struct serdev_device *serdev,
                                         const unsigned char *buf,
                                         size_t count) {
  int ret;

  ret = ssf_mems_rx_push(serdev, buf, count);
  if (ret < 0) {
    dev_err(&serdev->dev, "failed to push received data into rx fifo: %d\n",
            ret);
    return count;
  }

  return ret;
}

static void ssf_mems_xyzs_ops_write_wakeup(struct serdev_device *serdev) {}

static const struct serdev_device_ops ssf_mems_xyzs_ops = {
    .receive_buf = ssf_mems_xyzs_ops_receive_buf,
    .write_wakeup = ssf_mems_xyzs_ops_write_wakeup,
};

static int ssf_mems_xyzs_probe(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data;
  int ret;

  data = devm_kzalloc(&serdev->dev, sizeof(*data), GFP_KERNEL);
  if (!data)
    return -ENOMEM;

  data->serdev = serdev;

  ret = ssf_mems_protocol_init(data);
  if (ret)
    return ret;

  ret = ssf_mems_modbus_request_init(data);
  if (ret)
    return ret;

  ret = ssf_mems_modbus_receive_init(data, ssf_mems_protocol_handle_frame, data);
  if (ret) {
    dev_err(&serdev->dev, "failed to initialize Modbus receive state: %d\n", ret);
    goto err_request;
  }

  serdev_device_set_drvdata(serdev, data);
  serdev_device_set_client_ops(serdev, &ssf_mems_xyzs_ops);

  ret = devm_serdev_device_open(&serdev->dev, serdev);
  if (ret)
    goto err_receive;

  serdev_device_set_baudrate(serdev, 9600);
  serdev_device_set_flow_control(serdev, false);

  return 0;

err_receive:
  ssf_mems_modbus_receive_remove(data);
err_request:
  ssf_mems_modbus_request_remove(data);
  return ret;
}

static void ssf_mems_xyzs_remove(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data;
  data = serdev_device_get_drvdata(serdev);
  ssf_mems_modbus_receive_remove(data);
  ssf_mems_modbus_request_remove(data);
}

static const struct of_device_id ssf_mems_of_matchs[] = {
    {
        .compatible = "sange-cbm,ssf-mems-xyzs",
    },
    {}};

MODULE_DEVICE_TABLE(of, ssf_mems_of_matchs);

static struct serdev_device_driver ssf_mems_driver = {
    .driver =
        {
            .name = "ssf_mems_xyzs_driver",
            .of_match_table = ssf_mems_of_matchs,
            .owner = THIS_MODULE,
        },

    .probe = ssf_mems_xyzs_probe,
    .remove = ssf_mems_xyzs_remove,
};

module_serdev_device_driver(ssf_mems_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("kernova-789");
MODULE_DESCRIPTION("SSF MEMS sensor driver");
