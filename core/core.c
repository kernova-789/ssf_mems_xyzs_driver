#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/types.h>

#include "core.h"

#define SSF_RX_BUF_SIZE 5
#define SSF_FRAME_MAX_SIZE 1024

struct ssf_mems_xyzs_data {
  struct serdev_device *serdev;
  struct ssf_rx_frame rx_frames[SSF_RX_BUF_SIZE];
  u8 slave_addr;    //从机地址
};

static int ssf_mems_xyzs_ops_receive_buf(struct serdev_device *serdev,
                                         const unsigned char *buf, size_t len) {
  u8 data[] = "message from serdev device";
  int ret = 0;
  ret += serdev_device_write(serdev, buf, len, 1000);
  if(ret < 0) {
    dev_err(&serdev->dev, "Failed to write data to serdev device\n");
    return ret;
  }
  ret += serdev_device_write(serdev, data, sizeof(data), 1000);

  return ret;
}
static void ssf_mems_xyzs_ops_write_wakeup(struct serdev_device *serdev) {}

static const struct serdev_device_ops ssf_mems_xyzs_ops = {
    .receive_buf = ssf_mems_xyzs_ops_receive_buf,
    .write_wakeup = ssf_mems_xyzs_ops_write_wakeup,
};

static int ssf_mems_xyzs_probe(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data;
  data = devm_kzalloc(&serdev->dev, sizeof(*data), GFP_KERNEL);
  if (!data)
    return -ENOMEM;
  data->serdev = serdev;
  data->slave_addr = 0x01; // Set the slave address

  serdev_device_set_drvdata(serdev, data);

  serdev_device_set_client_ops(serdev, &ssf_mems_xyzs_ops);
  devm_serdev_device_open(&serdev->dev, serdev);
  serdev_device_set_baudrate(serdev, 9600);
  serdev_device_set_flow_control(serdev, false);
  return 0;
}
static void ssf_mems_xyzs_remove(struct serdev_device *serdev) {}

static const struct of_device_id ssf_mems_of_matchs[] = {
    {
        .compatible = "sange-cbm,ssf-mems-xyzs",
    },
    {/* sentinel */}};

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