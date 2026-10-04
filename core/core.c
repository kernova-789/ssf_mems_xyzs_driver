#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "core.h"
#include "modbus_receive.h"
#include "modbus_request.h"

#define SSF_RX_FIFO_SIZE 1024

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
  int index;
  int ret;

  data = devm_kzalloc(&serdev->dev, sizeof(*data), GFP_KERNEL);
  if (!data)
    return -ENOMEM;

  for (index = 0; index < SSF_MEMS_FRAME_SLOT_NUM; index++) {
    data->frame[index].data = NULL;
    data->frame[index].data_len = 0;
    data->frame[index].data_pos = 0;
    data->frame[index].frame_len = 0;
    data->frame[index].state = SSF_MEMS_RX_IDLE;
    data->frame[index].slave_id = 0;
    data->frame[index].function = 0;

    atomic_set(&data->frame[index].in_use, SSF_MEMS_FRAME_SLOT_FREE);
  }

  data->serdev = serdev;

  spin_lock_init(&data->rx_fifo_lock);
  mutex_init(&data->sensor_data_lock);
  data->sensor_data_valid = false;

  ret = kfifo_alloc(&data->rx_fifo, SSF_RX_FIFO_SIZE, GFP_KERNEL);
  if (ret) {
    dev_err(&serdev->dev, "failed to allocate RX FIFO\n");
    return ret;
  }

  serdev_device_set_drvdata(serdev, data);

  ret = ssf_mems_modbus_request_init(data);
  if (ret) {
    kfifo_free(&data->rx_fifo);
    return ret;
  }

  serdev_device_set_client_ops(serdev, &ssf_mems_xyzs_ops);

  ret = devm_serdev_device_open(&serdev->dev, serdev);
  if (ret) {
    ssf_mems_modbus_request_remove(data);
    kfifo_free(&data->rx_fifo);
    return ret;
  }

  serdev_device_set_baudrate(serdev, 9600);
  serdev_device_set_flow_control(serdev, false);

  return 0;
}

static void ssf_mems_xyzs_remove(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data;
  data = serdev_device_get_drvdata(serdev);
  ssf_mems_modbus_request_remove(data);
  kfifo_free(&data->rx_fifo);
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
