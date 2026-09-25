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

#define SSF_RX_BUF_SIZE 5
#define SSF_FRAME_MAX_SIZE 1024

static int ssf_mems_xyzs_ops_receive_buf(struct serdev_device *serdev,
                                         const unsigned char *buf,
                                         size_t count) {
  struct ssf_mems_data *data;
  int ret;

  data = serdev_device_get_drvdata(serdev);

  ret = ssf_mems_rx_push(serdev, buf, count);
  if (ret < 0) {
    dev_err(&serdev->dev, "failed to push received data into rx fifo: %d\n",
            ret);
  }
  return ret;
}

static void ssf_mems_xyzs_ops_write_wakeup(struct serdev_device *serdev) {}

static const struct serdev_device_ops ssf_mems_xyzs_ops = {
    .receive_buf = ssf_mems_xyzs_ops_receive_buf,
    .write_wakeup = ssf_mems_xyzs_ops_write_wakeup,
};

static int ssf_mems_xyzs_probe(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data = NULL;
  int index = 0;
  struct kfifo *rx_fifo = NULL;
  int ret = 0;
  data = devm_kzalloc(&serdev->dev, sizeof(*data), GFP_KERNEL);
  if (!data)
    return -ENOMEM;
  for(index = 0; index < SSF_MEMS_FRAME_SLOT_NUM; index++){
    data->frame[index].data = NULL;
    atomic_set(&data->frame[index].in_use, SSF_MEMS_FRAME_SLOT_FREE);
  }
  data->serdev = serdev;
  data->parse_frame_state = 0;
  spin_lock_init(&data->rx_fifo_lock);
  ret = kfifo_alloc(rx_fifo, 1024, GFP_KERNEL);
  if (ret) {
    dev_err(&serdev->dev, "Failed to allocate RX FIFO\n");
    return ret;
  }
  data->rx_fifo = rx_fifo;

  serdev_device_set_drvdata(serdev, data);

  serdev_device_set_client_ops(serdev, &ssf_mems_xyzs_ops);
  devm_serdev_device_open(&serdev->dev, serdev);
  serdev_device_set_baudrate(serdev, 9600);
  serdev_device_set_flow_control(serdev, false);
  return 0;
}
static void ssf_mems_xyzs_remove(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data = serdev_device_get_drvdata(serdev);
  kfifo_free(data->rx_fifo);
}

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