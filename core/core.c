#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/types.h>

struct ssf_mems_xyzs_data{
    struct serdev_device *serdev;

};

static int ssf_mems_xyzs_ops_receive_buf(struct serdev_device *serdev, const unsigned char *buf, size_t len){
    u8 data[] = "message from serdev device";
    int ret = 0;
    ret += serdev_device_write_buf(serdev, buf, len);
    ret += serdev_device_write_buf(serdev, data, sizeof(data));
    return ret;
}
static void ssf_mems_xyzs_ops_write_wakeup(struct serdev_device *serdev){
    
}

static const struct serdev_device_ops ssf_mems_xyzs_ops = {
    .receive_buf = ssf_mems_xyzs_ops_receive_buf,
    .write_wakeup = ssf_mems_xyzs_ops_write_wakeup,
};

static int	ssf_mems_xyzs_probe(struct serdev_device *serdev){
    struct ssf_mems_xyzs_data *data;
    data = devm_kzalloc(&serdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;
    data->serdev = serdev;
    serdev_device_set_drvdata(serdev, data);

    serdev_device_set_client_ops(serdev, &ssf_mems_xyzs_ops);
    devm_serdev_device_open(&serdev->dev, serdev);
    serdev_device_set_baudrate(serdev, 9600);
    serdev_device_set_flow_control(serdev, false);
    return 0;
}
static void ssf_mems_xyzs_remove(struct serdev_device *serdev){

}

static const struct of_device_id ssf_mems_of_matchs[] = {
    { .compatible = "sange-cbm,ssf-mems-xyzs", },
    { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ssf_mems_of_matchs);

static struct serdev_device_driver ssf_mems_driver = {
    .driver = {
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