#pragma once
#include <linux/serdev.h>
#include <linux/types.h>

enum ssf_mems_frame_type {
    SSF_MEMS_FRAME_MODBUS,
    SSF_MEMS_FRAME_RAW_VIB,
};
struct ssf_mems_frame_desc {
    enum ssf_mems_frame_type type;
    u8 slave_id;
};
enum ssf_mems_modbus_func {
    SSF_MEMS_MODBUS_FUNC_READ = 0x03,
    SSF_MEMS_MODBUS_FUNC_WRITE_MULTI = 0x10,
};

static const struct ssf_mems_frame_desc ssf_mems_frames[] = {
    {
        .type = SSF_MEMS_FRAME_MODBUS,
        .slave_id = 0x01,
    },
    {
        .type = SSF_MEMS_FRAME_RAW_VIB,
        .slave_id = 0x15,
    },
};

int ssf_mems_rx_push(struct serdev_device *serdev, const unsigned char *buf,
                     size_t count);