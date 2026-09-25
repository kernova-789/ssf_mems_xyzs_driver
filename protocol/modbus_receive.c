#include "modbus_receive.h"
#include "core.h"
#include <linux/kfifo.h>
#include <linux/serdev.h>
#include <linux/slab.h>

enum ssf_mems_rx_state {
  SSF_MEMS_RX_IDLE,
  SSF_MEMS_RX_ADDR,
  SSF_MEMS_RX_FUNC,
  SSF_MEMS_RX_DATA,
  SSF_MEMS_RX_CRC,
  SSF_MEMS_RX_DONE,
  SSF_MEMS_RX_ERROR,
};
struct ssf_mems_rx_ctx {
  enum ssf_mems_rx_state state;

  u8 function;
  size_t frame_len;
  size_t received_len;
};

int ssf_mems_rx_push(struct serdev_device *serdev, const unsigned char *buf,
                     size_t count) {

  struct ssf_mems_xyzs_data *data = serdev_device_get_drvdata(serdev);
  unsigned int ret;

  if (!count)
    return 0;

  ret = kfifo_in_spinlocked(data->rx_fifo, buf, count, &data->rx_fifo_lock);

  if (ret != count) {
    dev_err(&data->serdev->dev,
            "rx fifo overflow: received %zu bytes, stored %u bytes\n", count,
            ret);
    return -ENOSPC;
  }

  return ret;
}

enum parse_frame_state {
  parse_frame_slave_id,
  parse_frame_function_code,
  parse_frame_len,
  parse_frame_max,
};
static int ssf_mems_modbus_parse_frame(struct serdev_device *serdev) {
  struct ssf_mems_xyzs_data *data = serdev_device_get_drvdata(serdev);
  unsigned int ret;
  u8 recv_data = 0;
  u8 recv_buf[parse_frame_max] = {0};
  enum parse_frame_state state = data->parse_frame_state;
  int index = 0;
  while (1) {
    if (kfifo_is_empty(data->rx_fifo)) {
      data->parse_frame_state = state;
      return 0;
    }
    ret =
        kfifo_out_spinlocked(data->rx_fifo, &recv_data, 1, &data->rx_fifo_lock);
    switch (state) {
    case parse_frame_slave_id:
      switch (recv_data) {
      case ssf_mems_frames[SSF_MEMS_FRAME_MODBUS].slave_id:
        recv_buf[parse_frame_slave_id] = recv_data;
        state = parse_frame_function_code;
        break;
      case ssf_mems_frames[SSF_MEMS_FRAME_RAW_VIB].slave_id:
        // 收到私有帧协议的数据，暂不做处理
        break;
      default:
        // 其他情况不做处理
        break;
      }
      break;
    case parse_frame_function_code:
      if ((recv_buf[parse_frame_slave_id] ==
           ssf_mems_frames[SSF_MEMS_FRAME_MODBUS].slave_id) &&
          ((recv_data == SSF_MEMS_MODBUS_FUNC_READ) ||
           (recv_data == SSF_MEMS_MODBUS_FUNC_WRITE_MULTI))) {
        recv_buf[parse_frame_function_code] = recv_data;
        state = parse_frame_len;
      } else {
        // 暂时不处理私有帧协议的数据，这里直接重置状态
        state = parse_frame_slave_id;
      }
      break;
    case parse_frame_len:
    recv_buf[parse_frame_len] = recv_data;
      // 寻找没有被使用的struct ssf_mems_frame_slot，然后创建一个帧数组
      for (index = 0; index < SSF_MEMS_FRAME_SLOT_NUM; index++) {
        if (atomic_cmpxchg(&data->frame[index].in_use, SSF_MEMS_FRAME_SLOT_FREE,
                           SSF_MEMS_FRAME_SLOT_USED) == 0) {
          data->frame[index].data = kzalloc(1+1+1+recv_data+2, GFP_KERNEL);
          data->frame[index].data[0] = recv_buf[parse_frame_slave_id];
          data->frame[index].data[1] = recv_buf[parse_frame_function_code];                  
          data->frame[index].data[2] = recv_buf[parse_frame_len];                  
          data->frame[index].data_len = 1+1+1+recv_data+2;
          data->frame[index].data_pos = 3;
          if (data->frame[index].data == NULL) {
            atomic_set(&data->frame[index].in_use, SSF_MEMS_FRAME_SLOT_FREE);
            dev_err(&serdev->dev, "frame slots failed to allocate memory\n");
          }
        } else {
          if (index == SSF_MEMS_FRAME_SLOT_NUM - 1) { // 没有空间可用了
            dev_err(&serdev->dev,
                    "frame slots are busy, dropping received frame\n");
          }
        }
      }
      state = parse_frame_slave_id;
      break;
    default:
      break;
    }
    // 往已经被占用的FRAME_SLOT中填数据
    for (index = 0; index < SSF_MEMS_FRAME_SLOT_NUM; index++) {
      if ((atomic_read(&data->frame[index].in_use) == SSF_MEMS_FRAME_SLOT_USED) && (data->frame[index].data_pos < data->frame[index].data_len)) {
        data->frame[index].data[data->frame[index].data_pos] = recv_data;
        data->frame[index].data_pos++;
        if(data->frame[index].data_pos == data->frame[index].data_len){
          //把指针送给CRC校验函数去校验
        }
      } else {
        continue;
      }
    }
  }
}
