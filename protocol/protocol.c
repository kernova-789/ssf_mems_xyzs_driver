#include "protocol.h"
#include "core.h"
#include "modbus_receive.h"
#include "modbus_table.h"

int ssf_mems_protocol_process(struct ssf_mems_xyzs_data *data,
                                     struct ssf_mems_frame_slot *slot) {
  const u8 *buf = slot->data;
  size_t len = slot->data_len;
  u8 function;

  if (!buf)
    return -EINVAL;

  // Modbus RTU 最小帧：
  // 从机地址 + 功能码 + CRC
  if (len < 4)
    return -EINVAL;

  // buf[0] : 从机地址
  // buf[1] : 功能码
  function = buf[1];

  switch (function) {
  case 0x03:
    /* Read Holding Registers */
    return ssf_mems_modbus_read_process(data, slot);

  case 0x06:
    /* Write Single Register */
    return ssf_mems_modbus_write_single_process(data, slot);

  case 0x10:
    /* Write Multiple Registers */
    return ssf_mems_modbus_write_multiple_process(data, slot);

  default:
    dev_err(&data->serdev->dev, "unsupported Modbus function: 0x%02x\n",
            function);
    return -EOPNOTSUPP;
  }
}