#include "modbus.h"
#include "modbus_table.h"
#include <linux/serdev.h>
#include <linux/types.h>

static u16 ssf_modbus_tx_len(const struct ssf_cmd_desc *cmd) {
  switch (cmd->function) {
  case 0x03:
    return 8;

  case 0x06:
    return 8;

  case 0x10:
    return 9 + 2 * cmd->reg_count;

  default:
    return 0;
  }
}
static u16 ssf_modbus_rx_len(const struct ssf_cmd_desc *cmd) {
  switch (cmd->function) {
  case 0x03:
    return 5 + 2 * cmd->reg_count;

  case 0x06:
    return 8;

  case 0x10:
    return 8;

  default:
    return 0;
  }
}


