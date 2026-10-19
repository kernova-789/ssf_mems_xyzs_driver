/* SPDX-License-Identifier: GPL-2.0-only */
#pragma once
#include <linux/kernel.h>
#include "modbus/modbus_types.h"
extern const struct ssf_modbus_frame_desc ssf_frame_table[];
extern const size_t ssf_frame_table_size;
#define SSF_FRAME_TABLE_SIZE ssf_frame_table_size
const struct ssf_modbus_frame_desc *ssf_mems_modbus_find_frame(u8 function);
const struct ssf_modbus_frame_desc *ssf_mems_modbus_find_rx_frame(u8 function);
