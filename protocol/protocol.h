#pragma once

struct ssf_mems_frame_slot;
struct ssf_mems_xyzs_data;

/*
 * TODO：未认领帧的处理策略待定，暂不声明和定义此函数。
 * 保留 modbus_receive.c 中的调用，故意让该位置编译报错以提醒实现。
 *
 * int ssf_mems_protocol_process(struct ssf_mems_xyzs_data *data,
 *                               struct ssf_mems_frame_slot *slot);
 */
