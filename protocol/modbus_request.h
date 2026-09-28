#pragma once

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

struct serdev_device;
struct ssf_mems_xyzs_data;
struct ssf_mems_frame_slot;

struct ssf_mems_modbus_request_state {
  struct mutex lock;        //保护这个请求状态结构体，防止多个线程同时修改
  wait_queue_head_t waitq;  //让发送请求的线程睡眠等待响应，并在响应到达后把它唤醒
  bool pending;             //表示当前是否有一个请求正在等待响应，true → 正在等待
  bool shutting_down;       //表示驱动是否正在关闭/卸载，true → 驱动正在退出
  u8 slave_id;              //保存当前请求对应的从机地址
  u8 function;              //保存当前请求对应的从机地址
  u16 display_reg;          //保存用户看到的寄存器号
  u16 protocol_addr;        //保存用户看到的寄存器号
  u16 reg_count;            //保存真正发送给 Modbus 设备的寄存器地址
  u16 *values;              //指向保存响应数据的内存
  size_t values_count;      //表示 values 数组最多能存多少个寄存器
  int status;               //保存这次请求最终的结果，0 → 成功
};

#define SSF_MEMS_MODBUS_DEFAULT_TIMEOUT_MS 1000U
#define SSF_MEMS_MODBUS_DEFAULT_SLAVE_ID 0x01U
#define SSF_MEMS_MODBUS_WRITE_TIMEOUT_MS 100U

int ssf_mems_modbus_request_init(struct ssf_mems_xyzs_data *data);
void ssf_mems_modbus_request_remove(struct ssf_mems_xyzs_data *data);
void ssf_mems_modbus_queue_parse(struct serdev_device *serdev);
int ssf_mems_modbus_read(struct serdev_device *serdev, u16 display_reg,
                         u16 reg_count, u16 *values, unsigned int timeout_ms);
int ssf_mems_modbus_read_reg(struct serdev_device *serdev, u16 display_reg,
                             u16 *value, unsigned int timeout_ms);
int ssf_mems_modbus_claim_frame(struct ssf_mems_xyzs_data *data,
                                struct ssf_mems_frame_slot *slot);
