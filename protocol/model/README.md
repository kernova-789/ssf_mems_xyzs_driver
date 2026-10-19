# 共用 Modbus 帧描述

这里只保留标准 RTU 功能码 0x03、0x06、0x10 及异常响应的帧布局。
`modbus_table.c` 定义唯一一份帧表，`modbus_table.h` 声明帧查找接口。

传感器寄存器表、块命令、数据结构、偏移映射和型号参数已移到
[sensor/](../../sensor/README.md)。查询型号寄存器必须显式传入当前设备的
`data->sensor`，不会使用进程级或模块级的默认传感器表。
