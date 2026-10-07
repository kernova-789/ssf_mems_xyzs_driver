# 协议模块与表驱动配置

## 文件职责

- `modbus.c/.h`：无设备状态的帧长度、CRC、请求编码和响应解码，不调用请求、接收或传感器业务。
- `modbus_request.c/.h`：查表校验、发送、请求状态、等待/超时，以及按实际请求上下文认领响应。
- `modbus_receive.c/.h`：FIFO、候选帧和拼帧工作项；只通过初始化注册的回调交出完整帧，不直接调用请求或业务模块。
- `protocol.c/.h`：完整帧分发、特征分段读取、寄存器字段映射、特征缓存和固定工作参数操作。
- `modbus_types.h`：公共线格式常量、帧/寄存器/块描述和实际请求描述类型，不包含上层模块接口。
- `sensor_data.h`：可复制的传感器结果类型，不含锁或运行状态。
- `modbus_table.h`：实际帧格式、寄存器属性和块命令的唯一配置入口，三个数据表仍保留在这里。

帧数据的处理顺序是：串口字节 → 接收工作项拼帧与 CRC 校验 → 已注册的 `ssf_mems_protocol_handle_frame()` → 请求层认领 → 由 `modbus.c` 按请求描述解码。认领成功后唤醒等待者；未认领帧直接丢弃，不额外解析、不更新请求结果或特征缓存，也不根据长度猜测寄存器起始地址。接收层在回调结束后统一释放帧缓冲区。

接收回调同步执行，帧缓冲区仅在回调期间有效，不能保存其指针。上下文必须存活到接收清理结束；回调不能同步等待需要同一接收工作项处理的新响应。FIFO 拼帧函数已经私有化，外部不能绕过工作项的串行保障。

## 模块状态和初始化

`ssf_mems_xyzs_data` 仅组合设备指针和三个模块状态，`probe()` 依次调用各模块初始化函数：

- `protocol` / `ssf_mems_protocol_init()`：特征缓存、互斥量及缓存有效标记。
- `modbus_req` / `ssf_mems_modbus_request_init()`：当前请求、请求互斥量和等待队列。
- `modbus_rx` / `ssf_mems_modbus_receive_init(data, handler, context)`：接收 FIFO、自旋锁、解析工作项、候选帧及完整帧回调。

锁都封装在所属模块的状态内。初始化只用于尚未投入使用的状态，不能直接对运行中的状态重复初始化。接收清理负责停止工作项并释放 FIFO/候选帧，请求清理负责唤醒等待者；探测失败或卸载时按逆序清理。传感器业务状态和纯帧编解码没有额外动态资源需要释放。

## 接口迁移

业务接口已从 `ssf_mems_modbus_*` 改为 `ssf_mems_protocol_*`，声明统一在 `protocol.h`：

- `ssf_mems_protocol_decode_features()`
- `ssf_mems_protocol_read_features()`
- `ssf_mems_protocol_get_features()`
- `ssf_mems_protocol_write_work_parameters()`
- `ssf_mems_protocol_get_baudrate()`
- `ssf_mems_protocol_set_baudrate()`

通用 `ssf_mems_modbus_read()`、`ssf_mems_modbus_write_reg()` 和 `ssf_mems_modbus_write_regs()` 仍在请求模块。新增 `ssf_mems_modbus_write_block()` 保留块表的功能码、方向、响应格式和固定范围校验，供业务层执行块命令。请求认领接口现在接收 `data, buf, len`，不再依赖接收模块的 `frame_slot`。`ssf_mems_modbus_plan_request()` 也归请求模块，不再属于帧编解码模块。

波特率读取先以当前 serdev 速率读 40102。只有请求超时（即没有可匹配的正确应答帧）才启动枚举扫描；每个不同实际速率只发送一次读请求，避免 `DEFAULT` 和显式 `9600` 重复。切换前清理旧速率留下的 FIFO/候选帧；成功时保留匹配速率，全部超时或扫描中发生其他错误时恢复入口速率。读特征、写工作参数和波特率操作由业务层 `io_lock` 串行化。

## 不再读取某个特征

从 `ssf_reg_table` 删除对应寄存器的整个初始化项即可，不需要修改业务代码，也不要重编号剩余寄存器的 `display_reg` 或 `protocol_addr`。

例如删除 40002 后，`ssf_mems_protocol_read_features()` 自动发送两次读取：40001（1 个寄存器）和 40003～40029（27 个寄存器）。不用修改块表中的起点和 `reg_count = 29`，这里仍表示原始地址范围，而非剩余表项数量。响应根据实际请求的起始地址映射，后续字段不会前移；删除项对应的缓存字段每轮成功读取后为零。

连续的保留地址会合并读取；表项顺序不影响读取顺序。全部特征项删除时不发送读取请求，缓存更新为全零。分段读取任何一步失败时，不覆盖上次成功的缓存。

通用连续读写不会跳过调用者指定范围内的缺失地址：缺失返回 `-ENOENT`，权限不足返回 `-EACCES`，均不发送请求。固定十参数写入仍要求块内所有寄存器存在且可写，不能通过删表项改变这个固定命令。

## 验证与待处理事项

运行 `python3 tests/protocol/run_tests.py` 可检查删表、分段读取、字段映射、组帧和回调分发。测试分别编译/链接各生产模块，单独编译公共头文件，并检查模块的符号依赖；模拟串口不能替代实际硬件验证。

未认领帧直接丢弃，原有未定义处理入口及编译报错提醒已经移除。私有原始数据格式尚未实现，通用读取会明确返回 `-EOPNOTSUPP`。
