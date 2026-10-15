# IIO publication regression test

Run `python3 tests/iio/run_tests.py` from the repository root.

The test compiles the complete production IIO source against a minimal IIO shim
and pthread mutexes with AddressSanitizer/UndefinedBehaviorSanitizer. One thread
publishes 100000 snapshots while another reconfigures the buffer 20000 times,
disabling it and freeing its scan mask before installing the next configuration.
Every push requires the IIO mutex to be held and checks the selected channel
layout, signed temperature, 8-bit flags, padding and optional timestamp.

The protocol cache interface is a mutex-protected test substitute. Configuration
sysfs callbacks also check rate-index mapping, feature-mask routing, decimal/hex
output, input parsing and error propagation using a mock acquisition interface.
Baud-rate and status callbacks are not exercised. This validates producer
synchronization, packing and attribute dispatch, not actual kernel IIO kfifo
implementation or hardware behavior.

Runtime acquisition-control callbacks are exercised against an acquisition
substitute: all three mode strings, optional newline, invalid/trailing input,
decimal duration routing, range errors and shutdown/unsupported errors. Actual
scheduling is covered by `tests/acquisition`, not by this substitute.
# 原始 IIO buffer 测试

增加原始 XYZ、包号和包内序号全部 31 种非空字段子集测试，各自覆盖带/不带
时间戳的扫描对齐；同时验证原始 scale、buffer 未启用及错误参数处理。
只读统计属性测试分别核对有效采集、停止排空、首包等待、停止次数和业务模式。
# Polling attributes

The production IIO sysfs handlers are tested for fixed/adaptive mode strings,
all three interval attributes, readback, decimal parsing, invalid input rejection,
and propagation of configuration errors. The effective interval remains read-only.
Range validation and scheduling use the production acquisition tests.

可选采样率和波特率列表由生产型号表生成，测试验证其输出及属性绑定。
发布并发测试同时断言 IIO 推送不会修改协议特征缓存，避免恢复重复缓存写入。
