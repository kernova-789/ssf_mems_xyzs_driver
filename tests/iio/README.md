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
只读统计属性测试分别核对有效采集、停止排空、首包等待和业务模式。
# Polling attributes

The production IIO sysfs handlers are tested for fixed/adaptive mode strings,
all three interval attributes, readback, decimal parsing, invalid input rejection,
and propagation of configuration errors. The effective interval remains read-only.
Range validation and scheduling use the production acquisition tests.

可选采样率和波特率列表由生产型号表生成，测试验证其输出及属性绑定。
发布并发测试同时断言 IIO 推送不会修改协议特征缓存，避免恢复重复缓存写入。
独立数值夹具核对全部 29 个通道的顺序、符号和单位，避免测试仅重复公共字段表。
注册测试覆盖两个设备各阶段失败后的清理，以及重复注册保护。

扫描长度按目标内核 `iio_compute_scan_bytes()` 的规则计算：各字段先按宽度对齐，
整条记录末尾再按最大字段宽度补齐。全部特征不带时间戳为 58 字节、带时间戳为
72 字节；原始五个数据字段不带时间戳为 16 字节、带时间戳为 24 字节。
31 种原始字段子集均检查尾部补齐。

注册测试覆盖特征/原始两设备的分配、buffer 分配和注册失败清理，以及重复注册保护。
独立的 29 通道数值夹具检查公共字段 schema 重构后的通道顺序、符号和单位比例。

XYZ V2.4 另用独立样本检查真实注册流程的 19 个特征通道、温度偏移、位移 scale、
缺失能力的属性隐藏，以及 38/48 字节完整扫描和带温度/位移的部分扫描。
XYZS 的原有 29 通道数值夹具及 100000 次发布/20000 次重配置并发测试继续保留。
