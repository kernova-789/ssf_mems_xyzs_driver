# 开发板回归测试

现行正式驱动已接入 1024 点定长采集，原始与特征共用 128000；集成结果见
[正式驱动验证报告](TEST_REPORT_2026-10-12_PRODUCTION_FINITE.md)。
驱动独立通信、断电恢复和 IIO 边界验证见
[完整功能回归报告](TEST_REPORT_2026-10-12_DRIVER_FUNCTIONAL.md)，其中保留了一次尚未定位的配置读取超时。
正式卸载回归使用 `board_finite_unload.py`。

## 驱动独立通信的功能回归

`board_driver_functional.py` 仅访问 IIO sysfs、IIO 字符设备和内核统计，
不会打开 tty、组装 Modbus 帧或加载独立通信模块。传感器的全部帧都由正式驱动发送。
运行前确认没有其他 IIO 消费者，并为每轮创建独立输出目录。

```sh
sudo python3 board_driver_functional.py /tmp/本次输出目录 api
sudo python3 board_driver_functional.py /tmp/本次输出目录 invalid-buffer
sudo python3 board_driver_functional.py /tmp/本次输出目录 layouts
sudo python3 board_driver_functional.py /tmp/本次输出目录 overflow
sudo python3 board_driver_functional.py /tmp/本次输出目录 feature-overflow
sudo python3 board_driver_functional.py /tmp/本次输出目录 lifecycle --module /绝对路径/ssf_mems.ko
sudo python3 board_driver_functional.py /tmp/本次输出目录 early-remove --module /绝对路径/ssf_mems.ko
```

- `api` 检查配置读回、非法写、采样率/特征开关同值写、四线程读取和单位比例。
  不实际切换采样率，也不执行波特率同值写。当前驱动同值波特率先读回确认，
  一致时跳过保存和重启；该优化已做离线回归，原实机报告不覆盖它。
- `invalid-buffer` 检查空掩码/仅时间戳被 EINVAL 拒绝、运行中改掩码被 EBUSY 拒绝。
- `layouts` 检查单通道、混合宽度、无时间戳和 8 位标志的实际扫描对齐。
- `overflow` 故意使用仅两条记录的小原始 buffer，不读取直到溢出，再确认特征恢复。
  这一轮丢样是故障注入的预期结果，不能混入正常长测的“零丢样”统计。
- `feature-overflow` 用两条记录的特征 buffer 暂停消费，检查预期满队列告警后缓存仍更新。
- `lifecycle` 检查打开的 IIO 句柄阻止 rmmod；随后解绑 serdev，检查阻塞读者以 ENODEV
  唤醒，关闭句柄后正常卸载、重载并自动连接，不强制卸载模块。
- `early-remove` 立即加载/卸载，用板端 **UART 4** 的 TX 计数验证后台首发前没有通信。
  后两项会中断采集，均应只使用已验证的正式模块，并在最后恢复加载。

新格式 `board_capture.py` 在状态中记录累计数据量，`check_capture.py` 检查两路持续推进。
二进制检查器也检查时间戳覆盖和最长空档，可用于没有累计字段的旧归档。
默认最大无进展时间为 30 秒，适用于默认 5 秒/5 秒交替；使用更长设备树窗口时，
需依据计划显式设置两个检查器的 `--progress-limit`，不能事后为掩盖故障扩大阈值。

所有用例退出后关闭其使用的 buffer，但不恢复扫描掩码和 buffer 长度。

高波特率定长/特征交替结果见
[2026-10-12 分档报告](TEST_REPORT_2026-10-12_HIGH_FINITE.md)：128000 通过约半小时测试，
更高枚举档位均观测到通信错误；保留 128000 / 5333.4 Hz，不恢复默认。
该分档报告是独立实验；正式驱动的集成验证另见上面的报告，原始记录仍有批间空窗。

全新传感器的同条件及最低采样率对照结果见
[2026-10-12 新传感器报告](TEST_REPORT_2026-10-12_NEW_SENSOR.md)：两轮均复现停采，
测试后已逐项核对恢复初始默认配置，证据归档附在报告内。

这些脚本需要在接好传感器的开发板上以 root 运行。先用目标板对应内核编译
并加载当前驱动；不要使用正在由其他程序消费的 IIO buffer。
采集记录脚本不修改设备树或重启设备。加载正式驱动时若发现不同的工作波特率，
驱动会保存/重启以应用 128000；`board_finite_baud_reload.py` 也会执行两次实际调速，
不应作为循环长测脚本使用。正常定长批次和卸载不保存或恢复默认参数。

## 双 IIO 数据记录

```sh
test_dir=$(mktemp -d /tmp/ssf-mems-capture.XXXXXX)
sudo python3 board_capture.py "$test_dir" 300
```

脚本按设备名字发现两个 IIO 设备，核对全通道布局（原始 24 字节、特征
72 字节），启用 buffer 并记录二进制、序号完整性和分阶段统计。
退出时关闭本次启用的 buffer。扫描通道掩码和 buffer 长度会被测试调整，
不会恢复原值。输出目录须事先存在，建议每轮使用新目录。

这是记录程序，不会仅因“没有收到数据”而返回失败。检查
`capture_summary.json`：两类记录都应非零；有效段内 `sequence_breaks`、
`bad_indices`、`timestamp_regressions` 应为 0；各时点 `raw_buffer_drops`
应为 0。有效段 CRC/序号异常与停止排空异常应分开分析。
`raw_sessions` 包括开始/结束时可能截断的段，不能等同于完整的采集轮数。
单项诊断属性并非原子快照，不能仅凭边界时的一行状态判断驱动故障。

记录完成还应检查日志、普通通信是否恢复和传感器是否在线；某次正确数据
记录不代表整个长测通过。计量精度与传感器内部过采样完整性不在测试范围内。

可用离线检查器避免把提前终止或长期停在恢复状态的记录误判为通过：

```sh
python3 check_capture.py "$test_dir/capture_summary.json" --seconds 300
python3 analyze_finite_iio.py "$test_dir" --require-whole-batches
python3 -m unittest discover -s tests/hardware -p 'test_*.py'
```

`--seconds` 必须填写原计划时长。两类数据非零、有效段完整性及监测到的
CRC/丢样统计均正常，且没有连续至少 15 秒的恢复未确认状态时才返回 0。
15 秒是该回归测试的判定阈值，不是驱动提供的恢复时限；可用
`--recovery-limit` 显式调整。此检查不能替代内核日志、停止排空分析和
采集完成后的普通通信确认；没有监测到异常也不能证明硬件长期可靠。

定长模式下，记录脚本到时后会尽量在当前批次的 15/63 边界结束，最多再等 30 秒。
若中途启用 buffer，第一批仍可能从中间开始；严格二进制检查要求从 0/0 开始，
此时应重新从驱动加载前启动记录，或者单独分析开始处的不完整批次。

## 卸载时序

将 `board_finite_unload.py` 和待测模块 `ssf_mems.ko` 放入同一新目录，
确认当前加载的就是该模块，并确认普通通信目标/传感器均为 128000：

```sh
sudo python3 board_finite_unload.py /tmp/本次测试目录
```

脚本分别等待“启动已发出但还没有首包”和“已收到原始包”，执行卸载、
确认两个 IIO 设备移除、重载并读回 128000。它检查卸载收完在途定长批次，
重载可恢复普通通信，输出 JSON 和各阶段日志。
此测试会暂时中断采集；不能与 buffer 记录程序或其他 IIO 消费者同时运行。
异常时保留现场和日志，不自动保存 Flash，不自动替用户断电。

硬件异常的根因需要结合供电、接线和串口抓包，不能由上述脚本单独确定。
