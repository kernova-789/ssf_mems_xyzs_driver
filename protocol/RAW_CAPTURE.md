# 定长原始 XYZ 与特征交替采集

驱动先异步匹配串口并读取特征，再交替执行原始批采集窗口和特征轮询窗口。
正式工作波特率默认为 **128000**，两种数据使用同一波特率；FFT 不在驱动中计算。
设备树 `current-speed` 仅作为初次匹配提示；缺失或无效时以手册默认 9600 开始匹配。
已经在 128000 通信时不重复写保存命令；其他速率匹配成功后，才执行
40102=8、40110=1、静默等待 8 秒、新速率读回验证。probe 本身不发 Modbus。
手动写 `sensor_baudrate` 仍可显式改变本次绑定的目标，下次绑定重新使用 128000。

## 采集窗口与数据边界

可选设备树属性（缺失、读失败或非法时打印日志并采用 5000 ms）：

```dts
sange-cbm,raw-duration-ms = <5000>;
sange-cbm,feature-duration-ms = <5000>;
```

范围为 100～60000 ms，原始窗口为 0 时仅轮询特征。
原始窗口内重复完整的 **1024 组同步 XYZ** 批次，至少完成一批；
窗口从第一批触发前计时，包含等待固件、采集、上传和只读确认。
窗口到期不截断当前批次，收完后才切回特征。因此它不是“连续采样 5 秒”，
实际窗口可超时完成，特征窗口也受轮询间隔影响，不能假定严格每 10 秒一个周期。

每批由固件先采集到 PSRAM，再上传 16 个包。采样索引默认仍为 6，
5333.4 Hz 的 1024 点仅代表约 0.192 秒波形；128000 下上传至少约 0.490 秒，
此外还要等待固件当前任务完成。批次之间、特征窗口期间存在没有原始波形的时间。
不能把所有批次拼成无间断波形；波特率低于连续吞吐需求不要求本功能降低采样率。

## 固件控制流程

1. 持有总线锁，读 40052，当前只开放已验证的无校验、采样索引 0～8 组合。
   手册和配置接口允许 0～9；索引 9（26667 Hz）在该原始采集路径尚未验证，
   返回 `-EOPNOTSUPP`。读到大于 9 的非法索引则返回 `-EPROTO`。
   两个上限分别由 `SSF_MEMS_SAMPLING_RATE_MAX_INDEX` 和
   `SSF_MEMS_RAW_SAMPLING_RATE_MAX_INDEX` 表达。
2. 读 40061/40062；若非零，仅用 FC10 写这两个长度寄存器为零并读回。
   固件把零解释为 1024 组 XYZ，不影响 40063～40070，也不发送保存/重启。
3. FC03 读取 **40060**（线上地址 `0x003b`，数量 1），无普通 Modbus 应答。
4. 保持工作波特率接收私有包：
   `15 + BE32 包号 + 64 × XYZ × BE16 + CRC16 + 17`，共 392 字节。
   CRC 覆盖前 389 字节，线上低字节在前；没有从站地址和普通响应字节计数。
5. 每批包号应为 0～15；首包不为零、丢包、重复、越界、CRC 错误均计入异常。
   不将越界包发布至 IIO。收到末包后仍验证包数、错误统计及总线安静窗口。
6. 至少 300 ms 无接收活动后切回 Modbus 解析，以只读 40102 确认通信，
   成功后才允许下一批或特征请求。原始阶段不插入普通请求。
7. 首包预算 15 秒；整批还保留线上传输时间及 2 秒余量。
   超时/信号/卸载时停止发布，等待在途批次自然结束后只读确认。
   恢复失败保留恢复门禁，后台仅重试收尾和只读确认，不自动停流、切速或写 Flash。

整个原始窗口持有 `protocol.bus_lock` 和采集锁；串口配置 sysfs 操作可能等待窗口结束。
卸载不会恢复默认设置，也不会为每轮采集写 40110；接收资源保持存活直到在途批次收尾。
如固件完全失去响应，软件只能有界等待并报告未确认，不能保证自动恢复硬件。
当前无校验以外的配置继续特征轮询，不进入未经验证的原始采集组合。

## 两个 IIO 设备

- `ssf_mems_xyzs`：原有特征通道，扫描布局保持不变。
- `ssf_mems_xyzs_raw`：原始 XYZ、包号、包内序号和可选主机接收时间戳。

原始设备的 `scan_elements`：

| 字段 | scan_index | 存储类型 |
| --- | --- | --- |
| `in_accel_x` / `y` / `z` | 0 / 1 / 2 | 本机字节序 s16 |
| `in_count0_packet_sequence` | 3 | 本机字节序 u32 |
| `in_count1_sample_index` | 4 | 本机字节序 u16，0～63 |
| `in_timestamp` | 5 | 本机字节序 s64，ns |

XYZ 保留固件平均降采样后的 ADC 计数，不是特征 RMS，也不去除重力/DC。
`in_accel_*_scale` 为 `0.004788403` m/s²/count（±16 g，16 位有符号）。
时间戳是主机解析/发布样本的时间，不能当作精确等间距的传感器采样时间。
后续计算 FFT 应结合 `raw_sampling_frequency`、包号和包内序号重建序列，
遇到每批 15/63→0/0 的边界、丢包、重复包或 IIO 丢样都必须分段。
驱动逐包发布，发生错误时此前发布的样本不会回滚，不能把失败批次当成完整波形。

原始设备只提供 buffer 采样及 scale，不提供逐通道 `in_accel_*_raw` 快照。
现有 `tools/iio_read` 读取特征 sysfs 缓存，不能用于读取此高速原始流。

在开发板 root shell 中启用并保存原始 buffer 的示例：

```sh
raw_iio_dir=
for candidate in /sys/bus/iio/devices/iio:device*; do
    if [ "$(cat "$candidate/name" 2>/dev/null)" = "ssf_mems_xyzs_raw" ]; then
        raw_iio_dir="$candidate"
        break
    fi
done
test -n "$raw_iio_dir" || exit 1
echo 0 > "$raw_iio_dir/buffer/enable"
for channel in "$raw_iio_dir"/scan_elements/*_en; do
    echo 1 > "$channel"
done
echo 8192 > "$raw_iio_dir/buffer/length"
echo 1 > "$raw_iio_dir/buffer/enable"
dd if="/dev/${raw_iio_dir##*/}" of=raw_capture.bin bs=24 count=4096 iflag=fullblock
echo 0 > "$raw_iio_dir/buffer/enable"
```

全部字段启用时，一条扫描记录在目标平台为 24 字节：XYZ 位于 0/2/4，
包号位于 8，包内序号位于 12，时间戳位于 16，中间填充为零。
实际程序仍应读取 `*_index`、`*_type` 和启用掩码计算布局，不硬编码设备编号。
buffer 未启用时驱动仍按时交替采集，但不保存未消费的历史样本。

原始设备的只读统计：`raw_active`、`raw_packets`、`raw_sequence_gaps`、
`raw_crc_errors`、`raw_buffer_drops`、`raw_sampling_frequency`。
统计在每轮启动时清零；`raw_packets`、`raw_sequence_gaps`、`raw_crc_errors`
只统计有效采集阶段，不再混入停止排空。`raw_crc_errors` 是损坏候选包次数，不是精确丢包数；
`raw_buffer_drops` 统计 buffer 推送失败的样本数。

新增只读诊断属性：

- `raw_drain_packets`、`raw_drain_sequence_gaps`、`raw_drain_crc_errors`：停止/恢复阶段统计，不发布到 IIO。
- `raw_start_wait_ms`：最近一批首次等待结束的耗时；卸载提前唤醒时不代表首包已到达。
- `raw_link_mode`：0=普通通信，1=等待首包，2=原始收流，3=恢复未确认。
- `raw_publishing`：是否处于向 IIO 发布的采集阶段；buffer 未启用时不存储样本。
- `raw_batches`：本轮原始窗口中成功完成的批次数。
- `raw_batch_packets`：当前批次的 CRC 正确包数，包含卸载时排空的包。
- `raw_batch_samples`：固定为 1024，单位为同步 XYZ 组。
- `raw_ignored_bytes`：无法归入原始包的字节数，非零会使本轮报错。

这些诊断属性按成员读取，不是一份原子快照，观察边界时可能跨越状态切换。

特征在原始模式下不更新，特征缓存会被置为无效；恢复后由普通轮询重新发布。
固件没有特征批次编号/就绪标志，恢复后的首份寄存器结果仍可能是前一批特征，
需要等待固件下一次完成计算才会更新。这不能由 Linux 读取操作强制刷新。

本功能基于固定固件 `samp.c:returnSampDataMode/vTaskAcquisition` 和
`Modbus.c:makeMasterResp_3` 实现。串口吞吐、固件处理延迟和定长结束仍需实机验证；
包号连续也不能证明固件内部完全没有过采样丢失，驱动不会对此作保证。
