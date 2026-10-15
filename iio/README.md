# IIO mapping

完整中文属性说明、权限、取值范围和操作示例见 [IIO 属性使用说明](ATTRIBUTES.md)。

`ssf_mems_iio.c` owns the complete IIO-facing implementation. Core code only
calls `ssf_mems_iio_register()` after the serdev transport is ready.

The 40001–40029 feature block is exposed in direct and software-buffer modes.
Acceleration values use `IIO_ACCEL`, values in mm/s use `IIO_VELOCITY`, and
signed temperature uses `IIO_TEMP`. The firmware computes sound RMS/peak values
after converting samples to dB, so these are named `IIO_COUNT` channels rather
than pressure channels. Device-specific dimensionless/spectral fields use named
`IIO_INDEX` or `IIO_COUNT` channels.
`*_raw` reads only copy the latest published feature snapshot and return
`-ENODATA` before the first snapshot is available, during reconnect, or after
read failures have made it stale; they never initiate Modbus I/O.

The protocol layer atomically updates the feature snapshot after a complete
read. `ssf_mems_iio_publish_features()` only pushes that sample to the IIO kfifo
when the buffer is enabled. The automatic producer lives in `acquisition/` and
starts after IIO registration. Protocol reads continue updating the cache when
the buffer is disabled.

The producer holds `indio_dev->mlock` throughout the enabled-state check, scan
packing, and buffer push. IIO buffer reconfiguration holds the same mutex, so
disabling a buffer or freeing its active scan mask cannot race with publication.
Acquisition is stopped before unregistering the IIO device. Buffer callbacks
must not acquire the acquisition mutex while holding `mlock`.

Scale callbacks follow the firmware implementation: acceleration and velocity
registers use a centi scale, temperature is a signed centi-degree value exposed
in millidegrees Celsius, and sound dB values use a centi scale. Register 40026
already contains whole percent, register 40027 contains Hz multiplied by 10,
and register 40028 is the firmware's unitless spectral-difference sum multiplied
by 100.

`sensor_baudrate` and `sensor_baudrate_available` are device attributes. Reading
`sensor_baudrate` may perform the complete enum scan described in
`ssf_mems_protocol_get_baudrate()`; writing it accepts one of the numeric values
listed by `sensor_baudrate_available`. Both operations are serialized with the
acquisition state machine. A successful write also changes its recovery target;
the background thread then verifies communication before polling again.

Read-only `sensor_online`, `sensor_poll_interval_ms`, and `sensor_sample_age_ms`
report link status, the current target polling period, and the age of the latest
complete sample. These three reads use a short status mutex, independent of
the acquisition transaction mutex; they do not wait for a raw window or OTA. Before the first successful sample, the age returns `-ENODATA`.
A reachable device returning Modbus exceptions can be online while its feature
cache is stale, so link status and sample age have separate meanings.

## 运行中修改采集策略

在特征设备 `ssf_mems_xyzs` 下增加以下主机调度属性，不改变扫描记录布局：

| 属性 | 取值 |
| --- | --- |
| `sensor_acquisition_mode`（读写） | `raw`、`feature`、`alternating` |
| `sensor_acquisition_mode_available`（只读） | 支持的三个模式名称 |
| `sensor_raw_duration_ms`（读写） | 原始窗口，十进制 100..60000 ms |
| `sensor_feature_duration_ms`（读写） | 特征窗口，十进制 100..60000 ms |

`raw` 持续重复完整 1024 点批次，不读取特征。
它仍会执行必要的配置读取和批后通信确认，因此不是无间断原始波形。
`feature` 持续轮询特征，不触发原始批次。`alternating` 从特征窗口开始交替，
到原始窗口截止时收完在途批次，不为了满足时长截断数据。特征轮询默认固定为
1000 ms，也可使用下面的独立轮询配置。

默认是交替模式、原始/特征各 5000 ms；设备树两项时段继续提供初始值。
旧 `sange-cbm,raw-duration-ms=0` 仍选择仅特征，但原始窗口读回为可供以后切换的
5000 ms。运行时写时长不接受 0，请用模式选择关闭某类采集。有校验位时默认仅特征，
请求含原始采集的模式返回 `-EOPNOTSUPP`，因为当前私有格式只支持无校验。

以 root 身份操作，先将路径中的 X 换为名字是 `ssf_mems_xyzs` 的设备编号：

```sh
IIO=/sys/bus/iio/devices/iio:deviceX
cat "$IIO/sensor_acquisition_mode_available"
echo raw > "$IIO/sensor_acquisition_mode"          # 持续定长采集
echo feature > "$IIO/sensor_acquisition_mode"      # 持续特征采集
echo 2000 > "$IIO/sensor_raw_duration_ms"
echo 3000 > "$IIO/sensor_feature_duration_ms"
echo alternating > "$IIO/sensor_acquisition_mode"  # 原始约 2 s / 特征约 3 s
```

这些属性只读写主机状态，不发送保存命令、不重启、不改变波特率或采样率。
配置与采集共用 `acquisition.lock`；写入可能等待当前原始窗口（最长配置 60 s 加
在途批次/恢复等待）收尾，返回后唤醒调度。模式/窗口变化后交替调度从新的特征窗口
开始；同值写不重置窗口。两个窗口分别更新，多个属性写入不是一次原子事务。
纯原始模式不再提供有效特征缓存；特征 buffer 可能仍存有切换前的旧记录，须按
时间戳识别。采集模式不自动开启 IIO buffer，仍需应用自行配置并消费对应 buffer。
设置可在离线时保存于驱动内存并在重连后沿用；卸载后不保留，下一次加载重新使用
设备树/默认值。新功能只完成离线回归和目标板内核编译，尚未实机验证。

## 特征轮询周期

以下属性位于特征设备 `ssf_mems_xyzs`，控制主机读取特征的间隔，不改变传感器
采样率、波特率或原始数据采样间隔。它们适用于 `feature` 和 `alternating` 的
特征阶段；`raw` 模式保存配置但不执行特征轮询。

| 属性 | 权限 | 默认值/取值 |
| --- | --- | --- |
| `sensor_poll_mode` | 读写 | `fixed`（默认）或 `adaptive` |
| `sensor_poll_mode_available` | 只读 | `fixed adaptive` |
| `sensor_poll_fixed_interval_ms` | 读写 | 1000 ms，范围 100..60000 |
| `sensor_poll_min_interval_ms` | 读写 | 500 ms，范围 100..60000 |
| `sensor_poll_max_interval_ms` | 读写 | 5000 ms，范围 100..60000 |
| `sensor_poll_interval_ms` | 只读 | 当前生效的周期，可能受通信耗时影响 |

以 root 身份操作，X 替换为特征设备编号：

```sh
IIO=/sys/bus/iio/devices/iio:deviceX
# 固定模式：两轮读取开始时间的目标间隔为 2000 ms。
echo 2000 > "$IIO/sensor_poll_fixed_interval_ms"
echo fixed > "$IIO/sensor_poll_mode"

# 自适应模式：在 500～5000 ms 内调整周期。
echo 5000 > "$IIO/sensor_poll_max_interval_ms"
echo 500 > "$IIO/sensor_poll_min_interval_ms"
echo adaptive > "$IIO/sensor_poll_mode"
cat "$IIO/sensor_poll_interval_ms"
```

下限不能超过上限，非法写入返回 `-EINVAL` 并保留旧配置。属性逐项更新，调整范围
时应按当前值选择顺序：提高下限越过旧上限之前先提高上限，降低上限越过旧下限之前
先降低下限。数值只接受十进制整数；上下限相等时周期被限定在该值。

固定周期和自适应上下限分别保存，切换模式不会覆盖另一个模式的配置。进入自适应
时从当前周期开始并限制到上下限；切回固定时使用保存的固定周期。自适应按 32 次
相邻特征比较调整周期，30 秒定期试探不清空未完成统计；交替采集也保留学习结果。
只修改非当前模式的参数不会打断当前轮询。

写入与采集共用采集锁，可能等待在途原始窗口完整结束；当前模式的参数变化会唤醒
调度，不重新开始特征窗口。配置在离线、重连后继续保留，本次驱动卸载后不保留。
这些操作不发送传感器配置命令。当前周期是调度目标，读取耗时超过周期时不会并行
发送或补发积压轮次。

## bin 固件升级

特征设备新增 root 只写的 `sensor_firmware_update` 和只读的
`sensor_firmware_update_status`。前者接受放在开发板 `/lib/firmware/` 下的 `.bin`
简单文件名；后者返回阶段、已应答字节数/总字节数、错误码、重启后版本号和暂停标志。
升级写入为同步操作，状态读取使用独立短锁，不会因升级占用采集锁而等待。
升级时暂停两种采集，成功后恢复；末包结果不确定时暂停自动通信。

**必须先阅读 [镜像地址、总线限制和操作说明](../protocol/OTA.md)**。
这不是对任意 bin 的通用烧录接口；目录内原有 bin 的链接地址不适合这套 OTA。

## Sensor feature register configuration

The following attributes query the sensor over Modbus, serialized with
acquisition and baud-rate recovery. They return `-ENODATA` while offline:

- `sensor_sampling_rate_index` (RW): register 40052, index 0 through 9.
- `sensor_sampling_frequency` (RO): the configured frequency in Hz.
- `sensor_sampling_rate_index_available` (RO): supported indices.
- `sensor_sampling_length_index` (RO): register 40053, raw-data length index.
- `sensor_parameter_switch` (RO): register 40070, complete switch word in hex.
- `sensor_feature_enable` (RW): bits 15..10 of 40070, shifted down to 0..63.
  Bit 5/4/3 enables X/Y/Z high-frequency acceleration RMS; bit 2/1/0 enables
  X/Y/Z low-frequency velocity RMS. `63` enables all six. Writes preserve
  the lower ten bits, including alarm settings.
- `sensor_firmware_version` (RO): register 40121 (the provided firmware is 5100).

Sampling rate indices correspond to `533.34 888.9 1066.68 1333.35 2666.7 2963
5333.4 8889 13333.5 26667` Hz. Writes use 0x06, skip unchanged register values,
and read back the result. Automatic connection/reconnection selects the target
baud rate (`current-speed`, or 9600 if missing/unreadable/unsupported), then sets sampling-rate index 6 (5333.4 Hz) before
publishing samples. Matching values skip writes; configuration errors use the
connection retry path. A manual sampling-rate change lasts until the next
reconnection, which selects index 6 again. MEMS2.0 saves rate changes to Flash in
`returnSampDataMode()`, so avoid rapidly toggling this setting. Feature enables
are initialized to all on by that firmware at power-up and are not saved here.
Readback confirms the register value; a new calculation may take several seconds.
After configuration, polling returns to its initial 1000 ms period and the old
cache is invalidated until the next successful register read. That next read
can still contain the sensor's previous calculation.

The existing `in_count6_startup_flags_raw` name is retained for compatibility,
but MEMS2.0 publishes **motor run detection**, not acquisition enable state:
each XYZ bit is set when the corresponding DC-removed acceleration peak is at
least 0.06 g. Zero does not mean that measurement is disabled.

See [tools/README.md](../tools/README.md#特征值长时间重复的排查) for a board test.

Run `python3 tests/iio/run_tests.py` for a pthread stress test of the production
publisher with AddressSanitizer/UndefinedBehaviorSanitizer. It checks packing
for selected masks, signed temperature, startup flags, padding and timestamps
while repeatedly disabling buffers and freeing/replacing masks. It uses an IIO
shim, so actual kernel buffer operations still require hardware verification.
# 原始数据设备

新增 `ssf_mems_xyzs_raw`，提供 XYZ、包号、包内序号及主机接收时间戳的独立
buffer，保持特征设备原有布局不变。全部字段启用时扫描记录为 24 字节。
配置与读取示例见[定长原始采集说明](../protocol/RAW_CAPTURE.md)。
