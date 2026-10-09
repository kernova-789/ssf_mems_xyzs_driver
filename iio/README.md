# IIO mapping

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

`ssf_mems_iio_publish_features()` atomically replaces that snapshot and pushes
the same sample to the IIO kfifo when the buffer is enabled. The automatic
producer lives in `acquisition/` and starts after IIO registration. It continues
updating the cache when the buffer is disabled.

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
complete sample. Before the first successful sample, the age returns `-ENODATA`.
A reachable device returning Modbus exceptions can be online while its feature
cache is stale, so link status and sample age have separate meanings.

## Sensor feature configuration

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
配置与读取示例见[连续原始采集说明](../protocol/RAW_CAPTURE.md)。
