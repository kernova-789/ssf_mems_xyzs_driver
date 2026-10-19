# SSF MEMS Modbus / IIO driver

This directory contains a Linux serdev driver for the SSF MEMS Modbus RTU
sensors SSF-MEMS-XYZS and SSF-MEMS-XYZ (V2.4). It validates and decodes Modbus responses, exposes the sensor feature
registers through IIO, and supports sensor baud-rate discovery and updates.

On XYZS, application-bin OTA is available through `sensor_firmware_update` and
`sensor_firmware_update_status` on the feature IIO device. See
[OTA requirements and board commands](protocol/firmware/OTA.md): the image must be linked
at `0x08020000`, at most 256 KiB, and the RS485 bus must contain only the target
sensor. No matching update image is currently available; the reference-tree bin
belongs to another sensor and is not used by the driver tests. OTA has synthetic
offline coverage and a target-kernel build, not a hardware flashing result.

## Layout

- `core/` — serdev probe/remove and shared device state.
- `sensor/` — model-specific layouts, register-to-member offset maps, profiles and
  acquisition callbacks; see [sensor architecture and model selection](sensor/README.md).
- `protocol/` — shared business and protocol components:
  `model/` for common RTU frame descriptions, `modbus/` for RTU framing and requests,
  `transport/` for serial I/O, `raw/` for finite capture, and `firmware/` for OTA.
  See [protocol layout](protocol/README.md) and [sensor configuration](sensor/README.md).
- `iio/` — IIO channel definitions, direct reads, registration, and IIO sysfs
  controls.
- `acquisition/` — automatic startup, polling, adaptive timing, IIO sample
  publishing, and communication recovery.
- `tests/protocol/` — userspace protocol tests with a Linux/serdev shim.
- `tests/acquisition/` — deterministic acquisition policy and lifecycle tests.
- `tests/iio/` — scan layout, attributes, registration and concurrent publication tests.
- `tests/ota/` — application image validation, staged-Flash transfer and fault tests.
- `tools/` — userspace IIO polling/printf test for the development board;
  see [tools/README.md](tools/README.md) for build and run instructions.

The top-level Kbuild Makefile links all subsystem objects into the single
`ssf_mems.ko` module. Add a new C source by adding its corresponding `.o` path
there.

## External module build

Build against a **prepared** kernel build tree whose configuration and headers
match the target kernel:

```sh
make KDIR=/path/to/linux/build
```

For cross-compilation, pass the target architecture and toolchain prefix:

```sh
make KDIR=/path/to/linux/build ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
```

The output module is `ssf_mems.ko` in this directory. Remove generated
build output with:

```sh
make clean KDIR=/path/to/linux/build
```

The default `make` target also runs `modules_prepare` and copies the module to
`INSTALL_DIR` (default `/home/u-yoimiya/nfs/`). Pass `INSTALL_DIR=/your/output`
to change that destination. To build without copying, invoke Kbuild directly:

```sh
make -C /path/to/linux/build M="$PWD" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules
```

## IIO interface

After the serdev device binds, the driver registers an IIO device named
`ssf_mems_xyzs` or `ssf_mems_xyz`, selected by the device-tree compatible.
XYZS retains its 29-feature ABI; XYZ exposes its own 19 features, including
three-axis displacement peak-to-peak (µm, IIO scale 0.000001 m). XYZ temperature
is normalized from signed °C ×10 to °C ×100. Unsupported sound, frequency-band,
feature-enable, firmware-version and OTA interfaces are absent on XYZ.
See [model differences and raw-format verification limits](sensor/README.md).
A channel's `*_raw` file reads the latest complete feature
snapshot. The corresponding `*_scale` file converts the register's fixed-point
value to the channel unit.

For XYZS, the normalized feature cache stores acceleration in g ×100, velocity in mm/s ×100,
temperature in °C ×100, sound in firmware-computed dB ×100, zero-crossing rate
in whole percent, spectral centroid in Hz ×10, and spectral flux ×100.
IIO scale converts acceleration to m/s², velocity to m/s, and temperature to
millidegrees Celsius. Negative temperature behavior on the target device
still requires confirmation; see [units and measurement limits](iio/ATTRIBUTES.md#12-单位与测量边界).

Both models provide the common acquisition and communication controls below;
feature-enable, parameter-switch and firmware-version controls are XYZS-only:

- `sensor_baudrate`: read to confirm the active communication rate using 40102,
  or write a numeric baud rate to stage 40102, save/reboot via 40110=1, wait
  8000 ms, switch the host UART and verify the new rate. Verification retries
  timed-out reads up to eight times without repeating save writes. The sysfs
  write may therefore take several seconds. Writing the active rate first reads
  back the configuration; matching communication and register values skip saving
  and rebooting. A successful write also
  changes the recovery target. The reboot wait holds the bus lock and is tunable
  via `SSF_MEMS_SENSOR_REBOOT_DELAY_MS` in `protocol.h`; verification attempts
  and retry delay are configured in the same header.
- `sensor_baudrate_available`: supported numeric baud rates.
- `sensor_online`: whether acquisition has verified the connection (0/1).
- `sensor_poll_interval_ms`: current polling period in milliseconds.
- `sensor_poll_mode` (RW): `fixed` (default) or `adaptive`.
- `sensor_poll_fixed_interval_ms` (RW): fixed period, default 1000 ms.
- `sensor_poll_min_interval_ms` / `sensor_poll_max_interval_ms` (RW): adaptive
  bounds, default 500 / 5000 ms. All periods accept 100..60000 ms, with min <= max.
- `sensor_sample_age_ms`: age of the latest complete sample; `-ENODATA` before
  the first successful acquisition.
- `sensor_sampling_rate_index` (RW) and `sensor_sampling_frequency` (RO):
  sensor calculation rate, independent of the host polling period. Index 9
  selects the maximum 26667 Hz, while index 6 selects 5333.4 Hz. Firmware saves
  changes to Flash and restores them on startup. The first driver connection
  selects the model-table default (XYZS: index 6; XYZ: index 3). Later reconnections use
  the last successfully selected index; reloading the driver resets that target.
- `sensor_feature_enable` (RW): six frequency-band enables (63 enables all),
  preserving alarm settings; `sensor_parameter_switch` reads the full word.
- `sensor_sampling_length_index` (RW): ordinary sampling length index 0..5;
  separate from the 1024-point finite raw batches. `sensor_firmware_version`
  (RO) provides a version diagnostic on supported models. See [iio/README.md](iio/README.md) and
  [the board test](tools/README.md#特征值长时间重复的排查).

Successful host reads may repeatedly return the same feature values.
`sensor_sample_age_ms` measures time since a successful register read, not time
since sensor-side measurement. Register 40029 (`startup_flags`) reports XYZ
run-detection flags; zero does not mean acquisition is disabled.

After registration, a background thread waits one second, matches the sensor's
baud rate, uses `current-speed` only as a startup hint (9600 if missing, unreadable
or unsupported), applies the model-table operational target (currently 128000),
sets the initial sampling-rate index (currently 6, 5333.4 Hz), and verifies a
complete feature read (raw-only mode instead starts finite batches after
configuration verification). Matching baud rates
and unchanged sampling settings skip writes. If the initial target rate does
not respond, discovery finds the active rate before saving/rebooting the sensor
to the target. No responding rate means retrying, not blindly writing a new rate;
configuration failures use the
connection retry path before publishing samples.
It defaults to a fixed polling period of 1000 ms for the measured 4.4–4.8 s
feature-update interval. Runtime IIO controls select fixed or adaptive polling,
set the fixed period, and configure adaptive bounds. Adaptive statistics survive
normal raw windows. Periodic probing requires changing samples, a repeat rate
below 10%, and enough read-time headroom; it preserves unfinished statistics.
Polling configuration survives reconnection but is reset on driver reload.
See [polling controls](iio/README.md#特征轮询周期). Sustained communication failure stops polling
and starts baud-rate discovery with exponential retry delays capped at 30 s.
Stale or disconnected samples become invalid and direct reads return `-ENODATA`.
See [acquisition/README.md](acquisition/README.md) for the policy and tuning points.

Runtime IIO controls `sensor_acquisition_mode` (`raw`, `feature`, `alternating`),
`sensor_raw_duration_ms`, and `sensor_feature_duration_ms` select the host-side
schedule. Durations accept 100..60000 ms; the default remains alternating with
5000 ms windows. Updates wait for the current raw window to finish and do not
save sensor configuration. See [IIO controls](iio/README.md#运行中修改采集策略).
The serdev receive callback now explicitly uses the target Linux 5.10 `int`
signature; the former `typeof` typedef for cross-kernel return-type inference
has been removed. A newer kernel with a `size_t` callback needs adaptation.

Baud changes stage 40102, save/reboot through 40110=1 at the old rate, wait,
switch the host UART, and verify. Sampling configuration follows that reboot;
the driver reads the setting back without an additional save/reboot request.
Readback confirms the configured value, not the start of a new measurement. Hardware validation of communication and
measurement latency is still required. Sensor parity is never changed.

If the first baud-rate-register request times out, the driver scans each unique
rate from the protocol enum once. It keeps the matching rate after a valid
response and restores the entry rate when no rate responds.

## Device-tree transport settings

The host must already know the slave address and parity before it can exchange
Modbus frames. Configure them on the serdev child node together with the initial
baud-rate hint. The operational target comes from the model table:

```dts
&uart1 {
    status = "okay";

    mems-sensor {
        compatible = "sange-cbm,ssf-mems-xyzs";
        current-speed = <9600>;
        sange-cbm,slave-id = <1>;
        sange-cbm,parity = "none";
    };
};
```

For XYZ V2.4, use `compatible = "sange-cbm,ssf-mems-xyz"`; all transport
properties keep the same meaning. The compatible selects a model per instance
and binds its `read_features` / `collect_raw` callbacks in probe.

`sange-cbm,slave-id` accepts 1 through 247, and `sange-cbm,parity` accepts
`"none"`, `"odd"`, or `"even"`. Missing or unreadable properties use the manual defaults:
address 1, 9600 baud, and no parity, with an info log for each fallback.
Unsupported `current-speed` values also log and fall back to 9600; a controller
that rejects a non-default startup rate is retried at 9600. Properties are parsed
internally by protocol initialization, without sending Modbus commands.
A readable but invalid slave address or parity still fails probe.

## Tests

Run the protocol and baud-rate scan tests with:

```sh
python3 tests/protocol/run_tests.py
python3 tests/acquisition/run_tests.py
python3 tests/iio/run_tests.py
python3 tests/ota/run_tests.py
```

## In-tree kernel integration

The supplied Makefile uses `obj-m` for external-module builds. For in-tree
integration, first replace that line with:

```make
obj-$(CONFIG_SSF_SENSOR) += ssf_mems.o
```

Then source this directory's `Kconfig` from the parent Kconfig and add the
directory to the parent's Kbuild Makefile, for example:

```make
obj-$(CONFIG_SSF_SENSOR) += ssf_sensor/
```

After this adjustment, `CONFIG_SSF_SENSOR` controls the in-tree build.

# 定长原始采集

支持 1024 点定长原始 XYZ 批次与特征轮询交替运行，窗口默认各 5 秒，
窗口到期收完在途批次；两种通信共用工作波特率，默认 128000。
批次之间有采样时间间隙，不是全时连续波形。原始数据在独立 IIO
设备 `ssf_mems_xyzs_raw` / `ssf_mems_xyz_raw` 中提供，原有特征设备 ABI 不变；暂不计算 FFT。
设备树时段配置、私有协议限制和 buffer 读取方法见
[定长原始采集说明](protocol/raw/RAW_CAPTURE.md)。

IIO 全部属性的中文说明和操作示例见 [iio/ATTRIBUTES.md](iio/ATTRIBUTES.md)。
