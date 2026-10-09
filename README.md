# SSF MEMS Modbus / IIO driver

This directory contains a Linux serdev driver for the SSF MEMS Modbus RTU
sensor. It validates and decodes Modbus responses, exposes the sensor feature
registers through IIO, and supports sensor baud-rate discovery and updates.

## Layout

- `core/` — serdev probe/remove and shared device state.
- `protocol/` — Modbus RTU framing, request matching, register map, feature
  decoding, and baud-rate operations.
- `iio/` — IIO channel definitions, direct reads, registration, and IIO sysfs
  controls.
- `acquisition/` — automatic startup, polling, adaptive timing, IIO sample
  publishing, and communication recovery.
- `tests/protocol/` — userspace protocol tests with a Linux/serdev shim.
- `tests/acquisition/` — deterministic acquisition policy and lifecycle tests.
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

## IIO interface

After the serdev device binds, the driver registers an IIO device named
`ssf_mems_xyzs`. A channel's `*_raw` file reads the latest complete feature
snapshot. The corresponding `*_scale` file converts the register's fixed-point
value to the channel unit.

The firmware encodings used by the channels are: acceleration and velocity
values multiplied by 100, signed temperature multiplied by 100, sound features
in dB multiplied by 100, zero-crossing rate in whole percent, spectral centroid
in Hz multiplied by 10, and spectral flux multiplied by 100.

The IIO device also provides:

- `sensor_baudrate`: read to confirm the active communication rate using 40102,
  or write a numeric baud rate to stage 40102, save/reboot via 40110=1, wait
  1000 ms, switch the host UART and verify the new rate. Verification retries
  timed-out reads up to eight times without repeating save writes. The sysfs
  write may therefore take several seconds. A successful write also
  changes the recovery target. The reboot wait holds the bus lock and is tunable
  via `SSF_MEMS_SENSOR_REBOOT_DELAY_MS` in `protocol.h`; verification attempts
  and retry delay are configured in the same header.
- `sensor_baudrate_available`: supported numeric baud rates.
- `sensor_online`: whether acquisition has verified the connection (0/1).
- `sensor_poll_interval_ms`: current polling period in milliseconds.
- `sensor_sample_age_ms`: age of the latest complete sample; `-ENODATA` before
  the first successful acquisition.
- `sensor_sampling_rate_index` (RW) and `sensor_sampling_frequency` (RO):
  sensor calculation rate, independent of the host polling period. Index 9
  selects the maximum 26667 Hz, while index 6 selects 5333.4 Hz. Firmware saves
  changes to Flash and restores them on startup. Automatic connection/reconnection
  selects index 6 for the current tapping/general-vibration tests.
- `sensor_feature_enable` (RW): six frequency-band enables (63 enables all),
  preserving alarm settings; `sensor_parameter_switch` reads the full word.
- `sensor_sampling_length_index` and `sensor_firmware_version` (RO): sensor
  configuration diagnostics. See [iio/README.md](iio/README.md) and
  [the board test](tools/README.md#特征值长时间重复的排查).

The provided MEMS2.0 firmware computes a whole feature batch before updating
its holding registers. Successful host reads may repeatedly return that same
batch. `sensor_sample_age_ms` measures time since a successful register read,
not time since sensor-side measurement. Its register 40029 (`startup_flags`)
reports XYZ motor-run detection at a 0.06 g DC-removed peak threshold, rather
than acquisition enable state.

After registration, a background thread waits one second, matches the sensor's
baud rate, uses `current-speed` as the target rate (9600 if missing, unreadable
or unsupported), sets sampling-rate
index 6 (5333.4 Hz), and verifies a complete feature read. Matching baud rates
and unchanged sampling settings skip writes. If the initial target rate does
not respond, discovery finds the active rate before saving/rebooting the sensor
to the target. No responding rate means retrying, not blindly writing a new rate;
configuration failures use the
connection retry path before publishing samples.
It polls at a fixed period of 1000 ms for the measured 4.4–4.8 s feature-update
interval. The adaptive policy remains available, but equal minimum and maximum
periods currently keep it at 1000 ms. Sustained communication failure stops polling
and starts baud-rate discovery with exponential retry delays capped at 30 s.
Stale or disconnected samples become invalid and direct reads return `-ENODATA`.
See [acquisition/README.md](acquisition/README.md) for the policy and tuning points.

Baud changes stage 40102, save/reboot through 40110=1 at the old rate, wait,
switch the host UART, and verify. Sampling configuration follows that reboot;
firmware applies it after its current batch and saves it without an additional
save/reboot request. Hardware validation of communication and
measurement latency is still required. Sensor parity is never changed.

If the first baud-rate-register request times out, the driver scans each unique
rate from the protocol enum once. It keeps the matching rate after a valid
response and restores the entry rate when no rate responds.

## Device-tree transport settings

The host must already know the slave address and parity before it can exchange
Modbus frames. Configure them on the serdev child node together with the initial
and target baud rate:

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
```

## In-tree kernel integration

Copy or link this directory into the kernel source tree, source this `Kconfig`
from the parent Kconfig, and add the directory to the parent's Kbuild Makefile,
for example:

```make
obj-$(CONFIG_SSF_SENSOR) += ssf_sensor/
```

When building in-tree, the same top-level Makefile is evaluated by Kbuild and
the `CONFIG_SSF_SENSOR` symbol controls whether the module is included.

# 连续原始采集

支持连续原始 XYZ 与特征轮询交替运行，默认各 5 秒。原始流在独立 IIO
设备 `ssf_mems_xyzs_raw` 中提供，原有特征设备 ABI 不变；暂不计算 FFT。
设备树时段配置、私有协议限制和 buffer 读取方法见
[连续原始采集说明](protocol/RAW_CAPTURE.md)。
