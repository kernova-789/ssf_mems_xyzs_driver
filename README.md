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

- `sensor_baudrate`: read to query register 40102, or write a numeric baud rate
  to update the sensor and serdev together, and change the recovery target.
- `sensor_baudrate_available`: supported numeric baud rates.
- `sensor_online`: whether acquisition has verified the connection (0/1).
- `sensor_poll_interval_ms`: current polling period in milliseconds.
- `sensor_sample_age_ms`: age of the latest complete sample; `-ENODATA` before
  the first successful acquisition.

After registration, a background thread waits one second, matches the sensor's
baud rate, switches both sides to 115200, and verifies a complete feature read.
It polls at an initial period of 100 ms, adapting within 20–1000 ms using
32 adjacent-sample comparisons. Sustained communication failure stops polling
and starts baud-rate discovery with exponential retry delays capped at 30 s.
Stale or disconnected samples become invalid and direct reads return `-ENODATA`.
See [acquisition/README.md](acquisition/README.md) for the policy and tuning points.

The current baud-change assumption is that the sensor replies at the old rate
and then switches; no save or reboot commands are sent. Hardware validation is
still required. Sensor parity is never changed.

If the first baud-rate-register request times out, the driver scans each unique
rate from the protocol enum once. It keeps the matching rate after a valid
response and restores the entry rate when no rate responds.

## Device-tree transport settings

The host must already know the slave address and parity before it can exchange
Modbus frames. Configure them on the serdev child node together with the initial
baud rate:

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
Properties are parsed internally by protocol initialization, without sending
Modbus commands. A readable but unsupported value fails probe instead
of silently using a different host configuration.

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
