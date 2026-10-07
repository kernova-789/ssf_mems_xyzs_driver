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
- `tests/protocol/` — userspace protocol tests with a Linux/serdev shim.

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
`ssf_mems_xyzs`. Reading a channel's `*_raw` file performs a fresh Modbus
feature-block read. The corresponding `*_scale` file converts the register's
fixed-point value to the IIO ABI unit.

The IIO device also provides:

- `sensor_baudrate`: read to query register 40102, or write a numeric baud rate
  to update the sensor and serdev together.
- `sensor_baudrate_available`: supported numeric baud rates.

If the first baud-rate-register request times out, the driver scans each unique
rate from the protocol enum once. It keeps the matching rate after a valid
response and restores the entry rate when no rate responds.

## Tests

Run the protocol and baud-rate scan tests with:

```sh
python3 tests/protocol/run_tests.py
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
