# SSF Modbus Sensor — kernel-module skeleton

This directory is a build skeleton for a Linux Modbus RTU sensor driver.  It
contains no driver implementation; the empty `.c` and `.h` files are intended
for your code.

## Layout

- `core/` — module entry/exit, shared state, and lifecycle management.
- `serdev/` — serial or RS-485 transport based on the Linux `serdev` bus.
- `protocol/` — Modbus RTU framing, CRC, and protocol helpers.
- `sensor/` — sensor-level operations and device-specific register mapping.
- `data/` — measurement representations (temperature and vibration are
  pre-created examples).

Every subsystem Makefile contributes its objects to the single module:
`ssf_sensor.ko`.  Add a new C source by adding its corresponding `.o` path to
the appropriate subsystem Makefile.

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

The output module will be `ssf_sensor.ko` in this directory.  Remove generated
build output with:

```sh
make clean KDIR=/path/to/linux/build
```

## First implementation requirement

Before the module can link, add the module entry and exit functions (typically
`module_init(...)` and `module_exit(...)`) to `core/core.c`.  The Makefile
deliberately has no placeholder implementation, so it does not impose any
driver design on your code.

## In-tree kernel integration

Copy or link this directory into the kernel source tree, source this `Kconfig`
from the parent Kconfig, and add the directory to the parent's Kbuild Makefile,
for example:

```make
obj-$(CONFIG_SSF_SENSOR) += ssf_sensor/
```

When building in-tree, the same top-level Makefile is evaluated by Kbuild and
the `CONFIG_SSF_SENSOR` symbol controls whether the module is included.
