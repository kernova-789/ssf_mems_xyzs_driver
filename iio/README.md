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
