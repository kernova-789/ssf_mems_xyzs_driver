# IIO mapping

`ssf_mems_iio.c` owns the complete IIO-facing implementation. Core code only
calls `ssf_mems_iio_register()` after the serdev transport is ready.

The 40001–40029 feature block is exposed in direct and software-buffer modes. Values documented in g
use `IIO_ACCEL`, values documented in mm/s use `IIO_VELOCITY`, temperature uses
`IIO_TEMP`, sound-pressure features use `IIO_PRESSURE`, and device-specific
dimensionless/spectral fields use named `IIO_INDEX` or `IIO_COUNT` channels.
`*_raw` reads only copy the latest published feature snapshot and return
`-ENODATA` before the first snapshot is available; they never initiate Modbus
I/O.

`ssf_mems_iio_publish_features()` atomically replaces that snapshot and pushes
the same sample to the IIO kfifo when the buffer is enabled. The future polling
producer is intentionally outside this module and is not implemented yet, so
enabling the buffer alone currently produces no samples.

The sensor stores most values multiplied by 100. Scale callbacks convert these
to IIO ABI units: acceleration to m/s², velocity to m/s, temperature to
millidegrees Celsius, and pressure to kPa. Other named metrics retain their
documented centi-unit scale.

`sensor_baudrate` and `sensor_baudrate_available` are device attributes. Reading
`sensor_baudrate` may perform the complete enum scan described in
`ssf_mems_protocol_get_baudrate()`; writing it accepts one of the numeric values
listed by `sensor_baudrate_available`.
