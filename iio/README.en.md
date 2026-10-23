# IIO mapping and usage

[简体中文](README.md) | English

The device-tree `compatible` selects the sensor model. XYZS retains its existing
29-channel ABI, while XYZ uses its own 19 channels. XYZ registers the IIO devices
`ssf_mems_xyz` and `ssf_mems_xyz_raw`. See [sensor models](../sensor/README.md)
for differences in channels, units, and capabilities. The examples below use
XYZS device names and attributes.

For the complete attribute reference, permissions, ranges, and examples, see
[IIO attribute usage](ATTRIBUTES.md) (Chinese). For the model-specific lists of
`IIO_DEVICE_ATTR` and `IIO_CONST_ATTR` attributes and userspace reading methods,
see [device attribute macros](DEVICE_ATTRIBUTES.md) (Chinese).

## Channel mapping and cache

`ssf_mems_iio.c` implements the complete IIO interface. The core calls
`ssf_mems_iio_register()` after the serdev transport is ready.

The XYZS 40001–40029 feature register block supports direct reads and software
buffers. Acceleration uses `IIO_ACCEL`, velocity uses `IIO_VELOCITY`, and signed
temperature uses `IIO_TEMP`. Sound features are device-defined dB indicators
exposed as named `IIO_COUNT` channels; they do not represent calibrated pressure
in Pa. Device-specific dimensionless and spectral fields use named `IIO_INDEX`
or `IIO_COUNT` channels. Reading `*_raw` only copies the latest published feature
snapshot and never initiates Modbus I/O. It returns `-ENODATA` before the first
snapshot, during reconnection, or after a read failure invalidates the cache.

The protocol layer atomically updates the feature snapshot after a complete
read succeeds. `ssf_mems_iio_publish_features()` pushes the sample to the IIO
kfifo only when the buffer is enabled. Background acquisition lives in
`acquisition/` and starts after IIO registration. Protocol reads continue to
update the cache while the buffer is disabled.

The publisher holds `indio_dev->mlock` while checking whether the buffer is
enabled, packing scan records, and pushing data. Buffer reconfiguration uses
the same mutex, so disabling the buffer or freeing its active scan mask cannot
race with publication. Acquisition stops before IIO devices are unregistered.
Buffer callbacks must not acquire the acquisition mutex while holding `mlock`.

Scale callbacks convert the normalized cache to IIO units: acceleration uses
0.0980665 m/s² per count, velocity uses 0.00001 m/s per count, temperature uses
10 millidegrees Celsius per count, and sound indicators use 0.01 dB per count.
Register 40026 stores whole percent, register 40027 stores Hz multiplied by 10,
and register 40028 stores the firmware-defined dimensionless spectral-difference
sum multiplied by 100.

## Baud rate and link status

`sensor_baudrate` and `sensor_baudrate_available` are device attributes. Reading
`sensor_baudrate` may perform the complete baud-rate scan implemented by
`ssf_mems_protocol_get_baudrate()`. Writes accept the actual numeric values
listed by `sensor_baudrate_available`. Both operations are serialized with the
acquisition state machine. A successful write also changes the reconnection
target; the background thread verifies communication before resuming polling.

Read-only `sensor_online`, `sensor_poll_interval_ms`, and `sensor_sample_age_ms`
report link status, the current target polling period, and the age of the latest
complete sample. These reads use a short status mutex independent of the
acquisition transaction mutex, so they do not wait for a raw window or firmware
update. Sample age returns `-ENODATA` before the first successful sample.
A reachable device returning Modbus exceptions can remain online while its
feature cache is invalid, so link status and sample age need separate checks.

## Changing acquisition policy at runtime

The following host scheduling attributes reside on the `ssf_mems_xyzs` feature
device and do not change the scan record layout:

| Attribute | Values |
| --- | --- |
| `sensor_acquisition_mode` (read/write) | `raw`, `feature`, `alternating` |
| `sensor_acquisition_mode_available` (read-only) | The three supported mode names |
| `sensor_raw_duration_ms` (read/write) | Raw window, decimal 100..60000 ms |
| `sensor_feature_duration_ms` (read/write) | Feature window, decimal 100..60000 ms |

`raw` repeatedly collects complete finite batches (1024 samples by default) without
polling features. It still reads required configuration and confirms communication
after each batch, so the waveform is not continuous. `feature` continuously polls
features without triggering raw batches. `alternating` starts with a feature
window, then alternates between feature and raw windows. At the raw window
deadline, it finishes the batch in flight rather than truncating data. Feature
polling defaults to a fixed 1000 ms period and also supports the settings below.

The default is alternating mode with 5000 ms raw and feature windows. The two
device-tree durations still supply the initial values. The legacy
`sange-cbm,raw-duration-ms=0` selects feature-only mode, but the raw window reads
back as 5000 ms for later mode changes. Runtime duration writes reject 0; select
a mode to disable a type of acquisition. With parity enabled, the default is
feature-only mode, and selecting a mode that includes raw acquisition returns
`-EOPNOTSUPP` because the current private format supports no parity only.

Run as root and replace X with the device number whose `name` is `ssf_mems_xyzs`:

```sh
IIO=/sys/bus/iio/devices/iio:deviceX
cat "$IIO/sensor_acquisition_mode_available"
echo raw > "$IIO/sensor_acquisition_mode"          # Repeated finite raw batches
echo feature > "$IIO/sensor_acquisition_mode"      # Continuous feature polling
echo 2000 > "$IIO/sensor_raw_duration_ms"
echo 3000 > "$IIO/sensor_feature_duration_ms"
echo alternating > "$IIO/sensor_acquisition_mode"  # About 2 s raw / 3 s features
```

These attributes read or write host state without saving sensor configuration,
restarting the sensor, or changing baud rate or sampling frequency. Reads use a
short status lock and do not wait for serial transactions. Writes share
`acquisition.lock` with acquisition and may wait for the current raw window
(configured up to 60 s, plus the batch in flight and recovery) to finish, then
wake the scheduler. A mode or window change restarts alternating scheduling
from a new feature window; writing the same value does not reset it. The two
windows update independently, so multiple writes are not an atomic transaction.

Raw-only mode no longer provides a valid feature cache. The feature buffer may
still contain records from before the switch; use timestamps to identify them.
Acquisition modes do not automatically enable IIO buffers; the application must
configure and consume the appropriate buffer. Settings can be saved in driver
memory while offline and survive reconnection, but are lost on driver unload.
The next load uses the device tree and defaults again. These controls have only
completed offline regression and target-board kernel compilation; hardware
validation remains pending.

## Feature polling period

The following attributes reside on the `ssf_mems_xyzs` feature device. They
control how often the host reads features without changing the sensor sampling
frequency, baud rate, or raw sample spacing. They apply in `feature` mode and
the feature phase of `alternating`. In `raw` mode, settings are retained but
feature polling does not run.

| Attribute | Access | Default / values |
| --- | --- | --- |
| `sensor_poll_mode` | Read/write | `fixed` (default) or `adaptive` |
| `sensor_poll_mode_available` | Read-only | `fixed adaptive` |
| `sensor_poll_fixed_interval_ms` | Read/write | 1000 ms, range 100..60000 |
| `sensor_poll_min_interval_ms` | Read/write | 500 ms, range 100..60000 |
| `sensor_poll_max_interval_ms` | Read/write | 5000 ms, range 100..60000 |
| `sensor_poll_interval_ms` | Read-only | Current effective period; communication time can affect it |

Run as root and replace X with the feature device number:

```sh
IIO=/sys/bus/iio/devices/iio:deviceX
# Fixed mode: target 2000 ms between the start of consecutive reads.
echo 2000 > "$IIO/sensor_poll_fixed_interval_ms"
echo fixed > "$IIO/sensor_poll_mode"

# Adaptive mode: adjust within 500–5000 ms.
echo 5000 > "$IIO/sensor_poll_max_interval_ms"
echo 500 > "$IIO/sensor_poll_min_interval_ms"
echo adaptive > "$IIO/sensor_poll_mode"
cat "$IIO/sensor_poll_interval_ms"
```

The minimum must not exceed the maximum. Invalid writes return `-EINVAL` and
preserve the previous settings. Attributes update individually: raise the
maximum before raising the minimum above the old maximum; lower the minimum
before lowering the maximum below the old minimum. Numeric values accept only
decimal integers. Equal bounds fix the period at that value.

The fixed period and adaptive bounds are stored separately; switching modes
does not overwrite the other mode's configuration. Adaptive mode starts from
the current period, clamped to its bounds. Fixed mode uses the saved fixed
period. Adaptive mode adjusts after 32 comparisons of adjacent feature samples.
A 30-second probe speeds polling up only when the current sample changed,
repetition in the comparison window is low, and communication has enough time
margin; it does not clear unfinished statistics. Alternating acquisition retains
the learned period. Identical data does not cause periodic probes to speed
polling up. Changing a parameter for the inactive mode does not interrupt polling.

Writes share the acquisition mutex and may wait for the raw window in flight
to finish. Changes to the active mode's parameters wake the scheduler without
restarting the feature window. Settings survive offline periods and reconnection
but not driver unload. These operations do not send sensor configuration commands.
The current period is a scheduling target: if a read takes longer, the driver
does not issue parallel reads or catch up with a backlog.

## Binary firmware update

The feature device provides root-write-only `sensor_firmware_update` and
read-only `sensor_firmware_update_status`. The first accepts a simple `.bin`
filename, without a directory, placed under `/lib/firmware/` on the board.
The second reports the stage, acknowledged/total bytes, error code, version
after reboot, and communication suspension flag. The update write is synchronous.
Status reads use an independent short lock and do not wait on the acquisition
mutex held by the update. Both types of acquisition pause during the update and
resume after success. An uncertain final-packet outcome suspends automatic
communication.

**Read the [image address, bus limits, and operating instructions](../protocol/firmware/OTA.md)
first.** Use an image matching the current hardware and Bootloader. The bin files
in this repository belong to other sensors; no applicable image is currently
available.

## Sensor feature register configuration

The following are XYZS feature configuration attributes.
`sensor_sampling_rate_index_available` generates its list from the model table.
The others query the sensor over Modbus, serialized with acquisition and
baud-rate recovery, and return `-ENODATA` while offline. XYZ does not create the
parameter-switch, feature-enable, or firmware-version attributes.

- `sensor_sampling_rate_index` (read/write): register 40052, index 0 through 9.
- `sensor_sampling_frequency` (read-only): the configured sampling frequency in Hz.
- `sensor_sampling_rate_index_available` (read-only): supported sampling-rate indices.
- `sensor_sampling_length_index` (read/write): register 40053, ordinary sampling-length
  index 0..5; it does not set the finite raw batch size.
- `sensor_parameter_switch` (read-only): register 40070, complete switch word in hex.
- `sensor_feature_enable` (read/write): bits 15..10 of 40070, shifted down to 0..63.
  Bits 5/4/3 enable X/Y/Z high-frequency acceleration RMS; bits 2/1/0 enable
  X/Y/Z low-frequency velocity RMS. `63` enables all six. Writes preserve the
  lower ten bits, including alarm settings.
- `sensor_firmware_version` (read-only): register 40121, an integer firmware version
  code such as `5100`.

Sampling-rate indices correspond to `533.34 888.9 1066.68 1333.35 2666.7 2963 5333.4 8889 13333.5 26667` Hz.
These writable register settings use function code `0x06`, skip unchanged values,
and read back the result. The first connection selects the model-table target
baud rate and sampling-rate index: both models currently use 128000 bit/s;
XYZS selects index 6 (5333.4 Hz), and XYZ selects index 3 (1333.35 Hz).
`current-speed` supplies only an initial matching hint. Matching values skip
writes, and configuration errors enter connection retry. Successful manual baud
and sampling-rate changes become reconnection targets until driver unload.

Avoid rapidly toggling persistent device settings. The driver does not send an
extra save command for sampling rate or feature enables; confirm power-up
behavior on the target device. Readback confirms only the register value; a new
calculation may take several seconds. After configuration, polling clears its
comparison history but retains the configured or learned period. The old cache
is invalidated until the next successful register read, which can still contain
the sensor's previous calculation.

The existing `in_count6_startup_flags_raw` name is retained for compatibility and
represents XYZ run-detection flags. Bits 0/1/2 correspond to X/Y/Z. Zero does not
mean that measurement is disabled. For a board troubleshooting procedure, see
[features remaining unchanged](../tools/README.md#特征值长时间重复的排查).

## Offline validation

Run `python3 tests/iio/run_tests.py` for a pthread stress test of the production
publisher with AddressSanitizer and UndefinedBehaviorSanitizer. It checks packing
for selected masks, signed temperature, run-detection flags, padding, and
timestamps while repeatedly disabling buffers and freeing or replacing scan
masks. It uses an IIO shim, so actual kernel buffer operations still require
hardware verification.

## Raw data device

The separate `ssf_mems_xyzs_raw` buffer provides XYZ, packet sequence, sample
index within a packet, and a host reception timestamp while preserving the
feature device's existing layout. With all data fields and the timestamp
enabled, each record is 24 bytes; with all data fields but no timestamp, it is
16 bytes. See [finite raw capture](../protocol/raw/RAW_CAPTURE.md) for configuration
and reading examples.

A successfully configured sampling rate remains the reconnection target during
the current driver session; unload restores the model-table default. Reads of
host mode, window, and polling settings use a short lock and do not wait for a
raw window or firmware update; writes remain serialized. Feature fields and
units come from `sensor/sensor_fields.h`. Both devices share buffer packing and
registration helpers.

## Userspace-controlled batch length

`sensor_raw_batch_samples` reads/writes the requested batch length;
`sensor_raw_batch_samples_available` lists the allowed lengths. The driver does not
cycle lengths automatically. See [training capture](../tests/training/README.md)
and [attribute definitions](DEVICE_ATTRIBUTES.md) (Chinese) for the userspace schedule
and synchronous manual-window interface. New lengths still require hardware validation.
