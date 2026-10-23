# 两种传感器的 IIO_DEVICE_ATTR / IIO_CONST_ATTR 清单

本文依据当前 [ssf_mems_iio.c](ssf_mems_iio.c)、[XYZ 型号表](../sensor/xyz.c)
和 [XYZS 型号表](../sensor/xyzs.c) 整理，说明驱动注册的设备属性及用户态读取方法。
这是当前源码的接口说明；目标板实际可见的文件取决于已加载的驱动版本和设备树配置。

## 1. 设备、宏和权限

每个传感器实例注册两个独立的 IIO 设备：

| 传感器 | 特征设备的 `name` | 原始数据设备的 `name` |
| --- | --- | --- |
| SSF-MEMS-XYZ | `ssf_mems_xyz` | `ssf_mems_xyz_raw` |
| SSF-MEMS-XYZS | `ssf_mems_xyzs` | `ssf_mems_xyzs_raw` |

`sensor_*` 属性在特征设备目录下，`raw_*` 属性在原始数据设备目录下。
两个设备共享同一个物理传感器，编号由内核分配，不能固定假设为 0、1。

```c
IIO_DEVICE_ATTR(name, mode, show, store, address);
IIO_CONST_ATTR(name, string);
```

- `name`：最终的 sysfs 文件名；C 变量名中的 `iio_dev_attr_` 不会出现在文件名中。
- `mode`：权限。`0444` 为所有用户只读；`0644` 为所有用户可读、所有者可写；
  `0200` 为仅所有者可写，通常所有者是 root。
- `show`：用户读取属性文件时调用的回调，返回文本或负数错误码。
- `store`：用户写入属性文件时调用的回调；为 `NULL` 表示没有写入接口。
- `address`：本驱动用于让共用回调区分属性的内部编号，**不是 Modbus 寄存器地址**。
- `IIO_CONST_ATTR`：提供固定字符串，当前两项均为只读；不需要逐项编写 `show` 回调。

宏只定义属性对象，属性还要加入属性数组并随 IIO 设备注册才会创建文件。
特征设备在注册前根据型号能力筛选属性，因此 XYZ 不支持的属性不会创建文件。

| 设备 | `IIO_DEVICE_ATTR` 数量 | `IIO_CONST_ATTR` 数量 | 本文统计的属性总数 |
| --- | ---: | ---: | ---: |
| XYZ 特征设备 | 20 | 2 | 22 |
| XYZS 特征设备 | 25 | 2 | 27 |
| XYZ 原始数据设备 | 16 | 0 | 16 |
| XYZS 原始数据设备 | 16 | 0 | 16 |

这里不计 `name`、`buffer/*`、`scan_elements/*`，以及由通道定义生成的
`in_*_raw`、`in_*_scale`。这些文件不是本文件中显式定义的上述两类宏属性。

## 2. 特征设备：两种型号共有的 20 个 IIO_DEVICE_ATTR

以下每项在 XYZ 和 XYZS 的特征设备上都会创建。
表中的“串口读取”表示读取时经过采集层访问传感器，可能等待当前采集事务；
“主机状态”或“型号表”表示读取驱动内存，不主动发送串口请求。

### 2.1 传感器参数与可选值

| 属性文件名 | 权限 | 读取结果 / 含义 | 读取来源 |
| --- | --- | --- | --- |
| `sensor_baudrate` | `0644` 读写 | 当前可通信波特率，单位 bit/s，例如 `128000`；使用实际数值而非寄存器编码 | 串口确认；当前速率超时时可能扫描其他速率 |
| `sensor_baudrate_available` | `0444` 只读 | 支持的波特率，以空格分隔，去除重复值 | 型号表 |
| `sensor_sampling_rate_index` | `0644` 读写 | 采样率编码，当前为 `0`～`9`，对应寄存器 40052 | 串口读取 |
| `sensor_sampling_rate_index_available` | `0444` 只读 | `0 1 2 3 4 5 6 7 8 9` | 型号表 |
| `sensor_sampling_frequency` | `0444` 只读 | 读取采样率编码并换算为 Hz 文本，例如 `1333.35` | 串口读取 40052，再查型号表 |
| `sensor_sampling_length_index` | `0644` 读写 | 普通采样长度编码 `0`～`5`，对应寄存器 40053 | 串口读取 |

当前两种型号的采样率编码相同：

| 编码 | Hz | 编码 | Hz |
| --- | --- | --- | --- |
| 0 | 533.34 | 5 | 2963 |
| 1 | 888.9 | 6 | 5333.4 |
| 2 | 1066.68 | 7 | 8889 |
| 3 | 1333.35 | 8 | 13333.5 |
| 4 | 2666.7 | 9 | 26667 |

普通采样长度编码 `0`～`5` 依次对应 1024、2048、4096、8192、16384、32768 点。
它与驱动定长原始采集的批次长度是不同配置；当前原始批次通过自定义长度寄存器设置为
1024 组 XYZ，不能用 `sensor_sampling_length_index` 推断实际批次长度。

当前波特率可选列表为：

```text
2400 4800 9600 19200 38400 57600 115200 128000 230400 256000 460800 500000 512000 600000 750000 921600 1000000
```

可选列表说明型号表接受这些设置，不代表每种采样率与波特率组合都已通过实机稳定性验证。

### 2.2 主机采集调度配置

| 属性文件名 | 权限 | 读取结果 / 含义 |
| --- | --- | --- |
| `sensor_acquisition_mode` | `0644` 读写 | `raw`、`feature` 或 `alternating` |
| `sensor_raw_duration_ms` | `0644` 读写 | 原始采集窗口时长，单位 ms |
| `sensor_feature_duration_ms` | `0644` 读写 | 交替模式下的特征采集窗口时长，单位 ms |
| `sensor_poll_mode` | `0644` 读写 | `fixed` 或 `adaptive` |
| `sensor_poll_fixed_interval_ms` | `0644` 读写 | 固定特征轮询周期配置，单位 ms |
| `sensor_poll_min_interval_ms` | `0644` 读写 | 自适应轮询周期下限，单位 ms |
| `sensor_poll_max_interval_ms` | `0644` 读写 | 自适应轮询周期上限，单位 ms |

这 7 项读取主机配置，不访问传感器寄存器。采集模式含义：

- `raw`：重复执行定长原始采集批次，不轮询特征。
- `feature`：持续轮询特征，不触发原始采集批次。
- `alternating`：从特征窗口开始，在特征和原始采集窗口之间交替。

`fixed` 使用固定周期；`adaptive` 根据特征变化和通信耗时在上下限之间调节周期。
传感器采样率、主机轮询周期、采集窗口是三个独立概念。

### 2.3 主机运行状态

| 属性文件名 | 权限 | 读取结果 / 含义 |
| --- | --- | --- |
| `sensor_online` | `0444` 只读 | `1` 为链路通过连接验证，`0` 为未验证、离线或升级等状态 |
| `sensor_poll_interval_ms` | `0444` 只读 | 当前特征轮询目标周期，单位 ms |
| `sensor_sample_age_ms` | `0444` 只读 | 距离最近完整特征样本的时间，单位 ms；没有有效样本时返回 `-ENODATA` |

这 3 项只读取主机状态，使用短临界区，不等待完整原始窗口或升级事务。
`sensor_online=1` 不保证特征缓存新鲜，也不表示 IIO buffer 已启用。

### 2.4 用户命令控制的真实批次长度

| 属性文件名 | 权限 | 含义 |
| --- | --- | --- |
| `sensor_raw_batch_samples` | `0644` 读写 | 用户请求的真实批次长度，默认 1024；读写驱动配置，下一窗口应用 |
| `sensor_raw_batch_samples_available` | `0444` 只读 | 型号可设置的长度列表；XYZS 为 `1024 2048 4096 8192 16384 32768`，XYZ 暂为 `1024` |
| `sensor_raw_manual` | `0644` 读写 | 默认 0 自动执行 raw；写 1 后，raw 模式只执行用户显式触发的窗口 |
| `sensor_raw_capture` | `0200` 只写 | 仅接受 1；在 raw + manual 模式同步执行一轮原始窗口，错误通过 errno 返回 |

驱动不自动轮换长度。写长度不立即发送串口采集命令，也不截断当前窗口。
触发窗口前需要上线并处于 raw 模式，窗口时长由 `sensor_raw_duration_ms` 决定。
写入 `sensor_raw_capture` 会等待窗口收尾，必须在另一线程/进程持续读取原始 buffer。
触发命令的可重启中断错误统一以 `EINTR` 返回，避免内核自动重新执行写入；
调用方应检查本窗口结果，不要在未确认状态时自动重发。
该窗口返回之后才读取稳定的 `raw_*` 计数、确认数据完整，再设置下一长度。
可设置范围不等于实机稳定性结论，新增长度须逐档上机验证。
具体双传感器脚本见 [训练采集说明](../tests/training/README.md)。

## 3. 特征设备：仅 XYZS 创建的 5 个 IIO_DEVICE_ATTR

以下属性只出现在 `name=ssf_mems_xyzs` 的特征设备目录中。
XYZ 当前型号表不提供相应能力，因此不创建这些文件。

| 属性文件名 | 权限 | 读取结果 / 含义 | 读取来源 |
| --- | --- | --- | --- |
| `sensor_parameter_switch` | `0444` 只读 | 完整参数开关字，以十六进制输出，例如 `0xfc00` | 串口读取 40070 |
| `sensor_feature_enable` | `0644` 读写 | 六个高低频特征计算开关的位掩码，十进制 `0`～`63` | 串口读取 40070，提取 bit15～bit10 并右移 10 位 |
| `sensor_firmware_version` | `0444` 只读 | 固件版本整数编码，例如 `5100`，不自动解释成语义版本号 | 串口读取 40121 |
| `sensor_firmware_update` | `0200` 只写 | 无读取接口；写入固件文件名提交升级操作 | 不支持读取 |
| `sensor_firmware_update_status` | `0444` 只读 | 升级阶段、已确认/总字节数、错误码、版本及通信阻塞标志 | 主机升级状态 |

`sensor_feature_enable` 中 `32/16/8` 分别对应 X/Y/Z 高频加速度有效值计算开关，
`4/2/1` 分别对应 X/Y/Z 低频速度有效值计算开关。写入时保留参数字的其他位。

升级状态的文本格式为：

```text
<阶段> <已确认字节数>/<总字节数> error=<错误码> version=<版本> blocked=<0或1>
```

例如 `sending 1024/4096 error=0 version=-1 blocked=0` 仅用于展示格式。
此属性是升级状态接口；日常查询传感器固件版本应使用 `sensor_firmware_version`。

## 4. 特征设备：两种型号共有的 2 个 IIO_CONST_ATTR

| 属性文件名 | 权限 | 固定输出 | 定义位置 |
| --- | --- | --- | --- |
| `sensor_acquisition_mode_available` | `0444` 只读 | `raw feature alternating` | 采集模式属性定义旁 |
| `sensor_poll_mode_available` | `0444` 只读 | `fixed adaptive` | 轮询模式属性定义旁 |

它们与 `IIO_DEVICE_ATTR` 一样表现为普通 sysfs 文本文件，用 `cat` 读取。
区别在于内容来自宏中给定的固定字符串，不读取设备或运行状态。

并非所有 `_available` 都使用 `IIO_CONST_ATTR`：
`sensor_baudrate_available` 和 `sensor_sampling_rate_index_available` 使用
`IIO_DEVICE_ATTR`，读取时由回调根据当前型号表生成列表。
固定的模式列表也不意味着在任意通信配置下都能成功切换模式；例如有奇偶校验时，
当前驱动不支持包含原始采集的模式。

## 5. 原始数据设备：两种型号共有的 16 个 IIO_DEVICE_ATTR

以下属性在 `ssf_mems_xyz_raw` 和 `ssf_mems_xyzs_raw` 上完全相同，均为
`0444` 只读，读取驱动内部统计，不主动发送串口请求。
原始数据设备没有 `IIO_CONST_ATTR`。

| 属性文件名 | 读取结果 / 含义 |
| --- | --- |
| `raw_active` | 原始接收解析器是否启用，`0/1`；恢复、排空期间也可能为 `1` |
| `raw_packets` | 发布阶段 CRC/尾标记正确的包数；不包含排空包 |
| `raw_sequence_gaps` | 发布阶段包序号异常次数，不是精确丢包数 |
| `raw_crc_errors` | 发布阶段 CRC 或尾标记错误的候选包次数 |
| `raw_buffer_drops` | 单个样本推送到 IIO 失败的次数；buffer 未开启时不计为失败 |
| `raw_sampling_frequency` | 本窗口启动前确认的采样率，单位 Hz；无有效活动/记录时可能返回 `-ENODATA` |
| `raw_drain_packets` | 排空、恢复阶段收到的有效包数；这些包不发布到 IIO |
| `raw_drain_sequence_gaps` | 排空阶段的包序号异常次数 |
| `raw_drain_crc_errors` | 排空阶段坏候选包或收尾残片次数 |
| `raw_start_wait_ms` | 最近一批从触发发送到采集线程结束首次等待的耗时，单位 ms；启动失败也可能记录 |
| `raw_link_mode` | 总线业务模式编号，含义见下表 |
| `raw_publishing` | 是否允许把解析出的样本交给 IIO，`0/1`；不代表用户已开启 buffer |
| `raw_batches` | 当前原始窗口成功完成的批次数，经过收齐、恢复和统计检查后增加 |
| `raw_batch_packets` | 当前或最近一批的有效包数，包含排空包；每批重新计数，正常完成为 `raw_batch_samples / 64` |
| `raw_batch_samples` | 每批同步 XYZ 组数，当前/最近执行窗口的实际长度，默认 `1024` |
| `raw_ignored_bytes` | 寻找包头时忽略的非包头字节数 |

`raw_link_mode` 返回十进制数字：

| 数值 | 含义 |
| --- | --- |
| 0 | 普通 Modbus 通信 |
| 1 | 原始启动命令已发送，等待首包 |
| 2 | 接收原始数据包 |
| 3 | 排空、恢复中，尚未确认恢复普通通信 |
| 4 | 固件升级 |

统计通常在一个原始窗口开始时重置，窗口内多个批次累计；启动前检查失败时可能保留旧值。
每包 64 组 XYZ，每批 `raw_batch_samples / 64` 包。`raw_packets` 即使在 buffer 关闭时也可能增加，
不能据此认定用户态已收到波形。各文件独立读取，不保证多个读数来自同一时刻。

## 6. 用户态如何读取

### 6.1 在目标板上确定设备路径

```sh
for d in /sys/bus/iio/devices/iio:device*; do
    [ -f "$d/name" ] || continue
    printf '%s: ' "$d"
    cat "$d/name"
done
```

按实际输出找到同一个传感器的特征设备和原始设备，再设置变量。
下面的 `X`、`Y` 必须替换成实际编号：

```sh
FEATURE=/sys/bus/iio/devices/iio:deviceX
RAW=/sys/bus/iio/devices/iio:deviceY
```

同型号有多个实例时，`name` 会重复，应结合 `readlink -f "$FEATURE"` 和
`readlink -f "$RAW"` 显示的父设备路径配对；不要只取第一个同名设备。
`name` 表示驱动按设备树选择的型号，不是寄存器自动识别结果。

### 6.2 读取 IIO_DEVICE_ATTR

对所有可读属性，使用 `cat <设备目录>/<表中的属性名>` 即可：

```sh
# 两种型号：主机配置、状态和型号可选值。
cat "$FEATURE/sensor_online"
cat "$FEATURE/sensor_acquisition_mode"
cat "$FEATURE/sensor_sample_age_ms"
cat "$FEATURE/sensor_sampling_rate_index_available"
cat "$FEATURE/sensor_baudrate_available"

# 两种型号：访问传感器读取当前参数，可能等待采集事务。
cat "$FEATURE/sensor_sampling_rate_index"
cat "$FEATURE/sensor_sampling_frequency"
cat "$FEATURE/sensor_sampling_length_index"
cat "$FEATURE/sensor_baudrate"

# 两种型号：原始采集统计。
cat "$RAW/raw_active"
cat "$RAW/raw_link_mode"
cat "$RAW/raw_sampling_frequency"
cat "$RAW/raw_batches"
cat "$RAW/raw_crc_errors"
cat "$RAW/raw_buffer_drops"

# 仅 XYZS：先确认文件存在再读取。
if [ -e "$FEATURE/sensor_firmware_version" ]; then
    cat "$FEATURE/sensor_firmware_version"
    cat "$FEATURE/sensor_parameter_switch"
    cat "$FEATURE/sensor_feature_enable"
    cat "$FEATURE/sensor_firmware_update_status"
fi
```

`sensor_firmware_update` 只有写回调，不能用 `cat` 读取。
其写入会执行升级，不应作为查询方式。

### 6.3 读取 IIO_CONST_ATTR

```sh
cat "$FEATURE/sensor_acquisition_mode_available"
# 输出：raw feature alternating

cat "$FEATURE/sensor_poll_mode_available"
# 输出：fixed adaptive
```

两种宏创建的可读文件都返回一行文本，末尾有换行。
数值属性通常输出十进制，`sensor_parameter_switch` 输出带 `0x` 前缀的十六进制；
采样率属性可能包含小数，模式属性输出字符串。

### 6.4 在 C/C++ 程序中读取

C 使用 `open()` / `read()`，C++ 也可以使用 `std::ifstream` / `std::getline()`
读取相同的 sysfs 路径，再按上表解析整数、小数或字符串。
重复查询时重新打开文件，或先把文件偏移移回 0；持续对同一个已读到末尾的文件描述符
调用 `read()` 不会自动获得下一次属性值。

要区分读取失败与数值 `0`，不要把错误当成离线值或空样本：

| 情况 | 含义 / 处理 |
| --- | --- |
| 文件不存在，`ENOENT` | 检查型号能力、设备路径和目标板实际加载的驱动版本 |
| `ENODATA`，常显示为 `No data available` | 暂无有效样本、参数读取时设备未上线，或尚无有效原始采样率等 |
| `EBUSY` | 升级等事务占用；稍后重试 |
| `ETIMEDOUT` 或其他通信错误 | 串口参数读取失败，不能当作返回值 0 |
| 权限错误 | 检查文件权限；`sensor_firmware_update` 本身没有读取接口 |

读取配置属性可能等待采集锁并访问串口，不建议在前端高频刷新时反复查询全部配置项。
在线状态、样本年龄和统计属性更适合周期性查看。

## 7. 与特征值、波形读取的区别

本文的 `sensor_*`、`raw_*` 是配置、状态或统计属性。
特征数值如 `in_temp_raw`、`in_accel_x_rms_raw` 由 IIO 通道定义生成，
可从特征设备目录用 `cat` 读取；多个文件的读取可能跨越缓存更新。

原始设备没有供 `cat` 获取波形单点的 `in_accel_x_raw` 接口。
波形通过启用扫描通道和 buffer 后读取 `/dev/iio:deviceY` 获得，内容为二进制记录。
读取这些设备属性文件不需要开启 buffer；开启 buffer 也不会自动改变后台采集模式。

完整通道、单位换算、buffer 配置和固件升级操作见 [IIO 属性使用说明](ATTRIBUTES.md)。

## 8. 源码对应关系

- [ssf_mems_iio.c](ssf_mems_iio.c)：搜索 `static IIO_DEVICE_ATTR`、`static IIO_CONST_ATTR` 查看定义。
- 特征属性数组：`ssf_mems_iio_attributes`；型号筛选：`ssf_mems_iio_attr_visible()`。
- 原始属性数组：`ssf_mems_raw_attributes`；读取统计：`ssf_mems_raw_status_show()`。
- 属性注册：`ssf_mems_iio_register_device()`；两个设备的注册入口：`ssf_mems_iio_register()`。
- [sensor/xyz.c](../sensor/xyz.c)、[sensor/xyzs.c](../sensor/xyzs.c)：设备名称、寄存器和能力标志。
- [acquisition/ssf_mems_acquisition.c](../acquisition/ssf_mems_acquisition.c)：配置查询、状态读取和串口事务协调。
