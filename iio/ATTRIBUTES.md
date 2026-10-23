本文原有操作示例使用 XYZS 名称。驱动现在还支持 XYZ V2.4：

| 型号 | 特征设备 / 原始设备 | 特征与能力差异 |
| --- | --- | --- |
| XYZS | `ssf_mems_xyzs` / `ssf_mems_xyzs_raw` | 原有 29 个特征、参数开关、版本和 OTA |
| XYZ | `ssf_mems_xyz` / `ssf_mems_xyz_raw` | 19 个特征，含三轴位移；无声音、高低频、运行标志、参数开关、版本和 OTA 属性 |

XYZ 位移通道为 `in_distance_x_peak_to_peak_raw`（Y/Z 同理），scale 为 `0.000001` m；
温度 raw 已从手册的有符号 ℃×10 换算为 ℃×100，scale 同样为 10 m℃。
型号布局和定长采集兼容假设见 [sensor/README.md](../sensor/README.md)。

# IIO 属性使用说明

本文对应当前 SSF-MEMS-XYZS 温振声驱动。属性名以 `ssf_mems_iio.c` 为准。
`只读`表示用 `cat` 查看，`读写`表示还可以用 `echo` 设置，`只写`表示只能提交操作。
写入通常需要 root 权限；下文示例在 root shell 中执行。

## 1. 找到两个 IIO 设备

同一个传感器注册两个设备，编号由内核分配，不能假定总是 0 和 1。

```sh
for d in /sys/bus/iio/devices/iio:device*; do
    echo "$d: $(cat "$d/name")"
done
```

| `name` 的值 | 用途 | 属性前缀 |
| --- | --- | --- |
| `ssf_mems_xyzs` | 查看特征值、修改采集策略和传感器参数、升级固件 | `sensor_*` 和特征通道属性 |
| `ssf_mems_xyzs_raw` | 从缓冲区读取定长 XYZ 波形，查看原始采集统计 | `raw_*` 和原始通道属性 |

根据实际编号设置路径，后面的示例都使用这两个变量：

```sh
FEATURE=/sys/bus/iio/devices/iio:deviceX
RAW=/sys/bus/iio/devices/iio:deviceY
```

## 2. 先分清三个时间设置

| 设置 | 控制什么 | 例子 |
| --- | --- | --- |
| 传感器采样率 `sensor_sampling_rate_index` | 传感器每秒采多少点，影响波形采样间隔和特征计算 | 索引 6 对应 5333.4 Hz |
| 特征轮询周期 `sensor_poll_*` | 主机多久读取一次传感器已经计算好的特征 | 1000 ms 约每秒读取一次 |
| 采集窗口 `sensor_raw_duration_ms`、`sensor_feature_duration_ms` | 主机一轮原始采集或特征采集持续多久 | 交替模式下原始 5 秒、特征 5 秒 |

修改主机周期不会修改传感器采样率。开启 IIO buffer 也不会自动改变采集模式。

## 3. 采集模式与窗口

以下属性位于特征设备。

| 属性 | 权限 | 用途、取值及用法 |
| --- | --- | --- |
| `sensor_acquisition_mode` | 读写 | 选择 `raw`、`feature`、`alternating`。例如 `echo feature > "$FEATURE/sensor_acquisition_mode"`。默认通常为 `alternating`。 |
| `sensor_acquisition_mode_available` | 只读 | 列出可写的模式名称：`raw feature alternating`。 |
| `sensor_raw_duration_ms` | 读写 | 每轮原始采集窗口，100～60000 ms，默认 5000。窗口截止后仍收完在途批次。例如写入 `2000`。 |
| `sensor_feature_duration_ms` | 读写 | 交替模式的一轮特征窗口，100～60000 ms，默认 5000。例如写入 `3000`。纯特征模式不按此窗口停止。 |

- `raw`：重复触发完整的定长批次，每批 1024 组 XYZ；不轮询特征。批间有通信确认和等待，并非无间断波形。
- `feature`：一直轮询特征，不触发原始批次。
- `alternating`：先执行特征窗口，再执行原始窗口，然后重复。

```sh
# 原始约 2 秒、特征约 3 秒，交替执行。
echo 2000 > "$FEATURE/sensor_raw_duration_ms"
echo 3000 > "$FEATURE/sensor_feature_duration_ms"
echo alternating > "$FEATURE/sensor_acquisition_mode"
```

模式和窗口在驱动内存中保存，重连后继续使用，卸载后恢复设备树/默认值。
有奇偶校验时默认仅特征模式，选择含原始采集的模式会返回“不支持”。
旧设备树 `sange-cbm,raw-duration-ms=0` 也会选择仅特征模式。
运行时不接受窗口为 0，请直接选择模式。

读取主机模式、窗口和轮询配置使用短临界区，不等待完整原始窗口或 OTA 事务。
写入配置仍可能等待当前原始窗口和在途批次结束；时长不是严格的实时截止时间。
模式或窗口实际变化后，交替模式重新从特征窗口开始；写入相同值不重启窗口。
多个属性分别生效，不构成一次原子设置。

## 4. 固定轮询与自适应轮询

以下属性位于特征设备，只影响 `feature` 模式和 `alternating` 的特征阶段。
在 `raw` 模式中可保存这些配置，切回特征采集后使用。

| 属性 | 权限 | 用途、范围与默认值 |
| --- | --- | --- |
| `sensor_poll_mode` | 读写 | `fixed` 为固定周期，`adaptive` 为自动调节周期。默认 `fixed`。 |
| `sensor_poll_mode_available` | 只读 | 列出支持的轮询模式：`fixed adaptive`。 |
| `sensor_poll_fixed_interval_ms` | 读写 | 固定模式的目标周期，100～60000 ms，默认 1000。 |
| `sensor_poll_min_interval_ms` | 读写 | 自适应周期下限，100～60000 ms，默认 500。 |
| `sensor_poll_max_interval_ms` | 读写 | 自适应周期上限，100～60000 ms，默认 5000。 |
| `sensor_poll_interval_ms` | 只读 | 当前生效的目标周期。固定模式等于固定周期，自适应模式会在上下限内变化。 |

```sh
# 固定为 2 秒。
echo 2000 > "$FEATURE/sensor_poll_fixed_interval_ms"
echo fixed > "$FEATURE/sensor_poll_mode"

# 自适应范围为 500～5000 ms。
echo 5000 > "$FEATURE/sensor_poll_max_interval_ms"
echo 500 > "$FEATURE/sensor_poll_min_interval_ms"
echo adaptive > "$FEATURE/sensor_poll_mode"
cat "$FEATURE/sensor_poll_interval_ms"
```

上下限必须满足“下限 ≤ 上限”，相等时锁定为同一个周期。
提高下限超过旧上限前，应先提高上限；降低上限低于旧下限前，应先降低下限。
非法写入不改变原配置。这里的周期和窗口数值只接受十进制整数。

固定周期与自适应上下限分别保存，切换模式不会覆盖另一组设置。
进入自适应时，将当前周期限制在新范围内作为起点。
修改当前未使用模式的参数只保存配置；修改生效参数会清空旧比较统计并唤醒轮询。
轮询设置不会重开交替模式的特征窗口，也不会截断在途原始批次。

自适应算法比较相邻两份完整特征，每积累 32 次比较进行一次判断：

| 条件 | 调整方式 |
| --- | --- |
| 完全相同的比较超过 50% | 周期增加约 20%，减少重复读取 |
| 完全相同的比较少于 10%，且最慢读取耗时不超过周期的一半 | 周期减少约 5%，更及时读取变化 |
| 其他情况 | 保持当前周期 |

统计窗口未满时，约每 30 秒可试探减少约 5%，但必须同时满足：本次特征确有变化、
当前窗口重复率低于 10%、读取耗时有余量。完全重复的数据不会因定时试探反而提速。
所有调整都受上下限约束。
这里判断的是“整份特征是否完全相同”，不是振动幅度是否超过某个阈值。
算法不能确认传感器内部何时生成了新特征。固定模式不执行这些比较。
原始窗口正常收尾后保留已学习的周期；重连会清空比较历史，但保留配置和当前周期。

周期按相邻读取的开始时间计算，扣除已经用于读取的时间，并保留串口帧间隔。
通信耗时过长、调度延迟或原始窗口都会使实际间隔变长；不会补发积压轮次。

## 5. 传感器参数

以下属性位于特征设备。除 `_available` 外，读取会访问串口，写入也会读回验证。
设备未上线时通常返回“无数据”，升级占用时可能返回“设备忙”。

| 属性 | 权限 | 用途及用法 |
| --- | --- | --- |
| `sensor_baudrate` | 读写 | 串口速率，单位 bit/s，例如 `echo 128000 > "$FEATURE/sensor_baudrate"`。读取会确认当前可通信速率，必要时扫描其他速率。 |
| `sensor_baudrate_available` | 只读 | 列出型号表支持的 bit/s 数值；设置时使用数值，不使用寄存器编码。 |
| `sensor_sampling_rate_index` | 读写 | 传感器采样率编码，当前为 0～9。例如写 `6` 选择 5333.4 Hz。首次连接默认索引 6；成功设置后，重连继续使用用户值，卸载后恢复型号表默认值。 |
| `sensor_sampling_rate_index_available` | 只读 | 列出可用采样率编码，当前为 `0 1 2 3 4 5 6 7 8 9`。 |
| `sensor_sampling_frequency` | 只读 | 将当前采样率编码换算成 Hz 文本，例如 `5333.4`。 |
| `sensor_sampling_length_index` | 读写 | 普通采样长度编码：0～5 对应 1024、2048、4096、8192、16384、32768 点；写后读回确认，同值不写。它不代表驱动原始定长批次的长度，后者通过 40061/40062 独立设置，默认 1024，可用 `sensor_raw_batch_samples` 更改。 |
| `sensor_parameter_switch` | 只读 | 查看完整参数开关字，以十六进制输出，例如 `0xfc00`。包含特征使能和报警等配置位。 |
| `sensor_feature_enable` | 读写 | 查看/修改六个高低频特征计算开关，范围 0～63。例如 `echo 63 > "$FEATURE/sensor_feature_enable"` 开启全部六项，`0` 关闭这六项。保留其他配置位。 |
| `sensor_firmware_version` | 只读 | 读取固件版本编码，原样输出整数，例如 `5100`；不将其解释为语义版本号。 |

当前采样率编码：

| 编码 | Hz | 编码 | Hz |
| --- | --- | --- | --- |
| 0 | 533.34 | 5 | 2963 |
| 1 | 888.9 | 6 | 5333.4 |
| 2 | 1066.68 | 7 | 8889 |
| 3 | 1333.35 | 8 | 13333.5 |
| 4 | 2666.7 | 9 | 26667 |

当前型号的配置和定长原始采集均允许索引 0～9；索引 9 对应 26667 Hz。
范围按当前型号手册配置，本次仅完成离线回归和内核编译，新增上限尚待实机验证。
采样率并不等于 UART 吞吐率，不能据此认为串口能无间断输出同速波形。

当前 `sensor_feature_enable` 的位含义如下，按位或组合：

| 数值 | 功能 | 对应完整开关字的位 |
| --- | --- | --- |
| 32 | X 轴高频加速度有效值 | bit15 |
| 16 | Y 轴高频加速度有效值 | bit14 |
| 8 | Z 轴高频加速度有效值 | bit13 |
| 4 | X 轴低频速度有效值 | bit12 |
| 2 | Y 轴低频速度有效值 | bit11 |
| 1 | Z 轴低频速度有效值 | bit10 |

例如 `56 = 32 + 16 + 8` 开启三个高频项，关闭三个低频项。
关闭这些计算项不会关闭整个传感器采集，也不会改变主机采集模式。

修改采样率/特征开关时，相同值不重复写入；驱动不额外发送保存重启命令。
实际何时应用和是否掉电保存由传感器固件决定。
修改为不同波特率会保存并重启传感器，至少包含 8 秒重启等待及后续验证，命令可能等待更久。
写入当前波特率会先只读确认；通信速率和读回配置都一致时不保存、不重启。
若寄存器存在不同的暂存值，仍会执行保存和验证，不能只凭主机缓存跳过。
成功设置波特率后，新的值同时成为本次驱动运行期间的重连目标；下次加载恢复型号表的默认工作目标，当前为 128000。

## 6. 链路状态与样本年龄

以下属性位于特征设备，只读取主机状态，不发送串口请求。

| 属性 | 权限 | 用途及读法 |
| --- | --- | --- |
| `sensor_online` | 只读 | `1` 表示链路已通过连接验证，`0` 表示未验证、离线或正在升级等状态。原始采集期间也可能为 `1`。 |
| `sensor_poll_interval_ms` | 只读 | 当前特征轮询目标周期，含义见第 4 节。 |
| `sensor_sample_age_ms` | 只读 | 距离最近成功读取完整特征的毫秒数。尚无样本、进入原始采集或配置使旧样本失效时返回“无数据”。 |

这三项使用短临界区读取，不等待完整原始采集窗口或升级事务。
`online=1` 不保证特征缓存一定新鲜，也不表示已启用 IIO buffer。
离线后样本年龄可能继续反映最后一份特征的时间，缓存本身已失效；请结合在线状态判断。

## 7. 特征通道的 `_raw` 与 `_scale`

以下属性位于特征设备，都是只读。`_raw` 读取最近一次完整特征缓存，不主动访问串口。
刚加载、原始采集、重连或缓存失效期间可能返回“无数据”。
多个 `cat` 可能读到不同轮次；需要整份同一轮快照时应使用特征 buffer。

表中 `{x,y,z}` 表示三个独立通道，例如 `in_accel_x_rms_raw`。
对注明有比例的通道，将表中名称末尾的 `_raw` 换成 `_scale` 即可查看比例。
物理值 = raw × scale；温度的 IIO 单位为毫摄氏度。

| `_raw` 属性 | 表示什么 | scale / 换算后的单位 |
| --- | --- | --- |
| `in_accel_{x,y,z}_high_freq_rms_raw` | 高频加速度有效值 | 0.098066500，m/s² |
| `in_velocity_{x,y,z}_low_freq_rms_raw` | 低频速度有效值 | 0.000010，m/s |
| `in_temp_raw` | 有符号温度 | 10，毫摄氏度；摄氏度 = raw / 100 |
| `in_accel_{x,y,z}_peak_to_peak_raw` | 加速度峰峰值 | 0.098066500，m/s² |
| `in_accel_{x,y,z}_peak_raw` | 加速度峰值 | 0.098066500，m/s² |
| `in_accel_{x,y,z}_rms_raw` | 加速度有效值 | 0.098066500，m/s² |
| `in_index_{x,y,z}_kurtosis_raw` | 峭度指标 | 0.010000，无量纲 |
| `in_velocity_{x,y,z}_rms_raw` | 速度有效值 | 0.000010，m/s |
| `in_count0_sound_rms_db_raw` | 声音有效值 | 0.010000，dB |
| `in_count1_sound_peak_db_raw` | 声音峰值 | 0.010000，dB |
| `in_count2_sound_peak_to_peak_db_raw` | 声音峰峰值 | 0.010000，dB |
| `in_count3_zero_crossing_rate_percent_raw` | 过零率 | 1，百分数；值 20 表示 20% |
| `in_count4_spectral_centroid_hz_raw` | 频谱重心 | 0.100000，Hz |
| `in_count5_spectral_flux_raw` | 频谱变化指标 | 0.010000，无标准物理单位 |
| `in_count6_startup_flags_raw` | XYZ 运行判定位掩码 | 无 `_scale`；bit0/1/2 对应 X/Y/Z |

`startup_flags` 表示 XYZ 运行判定，值为 0 不表示主机禁用了采集。
声学特征按设备定义的 dB 指标解释，不直接转换为声压 Pa。

```sh
cat "$FEATURE/in_accel_x_rms_raw"
cat "$FEATURE/in_accel_x_rms_scale"
cat "$FEATURE/in_temp_raw"
cat "$FEATURE/in_count6_startup_flags_raw"
```

## 8. 原始采集统计

以下属性位于原始设备，全部只读，不主动发串口请求。
统计通常在一个原始采集窗口开始时清零，持续多个定长批次的窗口会累计。
如果启动前的参数检查失败，仍可能保留上一窗口的统计。
各文件独立读取，不保证来自同一时刻。

| 属性 | 表示什么 | 如何判断 |
| --- | --- | --- |
| `raw_active` | 原始接收解析器是否启用，0/1 | 恢复/排空阶段也可为 1，不等同于传感器正在采样。 |
| `raw_publishing` | 是否允许将解析的样本交给 IIO，0/1 | 为 1 也不代表用户开启了 buffer。 |
| `raw_link_mode` | 当前总线业务模式 | 0=普通 Modbus，1=等待首包，2=接收原始包，3=排空和恢复，4=固件升级。 |
| `raw_sampling_frequency` | 本窗口启动前确认的采样率，单位 Hz | 无本轮活动且没有有效包时返回“无数据”；不重新读取采样率寄存器。 |
| `raw_packets` | 发布阶段 CRC/尾标记正确的包数 | 一批正常为 16；buffer 关闭时仍会计数，因此不等于用户已读到的包数。 |
| `raw_sequence_gaps` | 发布阶段的包序号异常次数 | 正常为 0；表示异常发生次数，不是精确丢包数。 |
| `raw_crc_errors` | 发布阶段 CRC 或尾标记错误的候选包次数 | 正常为 0；重同步可能对同一段损坏数据产生多个候选。 |
| `raw_buffer_drops` | 单个样本推送到 IIO 失败的次数 | 正常为 0；buffer 未启用时不计为丢弃。 |
| `raw_drain_packets` | 排空/恢复阶段收到的有效包数 | 这些包被解析但不发布，用于安全等待批次结束。 |
| `raw_drain_sequence_gaps` | 排空阶段的包序号异常次数 | 正常为 0。 |
| `raw_drain_crc_errors` | 排空阶段的坏候选包或收尾残片次数 | 正常为 0。 |
| `raw_start_wait_ms` | 最近一批从触发发送到线程确认首包的等待时间 | 启动后首包到达前可能有较长等待；启动失败也会记录等待。 |
| `raw_batches` | 当前窗口成功完成的批次数 | 只有批次收齐、恢复成功且统计检查通过才增加。 |
| `raw_batch_packets` | 当前或最近一批的有效包数，包含排空包 | 正常结束为 16，每批重新计数。 |
| `raw_batch_samples` | 每批的 XYZ 组数 | 当前/最近窗口的真实长度，默认 1024。 |
| `raw_ignored_bytes` | 寻找包头时忽略的非包头字节数 | 正常为 0；非零会使本轮完整性检查失败。 |

当前一包有 64 组 XYZ，一批有 `raw_batch_samples / 64` 包；包号每批从 0 开始，包内样本号为 0～63。
成功批次应无序号、CRC、忽略字节和推送错误。请同时检查排空统计。
这些计数无法替代用户态对缓冲区记录数、包号和样本号的完整性检查。
原始 XYZ 顺序与传感器手册一致。

```sh
cat "$RAW/raw_link_mode"
cat "$RAW/raw_batches"
cat "$RAW/raw_packets"
cat "$RAW/raw_sequence_gaps"
cat "$RAW/raw_crc_errors"
cat "$RAW/raw_buffer_drops"
```

## 9. IIO 缓冲区与扫描属性

两种设备都有以下内核提供的属性。修改扫描通道和长度前先关闭 buffer。

| 路径 | 权限 | 用途 |
| --- | --- | --- |
| `name` | 只读 | 设备名称，用于辨别特征与原始设备。 |
| `buffer/enable` | 读写 | `1` 开启缓冲区，`0` 关闭；不改变后台采集模式。 |
| `buffer/length` | 读写 | 缓冲区容量，单位为扫描记录数，不是字节。 |
| `buffer/watermark` | 读写 | 普通阻塞读取/轮询期望的可用记录阈值，应不超过容量。 |
| `buffer/data_available` | 只读 | 当前可以读取的记录数。 |
| `scan_elements/<通道>_en` | 读写 | `1` 将该通道加入记录，`0` 排除。 |
| `scan_elements/<通道>_index` | 只读 | 通道在扫描记录中的排序编号。 |
| `scan_elements/<通道>_type` | 只读 | 字节序、有无符号、有效位数、存储位数和移位方式。 |

读取 `/dev/iio:deviceX` 或 `/dev/iio:deviceY` 得到二进制记录，不是文本。
按 `_index` 排序，并按 `_type` 的存储宽度对齐各字段；时间戳按 8 字节对齐。
记录末尾还需按本条记录中最大字段的宽度补齐，补齐字节不属于任何通道。
记录布局随已启用通道变化，不能直接把整个文件当成紧密排列的 XYZ 16 位数组。

特征设备一条记录包含一轮特征，全部 29 个特征开启时不带时间戳为 58 字节，带时间戳为 72 字节。
原始设备全部五个数据字段开启时不带时间戳为 16 字节，带时间戳为 24 字节。
原始设备一条记录包含一组 XYZ，可选的扫描通道如下：

| 扫描通道 | index | 类型/含义 |
| --- | --- | --- |
| `in_accel_x`、`in_accel_y`、`in_accel_z` | 0、1、2 | 有符号 16 位 ADC 计数，满量程 ±16 g |
| `in_count0_packet_sequence` | 3 | 无符号 32 位，本批包号 |
| `in_count1_sample_index` | 4 | 无符号 16 位，包内样本号 |
| `in_timestamp` | 5 | 有符号 64 位，主机发布时的纳秒时间戳 |

原始加速度有只读 `in_accel_{x,y,z}_scale`，当前为约 0.004788403 m/s²/计数。
原始设备没有通过 `cat in_accel_x_raw` 获取单点的接口，应读取 buffer。
原始时间戳表示主机收到并发布样本的时间，不是传感器逐点采样时刻；一包会集中发布。
分析波形采样间隔时应结合采样率和包内索引，并处理批间空隙。

以下示例配置全部原始扫描通道并保存约 3 秒的二进制数据：

```sh
echo feature > "$FEATURE/sensor_acquisition_mode"
echo 0 > "$RAW/buffer/enable"
for f in "$RAW"/scan_elements/*_en; do echo 0 > "$f"; done
for c in in_accel_x in_accel_y in_accel_z in_count0_packet_sequence in_count1_sample_index in_timestamp; do
    echo 1 > "$RAW/scan_elements/${c}_en"
done
echo 4096 > "$RAW/buffer/length"
echo 1 > "$RAW/buffer/enable"
echo raw > "$FEATURE/sensor_acquisition_mode"
timeout 3 cat "/dev/${RAW##*/}" > raw_capture.bin
echo feature > "$FEATURE/sensor_acquisition_mode"
echo 0 > "$RAW/buffer/enable"
```

全部六项开启时，当前原始记录为 24 字节，包含字段间和时间戳前的对齐填充。
`timeout` 到时可能切在批次中间，所以示例文件不保证包含完整的起止批次。
更完整的采集和检查工具见 [硬件测试说明](../tests/hardware/README.md)。

## 10. 固件升级

以下属性位于特征设备。

| 属性 | 权限 | 用途及用法 |
| --- | --- | --- |
| `sensor_firmware_update` | 只写 | 写入 firmware 文件名，加载 `.bin` 并执行升级；写入调用持续到升级成功或失败。 |
| `sensor_firmware_update_status` | 只读 | 查看升级阶段、已确认字节数、总字节数、错误码、版本和是否暂停自动通信。升级期间可以另开终端读取。 |

先将适用于当前传感器 Bootloader 的镜像放到系统 firmware 搜索目录，例如
`/lib/firmware/update.bin`，再写简单文件名。只接受字母、数字、点、下划线、
连字符，必须以 `.bin` 结尾，最长 127 字节，不接受目录或开头的点：

```sh
echo update.bin > "$FEATURE/sensor_firmware_update"
# 可在另一终端查看。
cat "$FEATURE/sensor_firmware_update_status"
```

状态示例：`sending 1024/4096 error=0 version=-1 blocked=0`。
`1024/4096` 表示已获得应答的原始文件字节数/文件总字节数，`version=-1` 表示尚未读到版本。
阶段依次可能为 `idle`、`loading`、`waiting`、`checking`、`sending`、`rebooting`、
`verifying`、`complete`；异常时为 `failed` 或 `uncertain`。
`complete` 表示数据包应答和重启后通信确认成功，不是整镜像读回校验。
`uncertain` 表示末包可能已生效但结果无法确认；`blocked=1` 时后台暂停自动通信。

驱动会检查镜像大小和启动向量，当前应用起点要求为 `0x08020000`。
目前没有已确认适用于此传感器的升级镜像，仓库中的 `.bin` 不属于此传感器。
示例文件名仅演示接口，不表示存在可用镜像；升级测试使用合成数据，不读取仓库 bin。
具体协议和测试说明见 [OTA 测试说明](../tests/ota/README.md)。

## 11. 常见返回错误

| 用户态表现 / errno | 常见原因 |
| --- | --- |
| 无数据 / `ENODATA` | 尚未上线、特征缓存无效、没有可用的样本年龄或原始采样率 |
| 无效参数 / `EINVAL` | 模式字符串错误、周期越界、上下限颠倒、采样率或使能掩码越界 |
| 不支持 / `EOPNOTSUPP` | 含校验位的原始采集，或当前型号不支持的配置 |
| 设备忙 / `EBUSY` | 升级或其他独占事务占用，或总线尚未确认恢复普通通信 |
| 超时 / `ETIMEDOUT` | 传感器无应答、首包未到或批后恢复未确认 |
| 输入输出错误 / `EIO` | 串口短写、写后读回不符等 |
| 无此文件 / `ENOENT` | 升级文件不存在，或当前寄存器表缺少所请求的 `.name` |

上述说明对应当前型号和固件。寄存器地址、编码和单位映射集中在
[sensor 型号描述](../sensor/README.md)；将来更换型号表时，应同步核对设备能力和文档。

## 12. 单位与测量边界

sysfs 的 `_raw`、`_scale` 只输出数值，不附加单位字符串；buffer 输出二进制记录，
也不携带单位。用户态根据通道类型、scale 和本文件的通道定义解释数据。
当前驱动没有 offset，物理值按 `raw × scale` 计算，buffer 计数也使用同样的换算。

标准物理量遵循 [Linux IIO ABI](https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-bus-iio)：
加速度为 m/s²、速度为 m/s、温度为毫摄氏度。声音、过零率、频谱指标等自定义通道
按第 7 节分别解释为 dB、百分数、Hz 或无标准物理单位的指标。

不要把特征加速度的比例与原始加速度的比例混用：前者为 0.0980665 m/s²/计数，
后者当前为 0.004788403 m/s²/计数。原始比例来自型号描述。
温度 `raw=2500` 时，乘 scale 10 得到 25000 毫摄氏度，即 25℃。

温度使用有符号 16 位编码，低温行为仍需目标设备验证。
频谱质心采用无符号 16 位的 Hz×10 编码，可表示 0～6553.5 Hz。
寄存器编码限制了可表达的范围和精度；驱动不能从已丢失的信息还原更精细的结果。

真实批次长度的查询、可选列表及显式窗口触发接口见 [设备属性宏清单](DEVICE_ATTRIBUTES.md#24-用户命令控制的真实批次长度)。长度轮换由 [用户态脚本](../tests/training/README.md) 实现。
