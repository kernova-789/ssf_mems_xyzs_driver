# IIO 映射与使用说明

简体中文 | [English](README.en.md)

两款型号按设备树 `compatible` 选择：XYZS 保留以下原有 29 通道 ABI，XYZ 使用自己的 19 个通道。
XYZ 的 IIO 名称为 `ssf_mems_xyz` / `ssf_mems_xyz_raw`；通道、单位和能力差异见
[sensor/README.md](../sensor/README.md)。下文以 XYZS 的属性和名称举例。

完整中文属性说明、权限、取值范围和操作示例见 [IIO 属性使用说明](ATTRIBUTES.md)。
按 XYZ / XYZS 型号汇总的 `IIO_DEVICE_ATTR`、`IIO_CONST_ATTR` 完整清单及用户态读取方法见
[设备属性宏清单](DEVICE_ATTRIBUTES.md)。

## 通道映射与缓存

`ssf_mems_iio.c` 实现全部 IIO 接口。核心层在 serdev 传输就绪后调用
`ssf_mems_iio_register()` 完成注册。

XYZS 的 40001～40029 特征寄存器块同时支持直接读取和软件缓冲区模式。
加速度使用 `IIO_ACCEL`，速度使用 `IIO_VELOCITY`，有符号温度使用 `IIO_TEMP`。
声音特征是设备定义的 dB 指标，通过带名称的 `IIO_COUNT` 通道提供，不表示经过校准的
Pa 声压。设备特有的无量纲指标和频谱字段使用带名称的 `IIO_INDEX` 或 `IIO_COUNT` 通道。
读取 `*_raw` 只复制最近发布的特征快照，不发起 Modbus 请求。首次快照生成前、重连期间，
或读取失败导致缓存失效后，返回 `-ENODATA`。

协议层在完整读取成功后原子更新特征快照。`ssf_mems_iio_publish_features()` 仅在
缓冲区启用时将样本推入 IIO kfifo。后台采集位于 `acquisition/`，在 IIO 注册后启动。
即使缓冲区关闭，协议读取仍会更新缓存。

从检查缓冲区启用状态、打包扫描记录到推送数据，发布流程始终持有 `indio_dev->mlock`。
IIO 缓冲区重配置也使用同一互斥锁，因此关闭缓冲区、释放活动扫描掩码不会与发布操作并发冲突。
注销 IIO 设备前先停止采集；缓冲区回调不得在持有 `mlock` 时再获取采集互斥锁。

比例回调将统一格式的缓存值转换为 IIO 单位：加速度每计数为 0.0980665 m/s²，
速度每计数为 0.00001 m/s，温度每计数为 10 毫摄氏度，声音指标每计数为 0.01 dB。
寄存器 40026 已按整数百分数存储，40027 按 Hz×10 存储，40028 为固件定义的
无量纲频谱差异累加值乘以 100。

## 波特率与链路状态

`sensor_baudrate` 和 `sensor_baudrate_available` 是设备属性。
读取 `sensor_baudrate` 时，可能执行 `ssf_mems_protocol_get_baudrate()` 中的完整波特率扫描；
写入时接受 `sensor_baudrate_available` 列表中的实际数值。
读写都与采集状态机串行执行。成功写入后也会修改重连目标，后台线程重新验证通信后再恢复轮询。

只读属性 `sensor_online`、`sensor_poll_interval_ms` 和 `sensor_sample_age_ms`
分别提供链路状态、当前目标轮询周期及最近完整样本的年龄。
这三项读取使用独立于采集事务锁的短状态锁，不等待原始窗口或固件升级。
首次成功获取样本前，样本年龄返回 `-ENODATA`。
可通信但返回 Modbus 异常的设备可能仍为在线状态，同时特征缓存已经失效，
因此链路状态和样本年龄需要分别判断。

普通 Modbus 接收器在特征设备上提供两个只读累计统计，XYZ 与 XYZS 均支持：

| 属性 | 含义 |
| --- | --- |
| `sensor_modbus_crc_rejected_candidates` | CRC 校验未通过的候选数 |
| `sensor_modbus_crc_valid_candidates` | CRC 校验通过并交给帧回调的候选数 |

这些是每个接收器实例的 64 位十进制计数；接收器初始化时清零，接收队列 flush 不清零。
读取使用解析锁获取一致值，不发送串口请求。合法响应的数据区也可能形成被拒绝的假候选，
因此拒绝数不等于坏帧数；通过数也不等于成功事务数，仍需结合请求结果、在线状态及 UART 统计判断。
CRC 不匹配的候选继续被拒绝，仅输出调试级日志；真实损坏且没有有效响应的请求仍会超时。
振动和声音批次的 CRC、序号及丢样检查不受影响。

## 运行中修改采集策略

所有调度控制位于特征设备；具体 ABI 见 [属性说明](ATTRIBUTES.md#3-采集模式与窗口)。

| 属性 | 取值 / 默认值 |
| --- | --- |
| `sensor_acquisition_mode` | `raw`、`feature`、`sound`、`alternating`；默认 alternating |
| `sensor_acquisition_mode_available` | 型号实际支持的列表；XYZ 不包含 sound |
| `sensor_feature_duration_ms` | 特征窗口，100..60000 ms，默认 5000 |
| `sensor_raw_duration_ms` | 振动窗口，100..60000 ms，默认 5000 |
| `sensor_sound_duration_ms` | 声音窗口，100..60000 ms，默认 5000；仅支持声音的型号 |
| `sensor_alternating_feature_en` | 0/1，默认 1 |
| `sensor_alternating_raw_en` | 0/1，默认 1 |
| `sensor_alternating_sound_en` | 0/1，默认 1；仅支持声音的型号 |

交替顺序为特征 → 振动 → 声音 → 特征，跳过关闭的路线和未启用 buffer 的振动/声音路线。
窗口到期收完在途完整批次，再释放总线给下一路线。仅选择一条路线时持续重复该路线。
特征轮询仍更新直接读取缓存，不要求开启特征 buffer。打开 `/dev/iio:deviceN` 不会自动
开启 buffer；应用须配置扫描字段、启用并持续消费振动/声音 buffer。
声音 4096 点批次会整体发布，建议声音 buffer 至少 4096 条。

```sh
IIO=/sys/bus/iio/devices/iio:deviceX  # 实际特征设备
# 例如只交替特征和声音；先启用声音设备的 buffer 并启动消费者。
echo 1 > "$IIO/sensor_alternating_feature_en"
echo 1 > "$IIO/sensor_alternating_sound_en"
echo 0 > "$IIO/sensor_alternating_raw_en"
echo 3000 > "$IIO/sensor_feature_duration_ms"
echo 5000 > "$IIO/sensor_sound_duration_ms"
echo alternating > "$IIO/sensor_acquisition_mode"
# 只重复采声音：echo sound > "$IIO/sensor_acquisition_mode"
```

配置读写只取短状态锁，不等待在途串口事务。写入成功表示主机配置已保存；
采集线程在当前完整批次和恢复结束后应用。模式、时长、路线开关实际变化时，
从第一个可运行路线开启新窗口；同值写不重置。最后一条交替路线不能关闭。
多个属性分别生效，重连保留配置，卸载后恢复默认。
旧设备树 `raw-duration-ms=0` 保持 feature 默认；有校验时仅支持 feature 调度。
新调度已做主机回归与目标内核交叉编译，尚未实机验证。

## 特征轮询周期

以下属性位于特征设备 `ssf_mems_xyzs`，控制主机读取特征的间隔，不改变传感器
采样率、波特率或原始数据采样间隔。它们适用于 `feature` 和 `alternating` 的
特征阶段；`raw` 模式保存配置但不执行特征轮询。

| 属性 | 权限 | 默认值/取值 |
| --- | --- | --- |
| `sensor_poll_mode` | 读写 | `fixed`（默认）或 `adaptive` |
| `sensor_poll_mode_available` | 只读 | `fixed adaptive` |
| `sensor_poll_fixed_interval_ms` | 读写 | 1000 ms，范围 100..60000 |
| `sensor_poll_min_interval_ms` | 读写 | 500 ms，范围 100..60000 |
| `sensor_poll_max_interval_ms` | 读写 | 5000 ms，范围 100..60000 |
| `sensor_poll_interval_ms` | 只读 | 当前生效的周期，可能受通信耗时影响 |

以 root 身份操作，X 替换为特征设备编号：

```sh
IIO=/sys/bus/iio/devices/iio:deviceX
# 固定模式：两轮读取开始时间的目标间隔为 2000 ms。
echo 2000 > "$IIO/sensor_poll_fixed_interval_ms"
echo fixed > "$IIO/sensor_poll_mode"

# 自适应模式：在 500～5000 ms 内调整周期。
echo 5000 > "$IIO/sensor_poll_max_interval_ms"
echo 500 > "$IIO/sensor_poll_min_interval_ms"
echo adaptive > "$IIO/sensor_poll_mode"
cat "$IIO/sensor_poll_interval_ms"
```

下限不能超过上限，非法写入返回 `-EINVAL` 并保留旧配置。属性逐项更新，调整范围
时应按当前值选择顺序：提高下限越过旧上限之前先提高上限，降低上限越过旧下限之前
先降低下限。数值只接受十进制整数；上下限相等时周期被限定在该值。

固定周期和自适应上下限分别保存，切换模式不会覆盖另一个模式的配置。进入自适应
时从当前周期开始并限制到上下限；切回固定时使用保存的固定周期。自适应按 32 次
相邻特征比较调整周期；30 秒试探只在本次有变化、窗口重复率低且通信耗时有余量时提速，
不清空未完成统计。交替采集保留学习结果，完全重复数据不会被定时试探反向加速。
只修改非当前模式的参数不会打断当前轮询。

主机轮询配置写入只取短状态锁，不等待在途批次；当前模式的参数变化会唤醒
调度，不重新开始特征窗口。配置在离线、重连后继续保留，本次驱动卸载后不保留。
这些操作不发送传感器配置命令。当前周期是调度目标，读取耗时超过周期时不会并行
发送或补发积压轮次。

## 二进制固件升级

特征设备新增 root 只写的 `sensor_firmware_update` 和只读的
`sensor_firmware_update_status`。前者接受放在开发板 `/lib/firmware/` 下的 `.bin`
简单文件名（不含目录）；后者返回阶段、已应答字节数/总字节数、错误码、重启后版本号和暂停标志。
升级写入为同步操作，状态读取使用独立短锁，不会因升级占用采集锁而等待。
升级时暂停两种采集，成功后恢复；末包结果不确定时暂停自动通信。

**必须先阅读 [镜像地址、总线限制和操作说明](../protocol/firmware/OTA.md)**。
必须使用匹配当前硬件和 Bootloader 的镜像；仓库中的 bin 属于其他传感器，当前没有适用镜像。

## 传感器特征寄存器配置

以下是 XYZS 的特征配置属性。其中 `sensor_sampling_rate_index_available` 从型号表
生成列表，其余属性通过 Modbus 查询传感器，与采集和波特率恢复串行执行；设备离线时
返回 `-ENODATA`。XYZ 不创建参数开关、特征使能和固件版本属性。

- `sensor_sampling_rate_index`（读写）：寄存器 40052，编码范围为 0～9。
- `sensor_sampling_frequency`（只读）：当前配置的采样率，单位 Hz。
- `sensor_sampling_rate_index_available`（只读）：支持的采样率编码列表。
- `sensor_sampling_length_index`（读写）：寄存器 40053，普通采样长度编码 0～5；
  不用于设置定长原始采集的批次长度。
- `sensor_parameter_switch`（只读）：寄存器 40070，以十六进制输出完整参数开关字。
- `sensor_feature_enable`（读写）：提取 40070 的 bit15～bit10，右移后输出 0～63。
  bit5/4/3 分别控制 X/Y/Z 高频加速度有效值计算，bit2/1/0 分别控制 X/Y/Z 低频速度有效值计算。
  `63` 开启全部六项；写入时保留低十位，包括报警配置。
- `sensor_firmware_version`（只读）：寄存器 40121，固件版本整数编码，例如 `5100`。

采样率编码依次对应 `533.34 888.9 1066.68 1333.35 2666.7 2963 5333.4 8889 13333.5 26667` Hz。
这些可写寄存器配置使用功能码 `0x06`，同值跳过写入，写后读回验证。
首次连接选择型号表中的目标波特率和采样率：当前两款均为 128000 bit/s，
XYZS 选择索引 6（5333.4 Hz），XYZ 选择索引 3（1333.35 Hz）。
`current-speed` 只提供初始匹配提示；配置一致时不重复写入，配置失败进入连接重试流程。
手动成功修改的波特率和采样率在本次驱动运行期间作为重连目标，卸载后不保留。

避免频繁切换持久化设备配置。驱动不会为采样率或特征使能额外发送保存命令，
上电行为需在目标设备上确认。读回只能确认寄存器值，新的计算结果可能要等待数秒。
配置后轮询会清空比较历史，但保留已配置或已学习的周期；旧缓存失效，直到下一次寄存器
读取成功，而这次读取仍可能包含传感器的上一轮计算结果。

为保持兼容，保留 `in_count6_startup_flags_raw` 名称，表示 XYZ 运行判定标志。
bit0/1/2 分别对应 X/Y/Z，值为 0 不表示测量被禁用。
实机排查方法见 [特征值长时间重复的排查](../tools/README.md#特征值长时间重复的排查)。

## 离线验证

运行 `python3 tests/iio/run_tests.py`，使用 pthread 对实际发布代码进行压力测试，
并启用 AddressSanitizer / UndefinedBehaviorSanitizer。测试在反复关闭缓冲区、释放及替换
扫描掩码的同时，检查指定掩码下的记录打包、有符号温度、运行标志、对齐填充和时间戳。
测试使用 IIO 模拟层，真实内核缓冲区操作仍需实机验证。

## 原始数据设备

新增 `ssf_mems_xyzs_raw`，提供 XYZ、包号、包内序号及主机接收时间戳的独立
buffer，保持特征设备原有布局不变。全部数据字段和时间戳启用时为 24 字节；
只启用全部数据字段、不启用时间戳时为 16 字节。
配置与读取示例见[定长原始采集说明](../protocol/raw/RAW_CAPTURE.md)。

成功设置的采样率会在本次驱动运行期间作为重连目标，卸载后恢复型号表默认值。
主机模式、窗口与轮询配置读写均使用短锁，不等待原始窗口或 OTA。
特征字段/单位统一来自 `sensor/sensor_fields.h`，两设备共用缓冲区打包和注册辅助函数。

## 用户命令设置定长长度

`sensor_raw_batch_samples` 读写当前请求长度，`sensor_raw_batch_samples_available` 列出可设置长度。
驱动不自动切换长度。双传感器轮换脚本与显式窗口控制见 [训练采集说明](../tests/training/README.md)，
完整属性定义见 [设备属性宏清单](DEVICE_ATTRIBUTES.md)。

## 4096 点原始声音

XYZS 新增一次性声音触发和独立 IIO buffer，详见 [声音采集](SOUND_CAPTURE.md)。

## 配置生效与同步采集状态

特征设备的只读 `sensor_acquisition_status` 返回一个一致快照，例如：

```text
connection=ready route=raw busy=1 manual_active=1 manual_pending=1 idle_reason=none requested_mode=feature applied_mode=raw config_version=8 applied_config_version=6 config_pending=1
```

- `connection`：startup / matching / switching / configuring / verifying / ready / backoff / stopped。
- `route`：实际采集路线 feature / raw / sound / none；`busy=1` 表示正在执行采集事务。
- `manual_active`：当前事务是否来自手动请求；`manual_pending`：是否存在排队或未完成的手动请求。
- `idle_reason`：none / interval / buffer_disabled / manual_wait / firmware_blocked 或连接阶段原因。
- `requested_mode` 为请求模式，`applied_mode` 为工作线程已采用的模式（首次应用前为 none）。
- `config_version` 为请求配置版本，`applied_config_version` 为已采用版本；不同则 `config_pending=1`。
  相等不代表已采到新数据，仍应检查 busy、路线和数据本身。

模式属性仍读回请求配置。轮询周期属性 `sensor_poll_interval_ms` 反映当前生效周期；
轮询配置写入也在工作线程边界应用，不再由写入线程改变生效统计。
显式手动振动窗口保留提交时的配置，新配置等待该窗口完成后应用。

`sensor_raw_capture` / `sensor_sound_capture` 保持同步写入语义，由后台线程执行。
每条路线同时最多一个请求，重复提交返回 EBUSY；声音可插入手动振动窗口的批次间隙，
不改变后台模式和窗口截止时间。信号取消在途请求时先安全收尾再返回 EINTR，应用不可自动重放。
有同步采集请求期间，波特率查询/修改、传感器参数写入及固件升级返回 EBUSY。

振动/声音批次若中途关闭或重启 Buffer，或未选择实际数据通道，会记录交付失败，不能当作成功批次。
即使 Buffer 在两次发布之间关闭后立即重启，驱动也会通过代次检查识别；Buffer 开启不保证应用持续消费。
