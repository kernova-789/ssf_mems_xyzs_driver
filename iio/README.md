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

在特征设备 `ssf_mems_xyzs` 下增加以下主机调度属性，不改变扫描记录布局：

| 属性 | 取值 |
| --- | --- |
| `sensor_acquisition_mode`（读写） | `raw`、`feature`、`alternating` |
| `sensor_acquisition_mode_available`（只读） | 支持的三个模式名称 |
| `sensor_raw_duration_ms`（读写） | 原始窗口，十进制 100..60000 ms |
| `sensor_feature_duration_ms`（读写） | 特征窗口，十进制 100..60000 ms |

`raw` 持续重复完整定长批次（默认 1024 点），不读取特征。
它仍会执行必要的配置读取和批后通信确认，因此不是无间断原始波形。
`feature` 持续轮询特征，不触发原始批次。`alternating` 从特征窗口开始交替，
到原始窗口截止时收完在途批次，不为了满足时长截断数据。特征轮询默认固定为
1000 ms，也可使用下面的独立轮询配置。

默认是交替模式、原始/特征各 5000 ms；设备树两项时段继续提供初始值。
旧 `sange-cbm,raw-duration-ms=0` 仍选择仅特征，但原始窗口读回为可供以后切换的
5000 ms。运行时写时长不接受 0，请用模式选择关闭某类采集。有校验位时默认仅特征，
请求含原始采集的模式返回 `-EOPNOTSUPP`，因为当前私有格式只支持无校验。

以 root 身份操作，先将路径中的 X 换为名字是 `ssf_mems_xyzs` 的设备编号：

```sh
IIO=/sys/bus/iio/devices/iio:deviceX
cat "$IIO/sensor_acquisition_mode_available"
echo raw > "$IIO/sensor_acquisition_mode"          # 持续定长采集
echo feature > "$IIO/sensor_acquisition_mode"      # 持续特征采集
echo 2000 > "$IIO/sensor_raw_duration_ms"
echo 3000 > "$IIO/sensor_feature_duration_ms"
echo alternating > "$IIO/sensor_acquisition_mode"  # 原始约 2 s / 特征约 3 s
```

这些属性只读写主机状态，不发送保存命令、不重启、不改变波特率或采样率。
主机配置读取使用短状态锁，不等待串口事务；配置写入与采集共用 `acquisition.lock`，
可能等待当前原始窗口（最长配置 60 s 加
在途批次/恢复等待）收尾，返回后唤醒调度。模式/窗口变化后交替调度从新的特征窗口
开始；同值写不重置窗口。两个窗口分别更新，多个属性写入不是一次原子事务。
纯原始模式不再提供有效特征缓存；特征 buffer 可能仍存有切换前的旧记录，须按
时间戳识别。采集模式不自动开启 IIO buffer，仍需应用自行配置并消费对应 buffer。
设置可在离线时保存于驱动内存并在重连后沿用；卸载后不保留，下一次加载重新使用
设备树/默认值。新功能只完成离线回归和目标板内核编译，尚未实机验证。

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

写入与采集共用采集锁，可能等待在途原始窗口完整结束；当前模式的参数变化会唤醒
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
主机模式、窗口与轮询配置读取使用短锁，不等待原始窗口或 OTA；写入继续串行化。
特征字段/单位统一来自 `sensor/sensor_fields.h`，两设备共用缓冲区打包和注册辅助函数。

## 用户命令设置定长长度

`sensor_raw_batch_samples` 读写当前请求长度，`sensor_raw_batch_samples_available` 列出可设置长度。
驱动不自动切换长度。双传感器轮换脚本与显式窗口控制见 [训练采集说明](../tests/training/README.md)，
完整属性定义见 [设备属性宏清单](DEVICE_ATTRIBUTES.md)。

## 4096 点原始声音

XYZS 新增一次性声音触发和独立 IIO buffer，详见 [声音采集](SOUND_CAPTURE.md)。
