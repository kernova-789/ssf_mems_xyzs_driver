# 采集数据存储与使用说明

本文对应 [collect_training_data.py](collect_training_data.py) 保存的 `format_version=2` 数据。
采集参数和上机操作见 [采集说明](README.md)，统一读取入口为
[training_dataset.py](training_dataset.py)。示例路径中的“实际会话目录”需要替换。

## 1. 一份数据包含什么

一次脚本运行称为一个**会话**：一只 XYZS 负责定长原始振动采集，另一只 XYZS 负责特征采集。
两只传感器的编号、安装位置和角色在本会话内固定。

| 名称 | 含义 | 保存方式 |
| --- | --- | --- |
| 原始点 / 扫描记录 | 同一序号下的一组 X、Y、Z 整数，加包号、包内点号和主机时间戳 | 一条二进制 IIO 记录 |
| 原始批次 | 完整的 N 组 XYZ；N 为本窗口配置的长度 | 一个 `.bin` 文件及同名 `.json` |
| 原始窗口 | 用户态请求的一轮定长采集；本窗口内所有批次使用同一个 N | `window-XXXXXXXX.json`，关联一个或多个批次 |
| 特征记录 | 一次收到的全部特征值及主机时间信息 | CSV 的一行 |
| 特征分片 | 多条特征记录；默认约 60 秒提交一次，达到 10000 行或正常停止时也提交 | 一个 `.csv` 文件及同名 `.json` |
| 会话 | 两个角色的数据、身份信息、配置和运行结果 | 一个独立目录 |

**1024 点表示 1024 组 XYZ，即每轴 1024 个数，共 3072 个加速度整数。**
N 可为 `1024、2048、4096、8192、16384、32768`，实际可用范围还受驱动及实机验收限制。
长度的轮换由脚本发命令完成，驱动不自行轮换。

窗口目标默认五秒，包含请求、采样、传输和收尾。窗口到期会等待在途批次完成，实际耗时可能更长。
批内按采样率分析，批次之间存在空档，不能直接拼接为连续波形。原始文件仅含 XYZ 振动，
没有温度逐点波形或原始音频；温度和声音指标位于另一只传感器的特征记录中。

## 2. 目录和文件

每次运行创建 `UTC开始时间-随机后缀` 目录，不覆盖上次会话：

```text
20261023T090000Z-ab12cd34/
├── session.json
├── summary.json
├── raw/
│   ├── original_config.json
│   ├── original_buffer.json
│   ├── capture_config.json
│   ├── status.json
│   ├── health.jsonl
│   ├── index.jsonl
│   ├── window-00000001.json
│   └── 2026-10-23/09/
│       ├── 000000000001.bin
│       └── 000000000001.json
├── feature/
│   ├── original_config.json
│   ├── original_buffer.json
│   ├── capture_config.json
│   ├── status.json
│   ├── health.jsonl
│   ├── index.jsonl
│   └── 2026-10-23/09/
│       ├── 000000000001.csv
│       └── 000000000001.json
└── rejected/                         出现异常时才创建
```

| 文件 | 用途 |
| --- | --- |
| `session.json` | 格式版本、会话 ID、参数/标签、设备映射、扫描 schema、比例、软件版本线索 |
| `summary.json` | 会话最终状态、两路记录数/文件数、错误和配置恢复结果 |
| `original_config.json` | 本角色对应传感器启动前的模式、轮询及原始采集控制配置 |
| `original_buffer.json` | 启动前本角色 IIO buffer 的容量、水位（若提供）和扫描使能值 |
| `capture_config.json` | 配置时的模式、采样频率、采样率索引、固件版本、波特率等 |
| `status.json` | 最近进度或最终状态；运行中会被更新，不是历史记录表 |
| `raw/health.jsonl` | 每个成功原始窗口结束后的驱动统计 |
| `feature/health.jsonl` | 特征进程约每秒记录的在线状态 |
| `index.jsonl` | 本角色已发布的有效载荷清单；每行一个完整 JSON 对象 |
| `window-XXXXXXXX.json` | 原始窗口的长度、耗时、最终统计和本窗口完整批次文件清单 |
| 载荷旁的同名 `.json` | 该文件的样本/记录数、时间范围、字节数、SHA256；内容与索引该行一致 |

`capture_config.json` 中的 `raw_batch_samples` 是配置阶段读取的当时/最近窗口值，
不是后续轮换长度表。分析某批实际长度时，使用该批索引的 `samples` 及所属窗口记录。

日期和小时目录按**文件保存时刻的 UTC** 划分，UTC 09 时对应北京时间 17 时。
文件名是角色内递增的 12 位编号，不是传感器编号或时间戳；`raw` 和 `feature` 各自编号。
异常隔离或跨小时保存可能使目录内编号不连续，按索引读取，不依赖连续文件名。
JSON/JSONL 和 CSV 均使用 UTF-8；JSONL 应逐行解析，不能把整个文件当成一个 JSON 对象。

## 3. 会话身份、标签和扫描 schema

`session.json` 中的关键字段如下。`raw` 和 `feature` 均为角色名。

| 字段 | 含义 |
| --- | --- |
| `format_version` | 当前为整数 `2`；读取器只接受支持的版本 |
| `session_id` | 创建时的会话目录名；搬运或重命名目录后，JSON 内 ID 保持原值 |
| `started_utc` | 创建会话的 UTC 墙上时间，ISO 8601 字符串 |
| `arguments` | 本次 CLI 参数；含 `lengths`、`window_seconds`、`poll_ms` 等 |
| `arguments.machine_id` | 用户定义的被测机器编号 |
| `arguments.condition` | 用户确认的工况/故障标签；未知时为 `unlabeled` |
| `arguments.notes` | 安装、转速、负载、实验目的等补充说明 |
| `devices.<角色>.sensor_id` | 用户维护的传感器实体编号，不是自动读取的硬件序列号 |
| `devices.<角色>.mount` | 实际安装位置 |
| `devices.<角色>.serdev / parent` | 本次采集的父实例名称及实际设备路径 |
| `devices.<角色>.raw / feature` | 该物理实例的两个 IIO sysfs 路径；搬运后仅作为来源记录 |
| `devices.<角色>.schema` | 本角色采集通道的格式、排列、比例和时间戳时钟 |
| `script_sha256` | 采集脚本文件的 SHA256；用于核对使用的脚本版本 |
| `host_kernel / driver_srcversion` | 主机内核版本与可读取的模块源版本标识；后者也可能为 `not exposed` |
| `raw_is_contiguous_across_batches` | `false`，批间不保证连续 |
| `cross_sensor_synchronization` | `false`，两只传感器没有同步采样保证 |

schema 的 `scan_bytes` 是一条二进制 IIO 记录占用的字节数，`timestamp_clock` 是驱动暴露的
IIO 时钟名称，未提供时为 `not exposed`。`fields` 按扫描索引排序，每个字段包含：

| 字段 | 含义 |
| --- | --- |
| `name / index` | 通道名 / 本次扫描索引；索引不是 Modbus 寄存器地址 |
| `type` | IIO 类型字符串，如 `le:s16/16>>0` |
| `offset / width` | 在一条记录中的字节偏移 / 存储字节数 |
| `bits / shift / signed / byteorder` | 有效位宽、右移量、是否有符号、字节序 |
| `scale` | 可选，字符串形式的换算比例；物理值 = 原始整数 × scale |
| `unit_after_scale` | 换算后的单位说明；未明确的指标保留设备定义的含义 |

保存的 CSV 数值也是 IIO 原始整数。读取历史数据应使用**本会话保存的 schema**，
不要从后来更新过的驱动或其他会话复制比例。当前驱动没有额外的通道 offset 换算。

## 4. 原始振动 `.bin` 和批次索引

`.bin` 是连续排列的 IIO 扫描记录，没有额外文件头，也不是完整的串口报文。
每条记录含以下字段：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `in_accel_x` | 有符号 16 位 | X 轴原始加速度计数 |
| `in_accel_y` | 有符号 16 位 | Y 轴原始加速度计数 |
| `in_accel_z` | 有符号 16 位 | Z 轴原始加速度计数 |
| `in_count0_packet_sequence` | 无符号 32 位 | 本批包号，从 0 到 N/64−1；每批重新从 0 开始 |
| `in_count1_sample_index` | 无符号 16 位 | 包内点号，从 0 到 63 |
| `in_timestamp` | 有符号 64 位 | 主机 IIO 时间戳，纳秒 |

当前 ARM64 小端平台、全部通道开启时，常见记录大小为 24 字节：XYZ 位于偏移 0/2/4，
包号位于 8，包内点号位于 12，时间戳位于 16，其余为对齐填充。
**解析仍以本会话 schema 为准**，不要固定按“三个 int16”读取整个文件。

`raw/index.jsonl` 每行和旁车 JSON 包含下列信息：

| 字段 | 含义 |
| --- | --- |
| `payload` | 相对于会话根目录的载荷路径，如 `raw/2026-10-23/09/000000000001.bin` |
| `samples` | 本批 XYZ 组数 N |
| `window_id` | 所属窗口编号，从 1 开始 |
| `sampling_frequency_hz` | 本批采样频率，当前保存为十进制字符串，使用时转换为数值 |
| `first_iio_timestamp_ns / last_iio_timestamp_ns` | 本批首尾记录的主机 IIO 时间戳 |
| `bytes / sha256 / stored_utc` | 文件字节数、文件 SHA256、保存时刻 UTC |
| `schema` | 格式位置提示，值为 `session.json:devices.raw.schema` |
| `validation` | 本批使用的校验规则名称 |
| `contiguous_across_batches` | `false`，不要据此跨批补成连续波形 |

应满足 `bytes = samples × scan_bytes`。24 字节布局下，1024 点为 24576 字节，
2048 点为 49152 字节；这些大小包含序号、时间戳和填充。

当前原始加速度每计数约为 `0.004788403 m/s²`，换算必须取本会话各轴的 `scale`。
它与特征加速度的比例不同。需要 g 单位时，可在派生数据中用 `m/s² ÷ 9.80665`。
批内第 i 点（从 0 开始）的相对采样时间为 `i / sampling_frequency_hz` 秒，
首末点间名义跨度为 `(N−1)/Fs`，N 点名义覆盖时长常记作 `N/Fs`。
这不等于首尾主机 IIO 时间戳之差。

## 5. 特征 `.csv` 和分片索引

CSV 第一行为列名，前两列固定为：

| 列名 | 含义 |
| --- | --- |
| `received_realtime_ns` | 用户态解码收到该行时的系统墙上时间，Unix epoch 纳秒 |
| `same_payload_as_previous` | `0/1`；除 IIO 时间戳外的特征值是否与上一条完全相同，跨分片继续比较 |

其后是 `devices.feature.schema.fields` 中的全部通道，按 schema 顺序排列。
列名不带 sysfs 属性的 `_raw` 后缀。当前 XYZS 有 29 个特征通道和一个 `in_timestamp`，
加上前两列共 32 列；未来版本仍以会话 schema 和 CSV 表头为准。
特征数值中温度为有符号 16 位，运行标志为无符号 8 位，其余特征为无符号 16 位；
IIO 时间戳为有符号 64 位。CSV 保存解码后的十进制整数，不保存二进制对齐填充。

下表的 `{x,y,z}` 表示分别存在 X/Y/Z 三列，比例列是当前驱动值的说明，实际使用保存的 scale：

| CSV 通道名 | 含义 | 当前 scale / 换算后单位 |
| --- | --- | --- |
| `in_accel_{x,y,z}_high_freq_rms` | 高频加速度有效值 | `0.0980665`，m/s² |
| `in_velocity_{x,y,z}_low_freq_rms` | 低频速度有效值 | `0.00001`，m/s |
| `in_temp` | 温度，有符号整数 | `10`，毫摄氏度 |
| `in_accel_{x,y,z}_peak_to_peak` | 加速度峰峰值 | `0.0980665`，m/s² |
| `in_accel_{x,y,z}_peak` | 加速度峰值 | `0.0980665`，m/s² |
| `in_accel_{x,y,z}_rms` | 加速度有效值 | `0.0980665`，m/s² |
| `in_index_{x,y,z}_kurtosis` | 峭度指标 | `0.01`，无量纲 |
| `in_velocity_{x,y,z}_rms` | 速度有效值 | `0.00001`，m/s |
| `in_count0_sound_rms_db` | 声音有效值 | `0.01`，设备定义的 dB |
| `in_count1_sound_peak_db` | 声音峰值 | `0.01`，设备定义的 dB |
| `in_count2_sound_peak_to_peak_db` | 声音峰峰值 | `0.01`，设备定义的 dB |
| `in_count3_zero_crossing_rate_percent` | 过零率 | `1`，百分数；20 表示 20% |
| `in_count4_spectral_centroid_hz` | 频谱重心 | `0.1`，Hz |
| `in_count5_spectral_flux` | 频谱变化指标 | `0.01`，设备定义，无标准物理单位 |
| `in_count6_startup_flags` | XYZ 运行判定位掩码 | 无 scale；bit0/1/2 分别对应 X/Y/Z |
| `in_timestamp` | 主机 IIO 时间戳 | 纳秒；时钟来源见 schema |

温度原始值 `2500 × 10 = 25000` 毫摄氏度，即 `25 ℃`；读取工具的 `physical['in_temp']`
仍为毫摄氏度，需要摄氏度时再除以 1000。速度转换为 mm/s 时，在换算后的 m/s 上再乘 1000。
运行标志 0 不代表主机禁用了采集；声音 dB 指标也不要自行当作已标定声压级或 dBA。

`feature/index.jsonl` 的公共字段与原始索引相同，数量使用 `records`，表示该 CSV 的数据行数，
不含表头；`schema` 指向 `session.json:devices.feature.schema`。
另有 `values` 说明这些值为整数、比例位于 schema。没有原始批次的 `samples/window_id`。
`same_payload_as_previous=1` 的记录仍被保存，不自动去重，也不等于没有发生新测量。

## 6. 时间与窗口验收记录

| 时间字段 | 用途与边界 |
| --- | --- |
| `*_utc / utc` | ISO 8601 UTC 墙上时间，用于实验记录、保存和运维关联 |
| `received_realtime_ns` | 用户态接收记录的 Unix epoch 纳秒；系统校时可能使其跳变 |
| `in_timestamp`、`*_iio_timestamp_ns` | 主机 IIO 发布时钟的纳秒值，不能无条件当作 Unix 时间转换 |
| 窗口 `started_monotonic_ns / ended_monotonic_ns` | 主机单调时钟，用差值计算该窗口实际运行耗时 |
| 读取工具的 `sample_time_seconds` | 按 Fs 推导的批内相对时间，从 0 开始；不用于跨批或跨传感器对时 |

主机 IIO 时间戳可以相等，但必须为正且不倒退。它记录主机发布过程，
不是每个点的传感器采样时刻；不能据此认定两只传感器同步。
纳秒时间戳应使用 64 位整数或 Python 整数，避免用浮点数或表格软件默认数字格式造成精度损失。

`raw/window-XXXXXXXX.json` 包含 `samples`、`requested_seconds`、UTC/单调起止时间、`health` 和
`files`。`files` 是本窗口通过验收的完整批次路径，顺序与原始索引一致。
`health` 保存窗口编号、实际长度、采样率、有效包数/批次数和以下错误计数：

| 字段 | 检查对象 |
| --- | --- |
| `raw_sequence_gaps / raw_crc_errors` | 原始包序号异常 / CRC 错误 |
| `raw_buffer_drops` | 向 IIO buffer 发布失败 |
| `raw_drain_sequence_gaps / raw_drain_crc_errors` | 排空阶段序号异常 / CRC 错误 |
| `raw_ignored_bytes` | 原始接收过程中被忽略的字节 |

发布窗口时要求上述错误计数全部为 0，并且 `raw_batches = len(files)`、
`raw_packets × 64 = 本窗口全部批次的 samples 之和`，实际长度/频率与配置一致。
脚本逐条检查包号、包内点号、填充和时间戳，再与窗口统计对账。
串口 CRC 由驱动检查；`.bin` 不保留串口 CRC，离线无法重新计算线级 CRC。

## 7. 哪些数据可以使用

先运行离线验收，成功后再通过索引或统一读取器使用数据：

```sh
python3 tests/training/collect_training_data.py --verify '/data/mems-training/实际会话目录'
```

通过时输出 `raw` 和 `feature` 两个总记录数：前者为全部有效 XYZ 组数，后者为特征数据行数。
`summary.json` 的 `workers.<角色>.records` 与之对应，`units` 分别是原始批次数或 CSV 分片数。
两路记录数不能直接比较为同一种采样数量。

验收要求会话及两路 worker 的 `state` 为 `completed`、没有配置清理错误，也没有
`manual_cleanup_required`。还会检查文件大小/哈希、旁车与索引一致性、窗口证明、完整批次、
特征字段/整数范围/重复标记、时间范围及摘要计数。

`.partial`、未出现在索引中的孤立文件、`rejected/` 和失败会话不自动进入训练数据。
`rejected/raw-window-XXXXXXXX/` 可保存未通过窗口验收的整批文件，
`raw/failed-window-XXXXXXXX.json` 记录相应文件；其他隔离 `.bin` 也可能是残片或
JSON 编码的未提交特征行，不能按正常原始批次解码。保留它们用于排查，不手工补入索引。
会话中存在隔离文件时，是否可使用已发布部分仍由完整会话验收结果决定。

SHA256 用于检查载荷字节完整性，不能证明传感器物理精度或标签真实，也不是数据真实性签名。
特征接口没有累计丢样计数，现有校验不能证明每次特征轮询都已保存。
实际量程、安装、标定、信号合理性和长期稳定性仍需上机验证。

## 8. 后续读取和整理

从项目根目录运行下面的 Python 示例。读取器构造时先验收整个会话，迭代时再检查载荷，
逐批/逐分片读取，不把整个会话装入内存：

```python
import sys
sys.path.insert(0, 'tests/training')
from training_dataset import TrainingDataset

dataset = TrainingDataset('/data/mems-training/实际会话目录')
for batch in dataset.raw_batches(physical=True):
    xyz_counts = batch['xyz_counts']            # N×3 原始整数，列顺序 X/Y/Z
    xyz_m_s2 = batch['xyz_m_s2']                # N×3，m/s²
    relative_t = batch['sample_time_seconds']   # N 个批内相对时间，秒
    host_t = batch['host_iio_timestamp_ns']     # N 个主机 IIO 时间戳，纳秒
    batch_length = batch['metadata']['samples']
    fs = float(batch['metadata']['sampling_frequency_hz'])
    identity = (batch['unit_id'], batch['sensor_id'], batch['mount'], batch['condition'])
    # 每个 batch 独立使用；需要张量时再自行转换为 NumPy / 框架张量。

for row in dataset.feature_rows(physical=True):
    integers = row['raw']                      # 包括 in_timestamp 的原始整数
    physical = row['physical']                 # 仅含提供 scale 的通道
    units = row['physical_units']              # 与 physical 对应的单位
    temperature_c = physical['in_temp'] / 1000
    identity = (row['unit_id'], row['sensor_id'], row['mount'], row['condition'])
```

两种迭代结果均附带 `session_id/role/machine_id/condition/sensor_id/mount/serdev/notes`。
原始 `unit_id` 为 `会话ID:raw:文件编号`，特征逐行 `unit_id` 为
`会话ID:feature:文件编号:从0开始的行号`，用于跟踪样本来源和避免重复导入。

也可生成每个载荷文件一行的 CSV 目录表，输出必须位于会话外且文件尚不存在：

```sh
python3 tests/training/training_dataset.py '/data/mems-training/实际会话目录' \
    --catalog /data/mems-training-catalog.csv
```

目录表包含身份/标签、相对 `payload` 路径、`records`、`samples`、`window_id`、
`sampling_frequency_hz`、首尾 IIO 时间戳、`bytes` 和 `sha256`。
其中 `records` 对 raw 是 XYZ 组数，对 feature 是 CSV 数据行数；仅适用原始批次的列在
feature 行中留空。目录表的特征 `unit_id` 标识整个分片，末尾不带逐行编号。
目录表是文件清单，不是合并后的波形或特征矩阵；路径相对于原会话根目录。

整理和训练时遵循这些约定：

- 搬运/备份整个会话目录并重新验收，保留本文、采集/读取脚本版本和实验记录；仅保留 `.bin/.csv` 会丢失比例、身份和验收依据。
- 保留原始载荷、索引和 JSON 不变。滤波、归一化、重采样、标签修订和张量文件放在会话外的派生目录，并记录来源 `unit_id`、参数及处理版本。
- 未确认的标签保持 `unlabeled`。转速、负载、安装位置或工况标签发生变化时开始新会话，避免一个会话的标签含义混杂。
- 两个角色来自不同物理传感器，原始批次与特征行不能默认一一对应。需要配对时，以实验设计、安装关系和时间依据建立明确规则。
- 不同长度分别使用或明确记录裁剪/补齐策略；按会话、机器或实验批次划分训练/验证/测试集，避免相邻批次随机拆分造成数据泄漏。
