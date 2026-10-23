# 两只温振声传感器的训练数据记录脚本

本目录集中存放采集脚本、数据读取工具、离线测试和数据格式说明。
本文中的运行命令均从项目根目录执行。

入口：[collect_training_data.py](collect_training_data.py)。仅依赖 Python 3 标准库，运行在开发板上。
脚本通过公开 IIO 接口工作；当前版本尚未进行实机验证，首次运行应作为验收测试。

保存后的目录、元数据、原始/特征字段、单位、时间戳、有效性判定和读取示例，见
[采集数据存储与使用说明](TRAINING_DATA_FORMAT.md)。交接或备份数据时请一并保留该说明。

## 支持范围与用户态长度轮换

- 两个独立 RS485 口，各连接一只 XYZS；用户明确指定两个 serdev 实例。
- 一只保持 `raw` 模式，由脚本逐次请求完整定长窗口，另一只保持 `feature` 模式。
- 长度顺序属于脚本；驱动没有轮换表或自动切换长度的定时器。
- 默认脚本轮换 `1024 2048 4096 8192 16384 32768`，每档请求五秒窗口。
  `--lengths` 和 `--window-seconds` 可改变顺序及窗口时长。
- 新驱动支持的长度是软件开放范围，新增非 1024 长度尚需逐档实机验收。
- 不改变采样率、波特率、固件或设备树，也不加载/卸载模块。
- 退出恢复原有主机配置和 buffer 状态；部署新版模块后才能使用本脚本。

脚本通过 `sensor_raw_batch_samples_available` 读取列表，通过
`sensor_raw_batch_samples` 写入并读回本档真实批次长度。它开启 `sensor_raw_manual=1`，
使后台 raw 模式等待显式命令；另一个线程写 `sensor_raw_capture=1`，同步执行一轮
`sensor_raw_duration_ms` 窗口，同时记录线程持续消费 IIO buffer。

窗口截止时仍收完在途批次。同步命令成功返回、buffer 排空后，脚本将记录数与驱动
有效包数/成功批次数进行完整对账，并确认错误计数为零、实际长度和采样率符合配置。
通过后才发布这一窗口的训练索引，然后由脚本发送下一档长度命令。
失败窗口的暂存文件不会发布到训练索引，目录里会保留 `failed-window-*.json`。

因此“五秒”是窗口目标，含采样、等待、上传及收尾，不是硬实时切换，也不表示五秒
无间断波形。长批次可能单批就超过五秒，此时完整收完后再切换，不截断、不拼接批次。
1024 点在默认 5333.4 Hz 下名义覆盖约 0.192 秒。原始通道只有 XYZ 振动，没有原始音频。

驱动默认仍为自动循环、1024 点，以兼容既有程序；手动窗口功能是脚本显式启用的。
`raw_batch_samples` 表示当前/最近执行窗口的实际长度，
`sensor_raw_batch_samples` 表示用户设置的目标长度；两者在下一窗口开始前可能不同。
普通 `sensor_sampling_length_index` 仍是另一项配置，不用于本脚本切换定长批次。

## 首次上机测试

先部署并加载已验证的驱动，确认两路传感器上线，然后只读枚举：

```sh
python3 tests/training/collect_training_data.py --list
```

依据输出的父设备路径确认安装位置，明确哪只负责原始采集，哪只负责特征。
不能只凭相同的 `ssf_mems_xyzs` 名称或固定 IIO 编号来区分。以下 serdev 名称仅为示例：

```sh
sudo python3 tests/training/collect_training_data.py \
    --output /data/mems-training \
    --raw-serdev serial0-0 \
    --feature-serdev serial1-0 \
    --lengths 1024 2048 --window-seconds 5 --seconds 60 \
    --machine-id machine-01 --condition unlabeled \
    --raw-sensor-id sensor-A --raw-mount '轴承座A' \
    --feature-sensor-id sensor-B --feature-mount '轴承座B' \
    --notes '填写两个传感器的安装位置、转速、负载和采集目的'
```

所有四个相关 IIO buffer 必须空闲。脚本会拒绝接管已启用的 buffer，
并使用实例文件锁防止本脚本的重复运行；其他控制程序也应退出，避免改动模式或配置。
前台出现的会话目录是后续校验入口：

```sh
python3 tests/training/collect_training_data.py --verify '/data/mems-training/<实际会话目录>'
```

`<实际会话目录>` 必须替换，不能原样复制到 shell。
短测后需要检查 UART/内核日志、文件数据与实际信号，再安排目标时长的双路长测。
本脚本的离线校验不代替这些实机验收。

正式长期采集时，将 `--seconds 60` 改为目标秒数；`--seconds 0` 表示一直运行到 Ctrl+C
或 SIGTERM。停止时尽量收完已经接收中的原始批次，所以退出可能晚于指定秒数。
会话时长从子进程启动后计算，包含上线和配置等待，不承诺等量的有效波形覆盖时间。

## 数据目录

每次运行创建新的 UTC 时间加随机后缀目录，不覆盖旧会话：

```text
20261023T090000Z-ab12cd34/
  session.json                 会话参数、标签、设备映射、扫描布局、scale、脚本哈希
  summary.json                 整轮结果、两路计数、清理结果
  raw/
    original_config.json       启动前主机配置
    original_buffer.json       启动前扫描和 buffer 配置
    capture_config.json        本次采集参数、固件及频率
    health.jsonl               窗口结束后的驱动统计（feature 下为约每秒在线状态）
    status.json                最近进度及最终状态
    index.jsonl                已通过整个窗口验收的原始批次索引
    window-00000001.json        本窗口长度、文件列表和最终驱动统计
    2026-10-23/09/
      000000000001.bin          单批完整 IIO 扫描记录
      000000000001.json         样本数、时间范围、SHA256 等元数据
  feature/
    ...                        配置、状态、索引同上
    2026-10-23/09/
      000000000001.csv          特征原始整数值，默认每 60 秒分片
      000000000001.json
  rejected/                    异常批次、残批、未提交记录及原因（出现异常时创建）
```

原始批次单独保存，不能直接串接后当作连续波形。文件名在每个角色内递增，目录按存储时刻的
UTC 日期/小时组织；真正的数据时间范围在 JSON 及记录中的 IIO 时间戳里。
时钟来源记录在扫描 schema 的 `timestamp_clock`。主机 IIO 时间戳不是传感器逐点采样时钟，
也不能用于证明两只传感器同步采样；分析批内采样间隔应使用采样率。

## Python 读取示例

推荐使用 [training_dataset.py](training_dataset.py) 统一读取。它先验收整个会话，
再逐批/逐分片检查文件，返回传感器编号、安装位置、机器及工况标签，减少手工解析出错。
只需 Python 标准库，不依赖 NumPy；需要张量时可在训练程序中自行转换。

```python
import sys
sys.path.insert(0, 'tests/training')
from training_dataset import TrainingDataset

dataset = TrainingDataset('/data/mems-training/实际会话目录')
for batch in dataset.raw_batches(physical=True):
    xyz = batch['xyz_m_s2']         # N×3，m/s²；原始整数另存于 xyz_counts
    relative_t = batch['sample_time_seconds']  # 本批内部的相对采样时间
    sensor_id = batch['sensor_id']
    condition = batch['condition']
    # 每次 yield 是独立批次；不跨越批间空档拼接。

for row in dataset.feature_rows(physical=True):
    values = row['physical']        # 单位对应 row['physical_units']
    counts = row['raw']             # 保留原始整数，不覆盖原始数据
```

也可生成便于筛选和统计的 CSV 目录表，每个载荷文件一行，包含标签、角色、相对路径、
样本数、窗口、采样率、时间范围和 SHA256。目录表是派生文件，放在会话外；不会覆盖已有文件：

```sh
python3 tests/training/training_dataset.py '/data/mems-training/<实际会话目录>' \
    --catalog /data/mems-training-catalog.csv
```

移动或备份数据时复制整个会话目录。内部使用相对路径，移动后仍可通过同一入口读取；
`session_id` 及每个读取单元的 `unit_id` 可用于避免重复导入。不要直接修改载荷、索引或旁车 JSON。

下例为更底层的二进制读取方式：

XYZ 原始二进制保留 IIO 原记录。不要无条件假设永远是 24 字节；根据本次会话的
`scan_bytes`、字段 `offset/type/bits/shift` 解码。可以复用脚本中的只读解码器：

```python
import json
import sys
from pathlib import Path

sys.path.insert(0, 'tests/training')
from collect_training_data import Decoder

session = Path('/data/mems-training/实际会话目录')
metadata = json.loads((session / 'session.json').read_text())
decoder = Decoder(metadata['devices']['raw']['schema'])
with (session / 'raw/index.jsonl').open() as index:
    for line in index:
        entry = json.loads(line)
        blob = (session / entry['payload']).read_bytes()
        xyz = []
        for offset in range(0, len(blob), decoder.size):
            row = decoder.decode(blob[offset:offset + decoder.size])
            xyz.append([row['in_accel_x'], row['in_accel_y'], row['in_accel_z']])
        # xyz 是本批 entry['samples']×3 有符号 ADC 整数；乘对应 scale 得到 m/s²。
```

特征 CSV 列为接收时的 `received_realtime_ns`、`same_payload_as_previous`，以及全部扫描通道。
`in_timestamp` 是内核 IIO 时间戳；数值字段为 IIO 原始整数，换算比例及单位保存在 schema。
温度乘 scale 得到毫摄氏度，再除以 1000 得到摄氏度。
`same_payload_as_previous=1` 只表示除时间戳外的特征数值与上一记录相同，
不会删除这条记录，也不能据此判断设备没有完成新一轮测量。

两个角色对应不同物理传感器。原始窗口与另一只传感器的特征不能默认视为同一个样本的标签，
需要结合安装位置、机器工况和实验记录建立对应关系。数据集划分宜以会话/机器/实验批次为单位，
避免相邻窗口被随机分到训练与验证集而造成数据泄漏。

`sensor_id` 是用户维护的实体编号，`mount` 是实际安装位置，不是驱动自动读出的产品序列号。
未知信息保留 `unlabeled`，不要填猜测的故障类别。工况或安装发生变化时开始新会话，
保持一个会话的标签含义一致。正式训练时应明确是否排除未标注会话。

## 完整性、失败与磁盘管理

- 原始记录逐条检查动态布局、填充、包号、包内点号、正且不倒退的时间戳；只提交与本窗口设置长度一致的完整批次。
- 原始 Modbus/私有包 CRC 已由驱动检查；IIO 不保留线级 CRC，脚本不能独立重算原始串口 CRC。
- 原始窗口返回后读取完整窗口累计计数，检查 CRC/序号/丢样/排空错误及包数，
  确认后才提交该窗口。显式窗口之间不会自动启动下一轮或重置计数。原始窗口超时或特征停采时停止会话。
- 当前格式为 `format_version=2`，离线验收再次核对窗口证据、文件清单、长度、频率、
  时间范围、CSV 字段/整数范围/重复标记及两路最终计数。缺少窗口证据的数据不能通过。
- 特征 buffer 没有累计丢样计数，本脚本可识别长时间停采和时间戳异常，不能证明每个特征轮次都收到。
- 数据文件先写 `.partial` 并 fsync，完成后改名并保存校验元数据，再追加索引。
  训练程序只使用正常完成会话中通过 `--verify` 的索引项；孤立文件、残留 `.partial`、
  `rejected/` 或失败会话不自动并入训练数据。SHA256 验证的是文件完整性，不是物理测量精度。
- 每档五秒的轮换示例可用 `--lengths 1024 2048 4096 --window-seconds 5`。
- 默认保留 512 MiB 磁盘余量；空间不足即停止，不删除旧数据，也不会静默覆盖旧文件。
  `--min-free-mib` 可调整；应将数据根目录设置到足够容量的本地存储。
- 内存只保留一批原始数据、一个窗口的文件索引和有界特征分片，不把整个长期会话装入内存。
  默认特征每 60 秒提交一次；异常中断/掉电可能留下未提交分片。
- 某一路故障时停止两路，返回非零并记录原因。正常退出恢复主机配置；强制杀死、掉电或内核卡住时
  不保证恢复，摘要中的 `manual_cleanup_required` 和两路 `cleanup_errors` 必须检查。

生产采集前，仍需确认实际传感器身份、安装/量程、采样率、标签含义、两路信号对应关系和长期通信质量。

## 本地离线验证

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests/training -p test_collect_training_data.py -v
```

这些测试使用合成数据、临时 sysfs 文件和管道，不访问开发板，不证明硬件长时间可靠性。
