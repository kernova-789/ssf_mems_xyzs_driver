# 三路线采集调度重构与验证记录

日期：2026-11-06。范围：`mems_driver` 的采集层、型号回调、定长振动事务、IIO 控制与回归测试。
本记录为主机验证和 ARM64 交叉编译证据；未加载新模块到开发板，未进行真实 UART 或长期稳定性验收。

## 结构调整

| 层 | 职责 |
| --- | --- |
| acquisition | 保存主机配置、连接状态、统一路线窗口截止时间，选择特征/振动/声音并轮转。 |
| sensor_ops | `read_features()` 一次完整特征；`collect_raw()` / `collect_sound()` 一次完整批事务。 |
| protocol/raw | 触发、收流、CRC/序号/数量验收、等待静默和只读确认普通通信恢复。没有重复采集窗口。 |
| IIO | 型号能力筛选、sysfs 配置分派、buffer 就绪检查、扫描布局与数据发布。 |

声音有独立 SOUND 阶段，不再借用 RAW。自动和手动声音使用相同的批事务、在线门禁和恢复处理。
统一选择器按 FEATURE → RAW → SOUND 查找下一可运行路线，窗口到期收完在途批次。
模式为 `feature` / `raw` / `sound` / `alternating`；单路线交替和全部七种非空组合均可运行。

`config` 由短状态锁保护，`active_config` 是采集锁保护的批次快照。
配置复制及变更标志清零在一个短临界区中完成，避免丢失并发写入。
模式、时长、路线开关、振动长度、手动标志和轮询配置写入不等待串口、采集锁或 OTA。
串口配置与手动事务仍取采集锁。普通自动采集每批释放锁，不持有整个振动窗口。

手动 `sensor_sound_capture=1` 等待当前自动批次释放锁后执行一批，保留原模式、交替开关、
`phase_until` 和振动长度；无需上位机先切 feature 再恢复。自动声音运行时手动请求会额外产生一批。
原有 `raw + manual` 显式窗口保持同步事务和窗口开始时的配置快照，以兼容训练采集工具。
该显式窗口仍独占采集锁直至整个窗口完成，其间的手动声音请求等待该事务完成。

振动统计仍按窗口累计，首批重置、后续批比较新增错误，保留 `raw_packets` / `raw_batches` 的窗口口径。
历史丢样留在窗口统计中，后续正确批次可独立返回成功。CRC、序号、数量及只读恢复检查均保留。

## 接口及方案中的具体取舍

- 新增 `sensor_sound_duration_ms`、`sensor_alternating_feature_en`、`sensor_alternating_raw_en`、`sensor_alternating_sound_en`。
- 三种窗口均为 100..60000 ms、默认 5000；支持的交替路线默认开启；禁止关闭最后一项。
- 时长设置使用统一的 route 枚举接口，不再用 bool 区分两个窗口，也不另加声音特例入口。
- XYZ 隐藏声音时长、声音交替开关、手动声音触发；available 列表不包含 sound；不支持配置返回 EOPNOTSUPP。
  XYZ 的特征/振动交替开关仍可用。
- 自动振动/声音要求对应 `buffer/enable=1` 且选中实际数据通道；无可运行路线时休眠，每 1000 ms 复查。
  打开字符设备不等于启用 buffer；启用 buffer 不能证明用户态持续消费，仍需检查丢样。
- 特征路线保留直接 sysfs 读取缓存，因此不依赖特征 buffer 开关。
- 主机配置写入成功表示已保存；批次边界才应用。重连保留配置，卸载后重置。
  旧设备树 raw-duration-ms=0 及有校验位时的 feature 默认行为保留。
- 声音批次维持固定 4096 点 / 16 kHz、原扫描布局和完整批次验收后发布；重复批次不等于连续音频。

## 主机验证

| 检查 | 结果 |
| --- | --- |
| `python3 tests/acquisition/run_tests.py` | 通过，状态机/策略 13817 次检查；真实 pthread mutex 下配置读写、替代型号配置另行通过。 |
| `python3 tests/iio/run_tests.py` | 通过，100000 次发布 / 20000 次 buffer 重配置，含新属性读写、输入限制、XYZ 实际注册属性过滤。 |
| `python3 tests/protocol/run_tests.py` | 全部 18 个变体及型号表校验通过，含独立编译、模块依赖边界、表项删除/移动、长批次和声音协议。 |
| `python3 tests/protocol/run_tests.py baseline` | 新增“丢样后下一批成功且保留窗口统计”用例后再次通过，753544 次检查。 |
| `python3 tests/ota/run_tests.py` | 通过，22586 次检查；修复旧测试链接清单缺少 sound_stream/sound_capture 的问题。 |
| `python3 -m unittest discover -s tests/training -p 'test_*.py'` | 18 项通过。 |
| `git diff --check` | 通过。 |

C 回归使用 AddressSanitizer / UndefinedBehaviorSanitizer。
调度测试包括全部七种组合及 jiffies 回绕、批次长于窗口时完整收尾、声音 buffer 未启用/恢复、
采集中配置变更、下一批应用新长度且不重启窗口、声音恢复失败退避、IIO 丢样不扫描波特率。
手动声音覆盖 POLL / RAW / SOUND / IDLE 就绪阶段，检查原模式和截止时间保持不变。
主机替身不能证明真实内核线程并发、UART 时序、固件响应或 IIO 消费稳定性。

## 目标内核编译

使用已准备的 `/home/u-yoimiya/rk3588/kernel`（Linux 5.10.160），在项目内独立目录复制生产源码后编译，
未执行默认 make 的 NFS 安装步骤，也未覆盖项目根目录原有 `.ko`。

```sh
make -C /home/u-yoimiya/rk3588/kernel M="$PWD/build/tdm-module" \
  ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- W=1 modules
```

编译通过，未报告警告。`readelf` / `file` 确认 ELF64 AArch64 可重定位模块；
vermagic 为 `5.10.160 SMP mod_unload aarch64`。

- 模块：`build/tdm-module/ssf_mems.ko`
- SHA256：`759430b543841d74446e3468f76830bfa6dede3ab58353ba2f3b1a4af140d20b`
- 生产源码清单：`build/tdm-module/source-manifest.sha256`
- 清单 SHA256：`cd904545ac62f5a181cfa93e6429d88ad9e88d30c86cd216a970aa985f0b5ee6`

已逐文件校验清单与当前生产源码及构建副本一致。构建目录被现有 `.gitignore` 排除。
开始前已有的 Makefile 修改和其他未跟踪内容保留；用户提供的设计文档未覆盖。

## 待完成的实机验收

需要在 XYZ / XYZS 实际板卡上验证三路线轮转、各子集组合、buffer 开关、手动插入、
采集中修改配置、断线恢复与卸载。应分实例记录数据数量/序号/CRC、UART 错误计数、
内核日志增量和超时恢复；原有单次声音测试报告不代表本次自动调度已通过实机验证。
