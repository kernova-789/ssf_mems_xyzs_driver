# RK3588 双路真实采集与 Replay 上机验收

本目录是 2026-10-26 实机测试适配程序，不是后端服务。正式驱动由两个 RS485
独立实例驱动两只 XYZS，测试不会直接打开 tty 或发送传感器协议帧。

`run_board_test.py`、`replay_live_import.py`、`verify_replay.py` 应与
`tests/hardware` 的 `board_capture.py`、`iio_devices.py`、`check_capture.py`、
`analyze_finite_iio.py` 等依赖放在板端同一目录。该目录还需包含板端构建的
`replay-build/replay_offline_example`。

```sh
sudo python3 run_board_test.py /data/ssf-tests/<新的目录> --seconds 2700
```

采集前两个实例的 IIO buffer 必须空闲。测试将两路设置为 1024 点、自动原始窗口、
5 秒特征 / 5 秒原始交替。第一路使用原有驱动采集脚本；第二路也保留原始文件，
适配程序将增长的文件按完整 1024 点原始批次、单条特征记录，分批经公开示例送入 Replay。
不是无间断波形：窗口之间以及设备内部采集/上传之间均可能存在空档。

数据以透明 IIO 字节块保存，单位 count；原始块有 5333.4 Hz 名义采样率，
特征块不声明波形采样率。块的字段布局和 scale 见捕获摘要、板端 sysfs 及最终报告。
IIO 时间戳属于主机接收时钟，不是传感器逐点时钟；适配器明确读取
`current_timestamp_clock`，realtime 直接换算微秒，monotonic 使用固定实测偏移。
不支持的时钟或偏离当前会话超过 5 分钟的时间明确拒绝。两路没有同步采样保证。
资产编号只表示本次试验的物理端口位置，不是自动读取的硬件序列号。

输入 TSV 和期望 SHA256 清单独立保存，查询和回放均逐条比较字节摘要、时间、来源、
单位与标签，并检查全部分页/积压结束、重复重投以及 SQLite 完整性。
没有 AI 诊断输入，读回状态必须为 unassessed。

此适配程序通过每 5 秒轮询增长文件实现有限积压，不提供持久队列确认协议。
它用于一次受控长测，不构成生产采集服务或掉电保护验收。
结束后两路留在 feature 模式，采集脚本关闭其拥有的 buffer；
原配置保存在 original-config.json，可据此人工恢复。

变长采集另用 `tests/training/collect_training_data.py`，逐档轮换 1024、2048、
4096、8192、16384、32768；不能用当前固定 1024 点二进制检查器验证其他长度。
全部采集、数据库、日志、构建和备份应位于确认挂载的 `/data` 盘。
