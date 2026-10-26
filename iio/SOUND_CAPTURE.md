# 获取 4096 点定长原始声音

适用于 SSF-MEMS-XYZS。手册的声音定长采集为固定 4096 点、16 kHz；
40063 写 1 开启，发送结束自动归零。驱动使用型号表查找寄存器，
不修改振动采样率、自定义振动长度或 Flash，不开启连续声音采集。
XYZ 不创建声音设备或触发属性。

## 接口与调用流程

- 特征 IIO 设备新增只写属性 `sensor_sound_capture`，只接受 `1`。
- 新增独立 IIO 设备 `ssf_mems_xyzs_sound`，与同一 serdev 下的特征、振动设备配对。
- `sampling_frequency`：16000 Hz，只读；`sound_batch_samples`：4096，只读。
- `sound_capture_status`：最近一次的 active、packets、sequence_gaps、crc_errors、
  ignored_bytes、buffer_drops、累计成功 captures 和最终 error。
  接收统计在解析锁内读取；发布/完成统计独立更新，不把它当成跨阶段原子完成通知。
- 同步写入返回成功才表示完整批次和普通通信收尾确认成功；错误返回负 errno，
  EINTR 不可由应用盲目自动重试。驱动会先等待在途声音上传自然结束，再只读确认通信。

先根据 sysfs 父路径选中实际 RS485 实例，配置声音扫描字段与 buffer，开启数据通道，
再触发一次。buffer 未启用或未选声音数据通道时返回 ENODATA，不发送开启命令。
获取过程中应消费 `/dev/iio:deviceN`；推荐 buffer 至少 4096 条，避免完整批次发布时溢出。
触发属性位于**特征设备**，声音数据从**声音设备**读取。

已有工具包含设备匹配、配置、消费、数量/包号校验与配置恢复：

```sh
sudo python3 tests/hardware/sound/board_sound_capture.py \
  /data/mems-sound/session-001 --serdev serial0-0 --captures 3
```

输出目录必须是新目录。`serial0-0` 对应本板 RS485-1，另一只用 `serial1-0`；
其他板卡需依实际父路径选择。工具会短暂选择 feature 模式，退出恢复原模式及声音 buffer
的扫描配置/长度。它只依赖标准库和仓库 `tests/training/collect_training_data.py` 中的布局读取器；
独立部署时将该文件放到工具同目录。

每批保存 `.iio.bin`（完整扫描记录）、`.pcm-s32le.bin`、16 kHz 单声道 32 位 PCM `.wav`，
以及带 SHA256、扫描 schema、状态和数量的 JSON。每批名义声音覆盖 0.256 秒；
设备等待、采集、传输与收尾总时间可能远大于 0.256 秒。

## 数据布局

驱动保留完整的线上 32 位声音数据字，按有符号计数输出；没有声压标定或 dB 换算。
现有声音 RMS/峰值等 dB 特征与此原始计数是不同接口。

| scan_index | 字段 | 类型 | 全通道字节偏移 |
| --- | --- | --- | --- |
| 0 | in_count0_sound | s32，主机端序 | 0 |
| 1 | in_count1_packet_sequence | u16，0～63 | 4 |
| 2 | in_count2_sample_index | u16，0～63 | 6 |
| 3 | in_timestamp | s64，IIO 主机时间戳 | 8 |

全部开启时每条 16 字节，一批 65536 字节。用户态仍应读取 `_index`、`_type` 计算布局，
不能对任意通道子集硬编码上述偏移。实际输出端序由 `_type` 声明。

接收端检查从机地址、64 包总数、顺序、包内长度及逐包 CRC；完整批次通过后才开始
推送 IIO。CRC/缺包/重复包等损坏批次不发布声音片段。IIO 溢出、发布时关闭 buffer 或卸载
可能导致已发布的部分记录和非零返回；调用方须将这种整批结果判为失败。

IIO 时间戳是完整批次验收后**推送记录的时刻**，不是传感器逐点采样时钟。
批内名义相对时间使用 `样本位置 / 16000`，两只传感器不保证同步。

声音触发由采集锁与特征轮询、振动定长采集、配置写入和固件升级串行化。
在途振动窗口结束后才执行声音命令，之后恢复原调度。每个传感器有独立状态与缓存。
当前为一次一批接口，不提供声音自动循环、可变长度或连续音频。

实测结果见 [4096 点声音测试报告](../tests/hardware/sound/TEST_REPORT_2026-10-26.md)。
