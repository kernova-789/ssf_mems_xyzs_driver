# 定长声音实机采集

工具 `board_sound_capture.py` 通过正式 IIO 接口获取 4096 点声音批次。
完整用法、数据布局和返回语义见 [接口说明](../../../iio/SOUND_CAPTURE.md)。
本轮结果见 [实机报告](TEST_REPORT_2026-10-26.md)。

数据输出到独立新目录，支持 IIO 原扫描、PCM 和 WAV，均保留数量及 SHA256 元数据。

## 五分钟测试（2026-10-26）

[本轮报告](TEST_REPORT_2026-10-26_5MIN_GATE.md)：两路各 48 个声音批次和 96 条声音回放通过；
普通通信出现两条 CRC 候选帧告警，门禁未通过，未执行后续 30 分钟三阶段长测。

`board_sound_capture.py --seconds 300` 持续重复完整声音批次，达到时长后收尾。
`run_sound_endurance.py OUTPUT 300` 同时测试两路，并检查 UART/完整内核日志门禁。
`replay_sound_session.py CAPTURE OUTPUT EXECUTABLE` 验证声音文件并使用板端 Replay 示例入库、查询、回放和核对。

## CRC 告警来源分析

[主机与实机复现报告](CRC_ROOT_CAUSE_ANALYSIS_2026-10-26.md)：已确认普通接收器对合法响应内部假候选的 CRC 误报警；正式逻辑尚未修复，UART framing 和采集超时仍需单独定位。
