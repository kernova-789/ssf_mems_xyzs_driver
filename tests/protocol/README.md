# Protocol tests

Run from the repository root:

```sh
python3 tests/protocol/run_tests.py
```

The runner compiles each real protocol source as an independent translation unit and links them with a single-threaded Linux/serial shim and AddressSanitizer/UndefinedBehaviorSanitizer. Each public header is also compiled on its own. Object-symbol checks enforce the dependency direction: the codec cannot call upper layers, receive cannot call request/business directly, and request can use receive's flush/activity interfaces but cannot call business. The codec's generated dependency file is checked for upper-layer state headers.

Temporary table configurations delete the first, middle, last, several, or all feature entries; they also remove configuration/work-parameter entries and reverse table order. Production files are never changed by the runner.

The harness exercises independent module initialization, FIFO allocation failure, partial-frame cleanup, rejection of new work after receive shutdown, complete-frame callback/context delivery, request encoding, fragmented byte reception through the receive work item, CRC, response matching, exception responses, permission checks, sparse feature reads, unchanged physical addresses, field masks, zeroing removed fields, failed-read cache preservation, and request ownership until the waiter consumes its result. Unclaimed frames must be discarded without changing request state, output buffers, feature caches, or wake counts; completed frame buffers and slots must be released, and a subsequent matching response must still complete the request.

Baud-rate cases cover a direct 40102 read, switching through unique enum rates,
skipping the duplicate default/9600 value, keeping the discovered rate,
restoring the entry rate after a full timeout, invalid register values, write
echo errors, and synchronizing the simulated sensor and serdev only after
40102 staging, the 40110=1 save acknowledgment, and a modeled reboot interval.

This does not test actual hardware, workqueue scheduling, or concurrent kernel execution. The tests link the production frame-dispatch implementation directly; no fallback-handler stub is used.

Sensor-setting cases cover sampling-rate and feature-mask limits, preserving
the alarm bits in register 40070, unchanged-value write suppression, readback
verification, rejected writes to read-only settings, and Modbus errors without
changing the caller's output. Sampling settings must not send a save/reboot command.

Startup serial configuration tests cover default properties, none/odd/even
mapping (the Linux enum order differs from the sensor enum), slave-ID bounds,
unsupported baud rates, logged defaults for missing/unreadable properties,
independent per-property fallback, invalid parity enums, controller
startup-rate rejection falling back to 9600, and target-first matching with
read-only discovery before any save/reboot. Unsupported `current-speed` values
are logged and default to 9600 instead of failing initialization. Tests cover controller
baud/parity failures, and recording a rounded actual host rate. Startup parsing
and configuration must not send any sensor commands. Reboot cases check that
both writes use the old baud rate, no verification frame precedes the reboot
deadline, cached features are invalidated, and missing save-table entries fail
before staging. Exceptions, lost/corrupt save acknowledgments, new-rate
verification failures, UART-setting failure, shutdown during the wait, and
staged-versus-active rate reads are covered. Sleep uses a controlled clock;
these tests do not establish the actual hardware boot time.

Reboot-verification cases model a 115200-to-9600 switch with the sensor becoming
ready later than the initial wait, a lost first verification response, perpetual
timeouts, exception responses, and shutdown between retries. Retrying must send
only 40102 reads, stop at the configured attempt limit, and never repeat either
staging or Flash save writes. Invalid/mismatched replies must still fail.

Isolation tests inject a late 40102 response while preparing a 40121 request
with the same response shape, verify recovery guards after timeout/partial
write, and check bounded idle waits under continuous noise and shutdown/signal
interruption. Receive tests occupy all four candidate slots with unfinished
frames, exercise both timer and per-byte expiration, and ensure a fragmented
valid frame remains usable within the low-baud wire-time budget. The shim uses
controlled jiffies; it does not validate real UART delivery latency.
# 连续原始流测试

测试直接编译 `raw_stream.c` 和实际 FIFO 接收器，覆盖逐字节拆包、坏 CRC、
坏尾标记、数据区含帧头/尾、重新同步、序号回绕/跳变和 IIO 推送失败。
连续模式控制测试覆盖无普通 ACK 的启动、1 Mbaud 切速、停止及恢复验证、
首包/收流超时、停止发送重试、信号打断、卸载，以及删除启动/停止表项时
禁止进入原始模式。共 12 组寄存器表变体使用 ASan/UBSan 运行。

恢复回归还覆盖：6 秒后首包正常到达、首包前卸载后固件迟到启动、
连续六次停止丢失后保留恢复状态、恢复期间普通读写禁止发送、后台再次
停止并只读确认、关闭请求模块后仍执行卸载确认，以及加载时从遗留
1 Mbaud 私有流恢复到不同的普通配置速率，全程不能误写 40102/40110。
有效采集和排空统计分别验证，候选 CRC 错误不冒充精确丢包数。
停止等待还覆盖新完整包唤醒和无新包的有界回退；这不能替代实机 RS485
方向控制和包间发送时序验证。
