# Protocol tests

Named-register cases check table-wide unique names, address/name equivalence,
normal and caller-locked access, invalid/missing names, permissions, private
responses and deleted entries across all table variants. They use the production
request executor and do not introduce a second address mapping.

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
# 定长原始批次测试

直接编译 raw_stream.c 和实际 FIFO 接收器，覆盖拆包、CRC、尾标记、
含帧头/尾的载荷、重同步、首包非零、序号跳变、越界包及 IIO 推送失败。
控制测试使用 128000，验证 FC03 40060 无普通应答、每批 16 包/1024 XYZ、
窗口内多批、非零自定义长度改为零且不改其他工作参数。

故障用例覆盖首包超时、末包丢失、中间包丢失、触发发送失败、
6 秒后首包、信号打断、首包前和上传中卸载。卸载收尾只读确认，
不发送 40058/40059，不切到 1 Mbaud，不写 40102/40110。
确认失败时普通写请求被恢复门禁拒绝，再次只读确认后才能恢复业务。

共 13 组寄存器表变体使用 ASan/UBSan；删除旧连续启停表项不再影响定长采集，
删除 40060 则拒绝触发。模拟测试不能代替实机 UART/RS485 方向时序和 IIO 验证。
