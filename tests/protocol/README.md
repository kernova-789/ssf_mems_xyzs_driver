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

The harness exercises independent module initialization, FIFO allocation failure, partial-frame cleanup, rejection of new work after receive shutdown, complete-frame callback/context delivery, request encoding, fragmented byte reception through the receive work item, CRC, response matching, exception responses, permission checks, sparse feature reads, unchanged physical addresses, field masks, zeroing removed fields, failed-read cache preservation, and request ownership until the waiter consumes its result. Unclaimed frames must be discarded without changing request state, output buffers, feature caches, or wake counts; consumed prefixes must leave the bounded receive window, and a subsequent matching response must still complete the request.

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
interruption. Receive tests inject multiple unfinished frame prefixes, exercise
both idle-window and independent frame-head expiration, and ensure a fragmented
valid frame remains usable within the low-baud wire-time budget. The shim uses
controlled jiffies; it does not validate real UART delivery latency.
# 定长原始批次测试

直接编译 raw_stream.c 和实际 FIFO 接收器，覆盖拆包、CRC、尾标记、
含帧头/尾的载荷、重同步、首包非零、序号跳变、越界包及 IIO 推送失败。
控制测试使用 128000，验证 FC03 40060 无普通应答、每批 16 包/1024 XYZ、
窗口内多批、非零自定义长度改为零且不改其他工作参数。

故障用例覆盖首包超时、末包丢失、中间包丢失、触发发送失败、
6 秒后首包、信号打断、首包前和上传中卸载。卸载收尾只读确认，
保持工作波特率，不写 40102/40110。
确认失败时普通写请求被恢复门禁拒绝，再次只读确认后才能恢复业务。

共 17 组寄存器表变体使用 ASan/UBSan；删除 40060 则拒绝触发。
每组均验证手册中的 40058、40059、40071 描述仍可按地址/名称查到，
且地址、权限、数据类型和响应格式保持完整；普通读取拒绝私有 XYZ 启动响应。
波特率匹配覆盖 1000000 下读到不同暂存配置时仍保留可通信速率，且不写配置。
新增变体迁移配置、定长触发、长度和波特率表项的显示地址，
验证业务层仅凭稳定名称仍可完成配置读写、16 包采集及波特率扫描/保存/重启验证。
另两组分别打乱长度高低字的显示地址、线上地址，确认触发和长度写入前即拒绝。
范围用例验证索引 9 可完成定长采集，索引 10 在发送触发前被拒绝。
模拟测试不能代替实机 UART/RS485 方向时序和 IIO 验证。

新增型号表变体覆盖有符号温度的比例换算、换算越界时保留缓存，
以及特征使能位布局变化后仍保留其他配置位。原始解析器只依赖样本回调，
依赖边界检查禁止其直接调用 IIO。缺少回调时统计推送失败，避免空指针调用。
帧表/寄存器表/块表只在 对应的 `sensor/xyz.c` / `sensor/xyzs.c` 实例化一次。

缩短型号的波特率编码表后，测试确认不写未提供的编码、扫描不越界，失败后恢复入口速率。

新增启动期型号表校验用例覆盖空编码表、默认值/范围越界、非法位布局、重复名称和地址、
重复或非法字段映射。字段描述由公共 schema 生成，表变体测试继续验证删除和迁移行为。
同值波特率用例验证只读确认、不保存、不重启；暂存值不同仍完整保存，读回错误不盲目成功。
抗噪接收使用真实 FIFO/解析工作项，覆盖多个伪长帧、最大帧、合法嵌套帧、重复交付和独立帧头过期。

# 两型号架构回归

型号变体现在修改临时副本的 `sensor/xyzs.c`，共用 RTU 帧表不随型号改变。
`python3 tests/protocol/run_tests.py xyz` 可单独运行 XYZ V2.4 夹具；不带参数运行全部测试。
XYZ 夹具独立列出手册全部 19 个寄存器的成员偏移，验证与 XYZS 同地址的不同语义、
型号回调绑定、单次连续读 19 项、有符号温度 ×10→×100、越界/通信失败时保留缓存，
以及显式 1024 点长度写入、16 包采集、丢包恢复和不写声音寄存器/Flash。
查询显式接收型号，不使用可变的全局当前型号。
