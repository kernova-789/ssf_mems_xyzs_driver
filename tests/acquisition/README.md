# Acquisition tests

Run `python3 tests/acquisition/run_tests.py` from the repository root.

The harness compiles the production acquisition state machine and policy with
AddressSanitizer/UndefinedBehaviorSanitizer, a deterministic clock, and protocol,
IIO and thread substitutes. It exercises startup delay, baud matching/switching,
verification before publishing, fixed-period bounds and actual 1000 ms read-start
intervals for changing and repeated samples, read-time headroom,
padding-independent comparisons, timestamp wraparound, communication-failure
thresholds, stale cache invalidation, sensor resets, exponential retry caps,
Modbus exceptions, manual baud changes, lost write acknowledgments, buffer errors,
low-baud response budgets, thread creation failure and shutdown guards.

Sensor-setting cases check offline/shutdown rejection, request timeouts,
cache invalidation after successful writes or lost acknowledgments, and resetting
the polling policy after a successful configuration change.
Connection cases check device-tree-derived targets and the 9600 baud / index 6 defaults, downgrading a
sensor that still holds the former maximum settings, configuring
sampling after baud verification and before publishing, skipping already-matching
settings, retrying a lost sampling acknowledgment without repeating the write,
reapplying settings after a simulated reset, and shutdown during configuration.

These are deterministic state-machine tests. They do not model concurrent kernel
scheduling, UART/firmware timing, or real IIO buffer consumers. Build the module
against the target kernel and validate the device's 40102 change timing on hardware.
# 交替采集测试

增加原始/特征周期切换验证，以及时段属性缺失、读取失败、范围非法和
关闭原始采集时的默认值/取值验证。
原始恢复失败时必须立即离线退避；下一轮先恢复模式，再重新匹配。
测试禁止恢复未确认期间读取/发布特征，且不能因此重复保存波特率。
