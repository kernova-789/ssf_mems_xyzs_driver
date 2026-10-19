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

Polling cases cover the default fixed 1000 ms behavior, a custom fixed period's
actual read-start intervals, adaptive acceleration/deceleration and bounds,
read-time headroom, and mode/interval changes during the real acquisition loop.
They check that 30-second probing requires low repetition and a changed sample,
constant samples never accelerate, and probing preserves an unfinished 32-comparison window;
normal raw windows preserve learned periods, reconnection retains configuration,
inactive-mode edits preserve statistics, and invalid bounds never change state.
Configuration tests verify no sensor commands, no restart of the feature window,
same-value write suppression, and range/shutdown errors.

Sensor-setting cases check offline/shutdown rejection, request timeouts,
cache invalidation after successful writes or lost acknowledgments, and resetting
the polling policy after a successful configuration change. Successful sampling
rate changes become the reconnection target; failed changes leave it unchanged.
Connection cases check device-tree-derived startup hints and the 128000 baud / index 6 operational defaults, downgrading a
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

运行时控制覆盖三种模式、离线配置、空参数/范围/关闭检查、同值写不唤醒，
以及 feature→raw→feature→alternating 的线程内切换；切换不重新匹配或调速。
纯原始模式检查零特征读写、恢复门禁和 IIO 满队列后继续采集；100 ms 短交替窗口
检查不会被 1000 ms 特征周期拖长。设备树 raw-duration-ms=0 仍默认仅特征，
但保存 5000 ms 原始窗口供未来运行时切换。有校验时仅允许特征模式。

另用真实 pthread mutex 验证生产 `get_status()` 和 `get_config()`：持有完整采集事务锁时，
在线状态、周期、样本年龄和主机配置仍可读取，调用受 5 秒测试超时保护。
替代型号表将默认工作波特率改为 115200，验证初始化确实采用表中目标。

替代型号测试还同时初始化 XYZ 和工作速率不同的 XYZS，确认目标配置按实例读取。
XYZ 的自适应比较忽略 union 未使用区域，且能识别位移成员变化。
