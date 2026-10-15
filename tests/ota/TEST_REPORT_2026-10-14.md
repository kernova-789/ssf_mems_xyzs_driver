# OTA 离线验证记录（2026-10-14）

这是当时的测试记录，检查数量与构建产物对应旧版本。
2026-10-15 澄清：仓库 bin 属于其他传感器，不是本设备升级候选；当前没有适用镜像。
现行测试全部使用合成镜像，使用要求见 [OTA.md](../../protocol/firmware/OTA.md)。

此次只修改 Linux 驱动及测试/文档，没有修改传感器固件，没有通过 SSH 操作开发板，
没有执行任何实机升级、重启或参数写入。

## 验证结果

| 检查 | 结果 |
| --- | --- |
| `python3 tests/ota/run_tests.py` | 22578 项检查通过，ASan/UBSan，真实 pthread 锁 |
| `python3 tests/acquisition/run_tests.py` | 5060 项检查通过 |
| `python3 tests/iio/run_tests.py` | 属性测试及 100000 次发布 / 20000 次缓冲区重配置通过 |
| `python3 tests/protocol/run_tests.py` | 13 种表变体全部通过，基线 121822 项检查 |
| ARM64 Linux 5.10.160，`W=1` 外部模块编译 | 成功，无编译警告 |
| 最终编译快照与当前生产 C/H、Makefile 比较 | 全部一致 |
| `git diff --check` | 通过 |

OTA 测试包含最大镜像和边界长度、模拟暂存 Flash 内容逐字节比较、无 CRC 应答的
分片/错误地址/错序、普通包重传、末包不可重传、信号转换为 EINTR、并发升级拒绝、
等待原始窗口结束以及该等待期间卸载。末包不确定时验证后台和配置通信会被暂停。

## 编译产物

- 模块：`build/ota-2026-10-14.ZMpY2J/ssf_mems.ko`
- 编译日志：`build/ota-2026-10-14.ZMpY2J/build.log`
- vermagic：`5.10.160 SMP mod_unload aarch64`
- SHA256：`bcd42294efa2b5529146294727bb8e9d28b77c37a3e10530612a91e0565ab597`

使用临时源码快照构建；未覆盖根目录已有的模块、此前实机长测模块，未安装到 NFS。

## 尚未验证及镜像限制

尚未验证真实 RS485 上传、Flash 耗时、Bootloader 复制/掉电行为和重启等待余量。
模拟及编译通过不代表已经完成实机 OTA 验收。

参考目录现有 bin 长 113524 字节，初始栈指针为 `0x2001C850`、复位入口为
`0x0800020D`。驱动按 Bootloader 的 `0x08020000` 应用区检查，将其以 ENOEXEC 拒绝，
没有尝试发送。该文件属于其他传感器，这些数值不能用来判断本设备固件工程有错误。
下一步实测需要固件方提供匹配硬件及 Bootloader 的应用 bin，
并确认设备已有对应 Bootloader、总线上只有目标传感器。

固件没有整镜像哈希、签名、读回验证或可靠回滚接口；驱动的 `complete` 仅确认
分包计数应答与重启后版本寄存器可读，不能据此声称整幅 Flash 已校验。
操作说明见 [OTA.md](../../protocol/firmware/OTA.md)。
