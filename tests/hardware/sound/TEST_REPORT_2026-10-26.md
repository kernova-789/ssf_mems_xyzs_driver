# 4096 点原始声音功能与实机验证（2026-10-26）

## 实现与接口

已为 XYZS 增加一次性固定 4096 点、16 kHz 原始声音采集，并部署到 RK3588 开发板。
- 特征设备新增 `sensor_sound_capture`，只接受写入 1；必须先启用声音数据 buffer。
- 新建独立 IIO 设备 `ssf_mems_xyzs_sound`，不改变既有特征/振动扫描字段。XYZ 不创建声音设备。
- 由型号 ops 绑定声音采集；链路为 IIO 触发 → 采集锁 → 型号回调 → 40063 开启 → 独立声音解析 → 全批验收及普通通信确认 → IIO 发布。
- 先检查完整 64 包、从机、CRC、顺序与 4096 点，再发布。损坏的线级批次不发布声音片段。
- 每个样本保留 32 位有符号声音计数，另带 u16 包号、u16 包内序号、s64 主机推送时间；全通道 16 字节。无声压标定，不能当成 dB。
- 与振动/特征/配置/升级共用采集锁。等待在途操作收尾后执行；不改振动采样率或长度，不保存 Flash，不自动循环声音。
- 中断返回 EINTR，防止内核通过 SA_RESTART 重放触发。失败/卸载先有界收完在途数据并只读恢复通信。

使用说明：[声音采集接口](../../../iio/SOUND_CAPTURE.md)。采集工具：[board_sound_capture.py](board_sound_capture.py)。

## 手册依据与时钟

依据仓库 XYZS 使用说明书第 13 页的声音定长采集（4096 点 / 16K），以及第 16 页的 40063（写 1，发送完毕自动归零）。手册与实际板端数据共同核对。
名义声音覆盖 4096 / 16000 = 0.256 秒。触发、设备等待、传输与收尾耗时不能视为声音覆盖时长。
完整批次验收后才推入 IIO，因此 IIO timestamp 是推送时刻，不能按其间隔计算实际声学采样率；批内相对时间以样本位置 / 16000 解释。

## 验证范围

| 层次 | 检查 | 结果 |
| --- | --- | --- |
| 主机 | 协议模块独立编译、依赖边界、型号表变体完整矩阵 | 通过 |
| 主机 | 声音完整批、坏 CRC、缺包、重复/错误序号、错误总包数、超时、中断、停止与 IIO 丢样 | 通过 |
| 主机 | IIO 分配/buffer/注册失败回滚，声音 scan 对齐、符号与可选时间戳，触发门禁和 EINTR 转换 | 通过 |
| 主机 | 原有 100000 次发布 / 20000 次 buffer 重配回归 | 通过 |
| 主机 | 采集策略、手动窗口、状态锁及型号隔离（9784 项主用例检查） | 通过 |
| ARM64 | 目标内核 5.10.160，W=1 外部模块编译 | 通过 |
| 实机 | 模块卸载后全部 MEMS IIO 移除，再加载最终模块 | 通过 |
| 实机 | 两路各连续触发 3 个完整声音批次 | 通过 |
| 实机 | 新模块的双路振动/特征 30 秒回归及二进制/持续推进检查 | 通过 |

物理环境：Embedfire LubanCat-5IOB，Debian 11 / 5.10.160 aarch64；两路固件均读回 5100，UART4/UART7，128000 bit/s，振动采样率索引 6。
最终部署模块 SHA256：`d5433df0affefc17440e3027e025ef05e9e22d923221871394c6d641d9a5dcce`。
驱动代码文件清单摘要：`b77ff5a512620450522764168a3f944f92b64401d627bb05e1c40504247fa858`；详细逐文件摘要在本机 build/sound-4096/source-sha256.txt。

本机内核配置的编译器能力选项与板端栈保护配置存在差异。核对 `/proc/config.gz` 后，使用与板端一致的 per-task SYSREG 栈保护参数编译，没有关闭栈保护或替换板端内核：

```sh
make -C /home/u-yoimiya/rk3588/kernel M="$PWD" ARCH=arm64 \
  CROSS_COMPILE=aarch64-linux-gnu- CONFIG_STACKPROTECTOR_PER_TASK=y \
  CONFIG_CC_HAVE_STACKPROTECTOR_SYSREG=y W=1 modules
```

这些参数属于本次目标板配置，不能对不同内核无条件套用。

## 最终模块的声音数据

| 端口 | 批次 | 点数 | 包数 | 总耗时 s | PCM 最小/最大 | 唯一值数 |
| --- | --- | --- | --- | --- | --- | --- |
| RS485-1 | 1 | 4096 | 64 | 6.599 | -967168 / 921088 | 1646 |
| RS485-1 | 2 | 4096 | 64 | 6.423 | -1358336 / 1204736 | 1996 |
| RS485-1 | 3 | 4096 | 64 | 6.410 | -1515520 / 1277440 | 2150 |
| RS485-2 | 1 | 4096 | 64 | 6.905 | -1214464 / 1303552 | 2009 |
| RS485-2 | 2 | 4096 | 64 | 6.407 | -1770496 / 1211904 | 2038 |
| RS485-2 | 3 | 4096 | 64 | 6.421 | -2037248 / 1573888 | 2229 |

上述六批合计 24576 点；每批的 CRC、序号、ignored bytes 和 IIO buffer drops 均为 0；error=0。
最终声音测试前后的 UART fe/brk/oe/pe 无新增，完整内核增量未见声音通信错误；随后特征读取和振动回归正常。
另保留早期模块的两路各 3 批短测（rs485-1/、rs485-2/），验收主结果以上述 final-rs485-* 为准。

## 数据与操作

板端主目录：`/data/ssf-tests/20261026-sound-4096/`，位于此前挂载的 1T 盘。
- `ssf_mems.ko`：最终已加载模块。
- `final-rs485-1/`、`final-rs485-2/`：最终批次 IIO 二进制、PCM 二进制、WAV、schema 和 SHA256/状态元数据。
- `vibration-feature-regression/`：同一模块的双路振动/特征 30 秒结果。
- `kernel-before-final-sound.log`、`kernel-after-final-sound.log`、UART 前后计数、`final-state.txt`：板端证据。
- 本机主机回归和构建日志：`build/sound-4096/`。

从仓库根目录运行，输出必须是新目录：

```sh
sudo python3 tests/hardware/sound/board_sound_capture.py \
  /data/mems-sound/new-session --serdev serial0-0 --captures 3
```

开发板独立副本也可直接运行：

```sh
cd /data/ssf-tests/20261026-sound-4096
sudo python3 board_sound_capture.py /data/mems-sound/new-session \
  --serdev serial1-0 --captures 3
```

结束时两路在线、feature 模式，全部 buffer 已关闭，声音 captures 各为 3、error=0。
本次是声音功能短测，不等于声音 45 分钟长测、声学精度校准、真实掉电或新的全面通信稳定性验收；此前长测发现的间歇通信问题仍需独立定位。
