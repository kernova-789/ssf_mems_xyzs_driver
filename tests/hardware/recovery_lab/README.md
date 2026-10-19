# 独立连续回报实验

`mems_continuous_lab.c` 是临时 serdev 实验模块，不链接正式驱动，不发布 IIO。
测试只用于温振声 XYZS、地址1、无校验、普通速率128000、采样索引6的已核实现场。
正式驱动保持定长采集架构。

模块按 MEMS2.0/User/samp.c 的分支发送 FC03/线上57（显示40058），立即将主机切到
1 Mbaud，最多等待45秒首包，从首包开始记录指定时长。每路独立保存接收字节、
CRC、包序号、首包等待和包间隔；默认三轮，每轮实际收流60秒。
启动和停止均没有普通 Modbus 回显，启动等待期间不发送探测请求。
停止最多发送三次 FC06/线上58=1，以安静及128000下读回40102=8共同确认；
失败即结束该实例后续轮次，记录现场，不自动保存/重启。

在新的构建目录复制 C 文件和 Makefile，用目标板内核交叉编译：

```sh
make -C /path/to/kernel M=/absolute/lab/build ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- W=1 modules
```

`board_continuous_lab.py` 会卸载 `ssf_mems`、加载实验模块并保存两路 debugfs 字节和日志。
它仅在四个 IIO buffer 都关闭时运行，结束后有意保留实验模块，避免在停流未确认时
直接加载正式定长驱动。应依据 `CL FINISH` 的 restored 字段及 `CL STOP VERIFIED` 核实恢复。
核实后卸载实验模块，加载原正式模块。

`recover_only=1 port=serialX-X` 是独立、显式的故障恢复动作：仅在1 Mbaud读回
持久波特率索引8后保存/重启一次，15秒后验证128000。它不属于采集测试，不能将
恢复后的成功当作之前停流已经成功，也不能自动循环使用。

不修改传感器固件、设备树、主机RS485方向或波特率枚举。字节文件由
`analyze_continuous_lab.py` 独立重算 CRC 和包序号；停止边界和轮间数据不能混算。
包号连续只能证明线上交付顺序，不能证明传感器内部没有漏采或覆盖数据。
