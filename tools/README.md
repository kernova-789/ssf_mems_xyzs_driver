# 开发板 IIO 读取测试

`iio_read.c` 是仅依赖 libc 的用户态 C 程序，不需要 libiio。
默认在 `/sys/bus/iio/devices/` 下按名称查找 `ssf_mems_xyzs`，
枚举所有 `in_*_raw` 通道，每轮读取并用 `printf` 打印。
默认每轮结束后等待 100 ms，持续运行，按 Ctrl+C 退出。

## 编译与运行

在开发主机上为 ARM64 开发板交叉编译，默认使用静态链接，
避免工具链与开发板的 glibc 版本不同导致 `GLIBC_2.34 not found` 等错误：

```sh
make -C tools CROSS_COMPILE=aarch64-linux-gnu-
file tools/iio_read
cp tools/iio_read /home/u-yoimiya/nfs/
```

开发板挂载对应的 NFS 目录、加载驱动并确认设备绑定后运行
（将下面的挂载路径替换为开发板上的实际路径）：

```sh
/mnt/nfs/iio_read
```

也可以复制 `iio_read.c` 到开发板，直接编译：

```sh
gcc -O2 -Wall -Wextra -o iio_read iio_read.c
./iio_read
```

静态链接需要交叉工具链提供 `libc.a`；`file tools/iio_read` 应显示
`ARM aarch64` 和 `statically linked`。若希望使用与目标系统匹配的
动态库，可先运行 `make -C tools clean`，再编译时指定 `LDFLAGS=`。
切换本机编译与交叉编译时也先运行 `make -C tools clean`。

## 参数与输出

```sh
./iio_read -h
./iio_read -i 500                         # 每轮结束后等待 500 ms
./iio_read -d /sys/bus/iio/devices/iio:device0
./iio_read -n 10                          # 打印 10 轮后退出
```

存在 `*_scale` 的通道会同时打印 `raw`、`scale` 和 `scaled = raw * scale`。
比例在程序启动时读取一次；`startup_flags` 没有 scale，打印原始值和
`running[X=... Y=... Z=...]`。依据 MEMS2.0 源码，这些位表示去直流后的
加速度峰值是否达到 0.06 g，不表示三轴测量是否被禁用。
换算值遵循驱动的 IIO 单位：加速度为 m/s²、速度为 m/s、温度为毫摄氏度
（例如 `scaled=25000` 表示 25 °C）。声音通道为 dB，过零率为百分数，
谱质心为 Hz，峭度和谱通量无量纲。

每轮还打印 `sensor_online`、`sensor_poll_interval_ms` 和
`sensor_sample_age_ms`。启动尚未取得样本、数据过期或通信恢复期间，
读取可能显示 `unavailable (No data available)`；程序会继续读取。
`-n` 统计读取轮数，包含读取失败的轮次，不代表有效样本数。

程序读取的是驱动后台采集的最新缓存，读取间隔不会改变驱动采集周期，
当前驱动固定每 1000 ms 读取一次寄存器，以匹配实测约 4.4～4.8 秒的批次更新。
因此相邻轮次可能打印相同样本。各 sysfs 属性是逐个读取的，
同一轮中的不同通道可能跨越一次缓存更新。此程序适合观察通道数据；
需要严格同步的完整帧时，应使用驱动的 IIO buffer 接口。

## 特征值长时间重复的排查

新驱动提供采样配置和固件版本属性，程序启动时查询并打印一次。
这些属性会发送 Modbus 请求，旧驱动没有相应属性时程序跳过它们。
当前驱动启动目标波特率采用设备树 `current-speed`，未配置/不可用时为 9600，
自动连接/重连会匹配并应用该目标（在线手动设置后使用手动目标）；采样率仍为索引 6
（5333.4 Hz），已匹配的值不重复写入。下面的索引 6 命令也可用于较旧模块，
手动选其他采样率后，下一次重连会重新设为 6。

MEMS2.0 的 `F_SampMode()` 不是每次收到读取命令就采样：它先采集一大批
数据，再顺序处理三轴滤波、FFT、声音和温度，最后才调用
`modebusInBuff()` 更新 40001～40029。Modbus 可以在这期间反复返回同一组
特征，因此 `sensor_sample_age_ms` 很小只证明主机最近读过寄存器。

源码默认采样率索引 0，对应 533.34 Hz，`sampTimeCnt1=50`。
一次原始采集循环处理 `max(1600 * 50, 16000) + 3000 = 83000` 个 XYZ 点，
配置频段又需处理 83000 点。索引 6 对应 5333.4 Hz，抽取因子降到 5，
上述两个循环分别缩短到 19000 和 11000 点，有助于缩短特征更新等待。
实际更新时间还取决于 MCU 的 SPI、PSRAM 和 DSP 运算速度，不能由主机
轮询周期保证。配置采样长度 40053 主要用于原始数据返回；
这份固件的特征算法固定使用 1600 点，不应套用 `采样长度/采样率` 推断
整个特征计算周期。

### 为什么提高配置采样率不一定提高更新频率

[IIS3DWB 原始输出率](https://www.st.com/resource/en/datasheet/iis3dwb.pdf)
为约 26.667 kHz，40052 主要控制固件的滤波和抽取因子，
不是更改芯片的原始输出率。当前固件每批采集的原始 XYZ 点数为
`max(1600 * sampTimeCnt1, 16000) + 3000`：

| 索引 | 配置采样率 | 抽取因子 | 原始 XYZ 点/批 | 普通特征有效点/轴 |
|---|---:|---:|---:|---:|
| 0 | 533.34 Hz | 50 | 83000 | 1600 |
| 6 | 5333.4 Hz | 5 | 19000 | 1600 |
| 9 | 26667 Hz | 1 | 19000 | 1600 |

所有原始点先存入 PSRAM。高频加速度路径滤波后每 2 点保留 1 点，低频速度
路径每 10 点保留 1 点；两条路径各轴最终也使用 1600 个有效点。前 3000 点
用于滤波稳定过程，不作为上述有效点。普通特征路径读取
`1600 * sampTimeCnt1 + 3000` 点，每一点都经过低通滤波，再按配置因子抽取
1600 点，计算峰值、峰峰值、RMS、峭度和频域积分得到的速度 RMS。
所以从索引 6 改为 9，不会多采五倍的原始点，也不会输出五倍的特征样本。
改变的是抽取间隔和普通特征的标称时间窗，后者从约 0.30 s 缩短到约 0.06 s。

索引 6 和 9 的原始采集、高低频滤波及大部分特征运算量相同，声音分析还要
单独采集 4096 点并计算频谱，全部处理后才更新寄存器。主机只读取特征寄存器，
这些内部波形点并不会逐点传给 Linux；提高波特率只会缩短寄存器传输时间。
这解释了两档更新时间可能接近；各阶段实际耗时需要固件计时才能确认。

索引 6 适合当前普通振动/敲击观察，但其低通截止约 2.67 kHz，无法保留完整的
1～5.3 kHz 高频段。需要完整高频段时应使用更高配置，并检查硬件滤波是否确实
切换：索引 9 分支设置了 `CTRL1_LPF2_EN=0xA4`，但特征模式随后仍固定写入
`CTRL1_XL=0xA6`，且该分支未更新 `CTRL8_XL`，因此寄存器索引 9 本身不能保证
实际硬件低通已关闭。这是传感器固件的配置细节，当前 Linux 驱动没有修改它。

在开发板重载新模块，等待 `sensor_online` 为 1，然后执行：

```sh
# 自动查找正确的 IIO 设备目录。
for candidate in /sys/bus/iio/devices/iio:device*; do
    if [ "$(cat "$candidate/name" 2>/dev/null)" = "ssf_mems_xyzs" ]; then
        sensor_iio_dir="$candidate"
        break
    fi
done
test -n "$sensor_iio_dir" || exit 1

cat "$sensor_iio_dir/sensor_firmware_version"
cat "$sensor_iio_dir/sensor_sampling_rate_index"
cat "$sensor_iio_dir/sensor_feature_enable"

# 在 root shell 中执行；保存原采样率便于需要时恢复。
original_rate_index=$(cat "$sensor_iio_dir/sensor_sampling_rate_index")
echo 6 > "$sensor_iio_dir/sensor_sampling_rate_index"
echo 63 > "$sensor_iio_dir/sensor_feature_enable"
cat "$sensor_iio_dir/sensor_sampling_frequency"

# 观察约 30 秒，比较静止和持续晃动时的峰值、峰峰值、RMS 及速度。
./iio_read -d "$sensor_iio_dir" -i 500 -n 60
```

这会把配置采样率改为 5333.4 Hz，并开启六个频段特征，保留报警配置。
固件会自行持久化采样率，重载 Linux 模块不会恢复它；需要恢复时执行
`echo "$original_rate_index" > "$sensor_iio_dir/sensor_sampling_rate_index"`。
不要频繁切换，以免重复写入传感器 Flash。

配置频段计算会减去均值，慢速旋转造成的重力方向变化不一定保留下来；
高频 RMS 则明确过滤 1 kHz 以下的运动。验证时使用持续晃动或稳定的
振动源，优先看普通 `peak`/`peak_to_peak` 通道。加速度特征在传感器端
转换为 `uint16_t(value * 100)`，不足 0.01 g 的正数会截断为 0，驱动无法
从该寄存器恢复更细的数值。
