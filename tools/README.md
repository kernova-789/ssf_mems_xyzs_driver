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
`running[X=... Y=... Z=...]`。这些位表示 XYZ 运行判定，不表示三轴测量是否被禁用。
换算值遵循驱动的 IIO 单位：加速度为 m/s²、速度为 m/s、温度为毫摄氏度
（例如 `scaled=25000` 表示 25 °C）。声音通道为 dB，过零率为百分数，
谱质心为 Hz，峭度和谱通量无量纲。

每轮还打印 `sensor_online`、`sensor_poll_interval_ms` 和
`sensor_sample_age_ms`。启动尚未取得样本、数据过期或通信恢复期间，
读取可能显示 `unavailable (No data available)`；程序会继续读取。
`-n` 统计读取轮数，包含读取失败的轮次，不代表有效样本数。

程序读取的是驱动后台采集的最新缓存，读取间隔不会改变驱动采集周期，
当前驱动默认每 1000 ms 读取一次特征寄存器，以匹配实测约 4.4～4.8 秒的批次更新。
用户可通过 IIO 修改固定周期或选择自适应模式，见 [轮询配置](../iio/ATTRIBUTES.md#4-固定轮询与自适应轮询)。
因此相邻轮次可能打印相同样本。各 sysfs 属性是逐个读取的，
同一轮中的不同通道可能跨越一次缓存更新。此程序适合观察通道数据；
需要严格同步的完整帧时，应使用驱动的 IIO buffer 接口。

## 特征值长时间重复的排查

新驱动提供采样配置和固件版本属性，程序启动时查询并打印一次。
这些属性会发送 Modbus 请求，旧驱动没有相应属性时程序跳过它们。
当前驱动以设备树 `current-speed` 作为初次匹配提示，未配置/不可用时以 9600 开始，
首次连接采用型号表目标，当前为 128000、采样率索引 6（5333.4 Hz），
已匹配的值不重复写入。成功手动设置后，重连保留用户的波特率和采样率目标；
卸载后目标重置为型号表默认值。下面的索引 6 命令也可用于较旧模块。

传感器特征寄存器可能多次返回相同结果，`sensor_sample_age_ms` 很小只说明主机
最近读过寄存器，不代表设备刚完成新测量。主机轮询周期、UART 波特率和传感器
采样率是三个独立参数，提高其中一个不保证特征刷新更快。

遇到长时间重复时，先确认 `sensor_online`、采样率读回、通道使能及通信日志，
再使用稳定振动源观察数值变化。采样长度寄存器不代表主机轮询周期，也不改变
本驱动定长原始批次的 1024 组 XYZ。

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
Linux 模块重载后会重新应用型号默认目标，不能将重载当作恢复测试前配置。
要恢复测试前的值，应在驱动在线后执行
`echo "$original_rate_index" > "$sensor_iio_dir/sensor_sampling_rate_index"`。
不要频繁切换，以免重复写入传感器 Flash。

验证时使用适合目标频段的稳定振动源，优先观察普通 `peak`/`peak_to_peak` 通道。
特征加速度编码分辨率为 0.01 g，驱动不能从该寄存器恢复更细的数值。
