# 传感器型号层

`core/probe` 根据设备树 `compatible` 选择只读 `struct ssf_mems_sensor`，
在任何模块初始化前设置 `data->sensor` 和 `data->sensor_ops`。
型号指针属于设备实例，没有全局“当前型号”，可以同时绑定不同型号。
不会通过试读寄存器自动猜型号，也不会在 probe 中发送传感器命令。

| 型号 / 手册 | compatible | 特征寄存器 | 特征 IIO / 原始 IIO |
| --- | --- | --- | --- |
| SSF-MEMS-XYZS，20260114 手册 | `sange-cbm,ssf-mems-xyzs` | 40001–40029，29 项 | `ssf_mems_xyzs` / `ssf_mems_xyzs_raw` |
| SSF-MEMS-XYZ，V2.4 手册 | `sange-cbm,ssf-mems-xyz` | 40001–40019，19 项 | `ssf_mems_xyz` / `ssf_mems_xyz_raw` |

## 文件和依赖

- `xyz.c`、`xyzs.c`：各自的寄存器表、块范围、型号参数和采集回调。
- `xyz.h`、`xyzs.h`：各自的特征结构体。XYZ 包含三轴位移峰峰值，XYZS 包含高低频、声音和运行标志。
- `sensor_data.h`：原始 XYZ 样本和特征快照容器。快照使用 union；只有当前型号对应的成员有效。
- `sensor_fields.h`：公共特征语义编号、单位类别和偏移描述类型，不绑定任何型号的布局。
- `sensor.c`：显式传入型号的查表、能力查询和启动校验。
- `features.c`：供型号回调复用的连续分段读取和整数解码机制。

上层采集通过以下函数指针调用型号接口：

```c
ret = data->sensor_ops->read_features(data, timeout_ms);
ret = data->sensor_ops->collect_raw(data, duration_ms);
```

两个型号各自提供回调，相同机制共用。`protocol/` 负责串口、Modbus 请求、
总线模式、缓存同步和原始包解析；采集线程负责调度；IIO 负责用户态表现。
增加型号无需在采集线程中增加型号分支。

## 寄存器与结构体的一一映射

每个特征寄存器的 `ssf_reg_desc` 同时包含：

- 显示地址、线上地址、权限和有无符号类型。
- `feature_field`：用于 IIO 查找的稳定特征语义编号。
- `field.offset`、`field.width`、`field.scale`：该型号快照成员的位置、宽度和公开单位类别。
- `value_mask`、`feature_multiplier`、`feature_divisor`：位过滤及规范单位换算。

例如 XYZ 的温度映射如下（见 `xyz.c`）：

```c
.display_reg = 40019,
.type = SSF_REG_S16,
.feature_field = SSF_MEMS_FIELD_TEMPERATURE,
SSF_MEMS_FEATURE_MEMBER(xyz.temperature_x100, TEMPERATURE),
.value_mask = 0xffff,
.feature_multiplier = 10,
```

`SSF_MEMS_FEATURE_MEMBER` 使用 `offsetof(struct ssf_mems_sensor_data, member)`
和成员 `sizeof` 生成映射，直接嵌入寄存器表，不维护第二份偏移表。
解码先解释寄存器符号，再做整数比例换算并检查范围。
XYZ 的 40019 为有符号 ℃×10，缓存保存 ℃×100；三轴位移保留 µm，IIO scale 为 0.000001 m。

连续读取由特征块的范围和寄存器表决定：完整型号表分别一次读 19 / 29 个寄存器；
表存在空洞时按连续显示地址和线上地址分段。所有段成功才加锁替换缓存，
通信或换算失败保留之前的完整缓存。稀疏表缺失成员在成功快照中置零。

初始化检查编码范围、默认参数、轴顺序、块描述、名称和地址唯一性，
以及特征成员偏移、宽度、范围和重叠。完整映射还由独立手册夹具验证。

## IIO 与自适应轮询

IIO 从公共通道目录中选择当前型号有映射的通道，使用相同偏移描述打包。
XYZS 保持原有 29 个特征的名称、顺序、存储宽度和时间戳索引 29；
XYZ 有 19 个特征，时间戳索引 19，增加 `in_distance_{x,y,z}_peak_to_peak_*`，
不会创建声音、高低频和运行标志通道。
XYZ 的扫描顺序为温度、XYZ 加速度峰峰值、峰值、有效值、峭度、速度有效值、位移峰峰值。
全部通道不带时间戳为 38 字节，带时间戳为 48 字节，应用仍应读取 scan_elements 计算布局。

自适应重复比较只遍历当前型号映射的成员，忽略结构体填充和 union 未使用空间；
XYZ 的位移变化也参与重复率判断。

型号缺失的参数开关、特征使能、固件版本及 OTA 属性不在 XYZ 上创建，
协议配置接口返回 `-EOPNOTSUPP`。XYZ 手册只展示升级工具，没有确认本仓库 OTA 的镜像布局、
私有升级协议和版本确认寄存器，因此该型号不启用现有 XYZS OTA。

## 定长原始采集

两款型号回调共用 `ssf_mems_raw_collect_finite()` 的总线独占、收流和只读恢复机制。
每批为 1024 组同步 XYZ，触发地址来自各自的 `start_custom_length` 表项（40060）。
仅通过 FC10 修改 40061/40062 并读回，不借用 XYZS 的十参数工作块：

- XYZS 保持原有已验证行为：长度编码为 0，固件默认输出 1024 组。
- XYZ 显式写入长度 1024（高字 0、低字 1024），依据 V2.4 的 U32 长度定义。

共用接收格式为 `0x15 + BE32 包号 + 64×XYZ×BE16 + CRC16 + 0x17`，
每批 16 包。两只 XYZ V2.4 实机已确认包头、包尾、CRC 覆盖范围及长度 1024 的行为。
实测 XYZ 在高采样率、低线速下会输出序号跳变及 CRC 错误：128000/5333.4 Hz
只收到 0、3、6、9、12、15 六包，前五包 CRC 错误，独立串口抓包也能复现。
128000/533.34 Hz 和 128000/1333.35 Hz 均得到完整的 16 包。

XYZ 型号表用 `raw_min_baudrates` 描述各采样率的原始数据最低线速：
按每 64 组 392 字节、8N1 加 20% 余量计算。共用采集流程在写长度和触发之前
检查该表，线速不足返回 `-EOPNOTSUPP`，保留特征缓存，不擅自改变用户采样率。
XYZ 驱动首次工作配置为 128000/1333.35 Hz（索引 3）；这是驱动工作参数，
并非手册出厂默认的 9600/26667 Hz（索引 9）。出厂默认组合可读特征，不能定长采集。
高线速条目是吞吐预算，不代表已经完成该档位的硬件可靠性验证。
XYZS 未设置此表，沿用原有定长行为和默认采样率，不增加此限制。
详见 [原始采集流程](../protocol/raw/RAW_CAPTURE.md)。

## 手册依据与验证

- [SSF-MEMS-XYZ-V2.4说明书](../SSF-MEMS-XYZ-V2.4说明书.pdf)，PDF 第 8–11 页寄存器表。
- [SSF-MEMS-XYZS 使用说明书 20260114](../三轴MEMS温振声传感器（SSF-MEMS-XYZS）使用说明书20260114.pdf)，PDF 第 14–20 页寄存器和数据格式。

XYZS 沿用仓库原有规范单位和 ABI；其温度符号和声学字段与手册口径的差异，
继续按 [IIO 单位与测量边界](../iio/ATTRIBUTES.md#12-单位与测量边界) 说明，未随本次重构改变。

```sh
python3 tests/protocol/run_tests.py
python3 tests/acquisition/run_tests.py
python3 tests/iio/run_tests.py
python3 tests/ota/run_tests.py
```

协议测试包含 XYZ 独立映射、单次连续读取、换算、缓存保留、1024 点采集和丢包恢复；
也保留 XYZS 的删表、移址、比例和通信故障变体。IIO 覆盖两型号通道/打包及并发发布。
模拟测试使用合成数据；XYZ 实机记录见 `tests/hardware/TEST_REPORT_2026-10-19_XYZ_RAW_FIX.md`。
