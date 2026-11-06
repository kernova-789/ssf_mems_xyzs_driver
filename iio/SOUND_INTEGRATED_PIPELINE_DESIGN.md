# 温振声 IIO 驱动：声音裸数据与全模式时分复用调度重构方案

## 1. 现状痛点与重构动因

### 1.1 现状与痛点
在当前 `mems_driver` 实现中，采集模型存在明显的状态割裂：
* **特征与振动裸数据（一等公民）**：由 `sensor_acquisition_mode`（`feature` / `raw` / `alternating`）统一管理，内核后台线程 `kthread` 按照时间窗口平滑轮转。
* **声音裸数据（二等公民 / 外挂动作）**：只能通过特征设备的只写属性 `sensor_sound_capture = 1` 单次触发。
* **致命痛点（内核硬编码检查）**：驱动源码 `ssf_mems_acquisition.c` 第 609 行强制要求 `phase == SSF_MEMS_ACQ_POLL`。如果系统当前正在采振动裸数据，触发声音会直接报错返回 `-ENODATA`。
* **上位机代价**：导致上位机（如 EdgeSense 后端的 `sound_capture.cpp`）在采一次声音时，必须被迫执行“偷梁换柱”操作：
  `保存旧模式 -> 强制改成 feature 模式 -> 等待振动停止 -> 触发声音 -> 恢复旧模式`。如果程序在此期间异常退出，底层采集模式就会被篡改致死。

### 1.2 物理边界与工程现实
* **RS485 单通道半双工**：串口总线速率通常为 128000 bps 或 230400 bps。
  * 传一批 1024 组 XYZ 振动（约 6KB~8KB）耗时约 0.5 秒；
  * 传一批 4096 点 32 位声音（约 18KB~20KB）耗时约 1.5~2.0 秒。
* **固件行为差异**：振动支持连续轮询；声音在传感器端为单次定长触发（向寄存器 40063 写 1 启动，传完 64 包后寄存器自动归零）。
* **结论**：三者在物理总线上**绝对无法并发**，但完全可以通过**内核层确定的时分复用（Time-Division Multiplexing, TDM）**，将三者统一度量、统一调度。

---

## 2. 重构目标与核心设计原则

1. **统一工作模式**：将传感器工作模式扩展为 4 种独立模式：
   * `feature`（纯特征数据）
   * `raw`（纯振动裸数据）
   * `sound`（纯声音裸数据）
   * `alternating`（交替模式）
2. **总线时间片可自由配置**：提供三个独立的窗口时长属性：
   * 特征窗口时间（`sensor_feature_duration_ms`）
   * 振动窗口时间（`sensor_raw_duration_ms`）
   * 声音窗口时间（`sensor_sound_duration_ms`）
3. **交替路线灵活可剪裁**：在 `alternating` 模式下，允许自由开启/关闭任意子项（例如只交替“特征+振动”，或“特征+声音”，或“特征+振动+声音”）。
4. **严格契合驱动既有风格**：
   * **短临界区配置**：写 sysfs 只在短临界区修改参数并标记 `config_changed`，唤醒 `waitq`，不阻塞串口等待；
   * **基于 jiffies 的非阻塞时间片**：调度线程到期自动切相，绝不截断在途的定长完整批次；
   * **型号能力感知**：在不支持声音的传感器（如 SSF-MEMS-XYZ）上，声音相关属性自动隐藏（`is_visible` 返回 0），非法配置返回 `-EOPNOTSUPP`；
   * **Buffer 状态感知保护**：未开启对应 IIO Buffer 时跳过采集，避免内核溢出和总线空耗。

---

## 3. Sysfs ABI 接口规范

所有控制属性统一挂载在**特征 IIO 设备**（如 `ssf_mems_xyzs`）下：

| 属性名 | 权限 | 取值范围 / 说明 | 默认值 |
| :--- | :---: | :--- | :--- |
| `sensor_acquisition_mode` | 0644 | `feature` / `raw` / `sound` / `alternating` | `alternating` |
| `sensor_acquisition_mode_available` | 0444 | `"raw feature sound alternating"`（只读列表） | - |
| `sensor_feature_duration_ms` | 0644 | 100 ～ 60000 ms，特征轮询时间窗口 | 5000 |
| `sensor_raw_duration_ms` | 0644 | 100 ～ 60000 ms，振动定长批次时间窗口 | 5000 |
| `sensor_sound_duration_ms` | 0644 | 100 ～ 60000 ms，声音定长批次时间窗口 | 5000 |
| `sensor_alternating_feature_en` | 0644 | `0` 或 `1`，交替模式是否包含特征轮询 | 1 |
| `sensor_alternating_raw_en` | 0644 | `0` 或 `1`，交替模式是否包含振动采集 | 1 |
| `sensor_alternating_sound_en` | 0644 | `0` 或 `1`，交替模式是否包含声音采集 | 1（仅 XYZS） |
| `sensor_sound_capture` | 0200 | 写 `1` 触发一次单批声音（保留旁路手动点检通道） | - |

---

## 4. 内核调度状态机设计

### 4.1 环形轮转时分复用模型

```text
       ┌───────────────────────────────┐
       │     POLL 阶段 (特征轮询)      │
       └──────────────┬────────────────┘
                      │
            (若 raw_en=1) │ (若 raw_en=0 且 sound_en=1)
                      ▼
       ┌───────────────────────────────┐
       │     RAW 阶段 (振动批次)       │
       └──────────────┬────────────────┘
                      │
           (若 sound_en=1) │ (若 sound_en=0 且 feature_en=1)
                      ▼
       ┌───────────────────────────────┐
       │     SOUND 阶段 (声音批次)     │
       └──────────────┬────────────────┘
                      │
                      └────► 返回 POLL (若 feature_en=1)
```

### 4.2 阶段流转算法（Round-Robin Transition）
```c
/* 环形查找下一使能路线：POLL -> RAW -> SOUND -> POLL */
static enum ssf_mems_acquisition_phase ssf_mems_next_alternating_phase(
    struct ssf_mems_acquisition_state *acq, enum ssf_mems_acquisition_phase current) {
  switch (current) {
  case SSF_MEMS_ACQ_POLL:
    if (acq->alt_raw_en) return SSF_MEMS_ACQ_RAW;
    if (acq->alt_sound_en) return SSF_MEMS_ACQ_SOUND;
    return SSF_MEMS_ACQ_POLL;
  case SSF_MEMS_ACQ_RAW:
    if (acq->alt_sound_en) return SSF_MEMS_ACQ_SOUND;
    if (acq->alt_feature_en) return SSF_MEMS_ACQ_POLL;
    return SSF_MEMS_ACQ_RAW;
  case SSF_MEMS_ACQ_SOUND:
  default:
    if (acq->alt_feature_en) return SSF_MEMS_ACQ_POLL;
    if (acq->alt_raw_en) return SSF_MEMS_ACQ_RAW;
    return SSF_MEMS_ACQ_SOUND;
  }
}
```

### 4.3 声音阶段内部的调度逻辑
* **循环触发与收尾**：在 `SOUND` 阶段，驱动根据 `sound_duration_ms` 计算 `deadline = jiffies + msecs_to_jiffies(acq->sound_duration_ms)`。
* **单批耗时考量**：一批 4096 点声音传输约耗时 1.5 秒。在 `deadline` 到达前，驱动可连续触发并读取 4096 点批次；一旦到期，收完当前在途批次后立即退出，平滑切至下一阶段。
* **Buffer 静默检查**：如果用户态尚未打开 `/dev/iio:deviceZ`（声音字符设备），即 `!iio_buffer_enabled(sound_dev)`，驱动在本阶段**不向传感器发送 40063 触发命令**，直接休眠让出总线，防止无消费导致的内核 Buffer 溢出。

---

## 5. 驱动源码修改清单

### 5.1 `acquisition/ssf_mems_acquisition.h`
1. 扩展 `enum ssf_mems_acquisition_mode`：增加 `SSF_MEMS_ACQ_MODE_SOUND`。
2. 扩展 `enum ssf_mems_acquisition_phase`：增加 `SSF_MEMS_ACQ_SOUND`。
3. 扩展 `struct ssf_mems_acquisition_config` 与 `struct ssf_mems_acquisition_state`：
   * 增加 `sound_duration_ms`；
   * 增加交替使能标志：`alt_feature_en`、`alt_raw_en`、`alt_sound_en`；
   * 统一截止时刻变量：用 `phase_until` 替代原先只针对特征的 `feature_until`。
4. 导出控制函数声明：
   * `int ssf_mems_acquisition_set_sound_duration(struct ssf_mems_data *data, unsigned int ms);`
   * `int ssf_mems_acquisition_set_alternating_en(struct ssf_mems_data *data, enum route, bool en);`

### 5.2 `acquisition/ssf_mems_acquisition.c`
1. **初始化**：在 `ssf_mems_acquisition_init` 中设置默认值：
   `sound_duration_ms = 5000`，`alt_feature_en = true`，`alt_raw_en = true`，`alt_sound_en = true`。
2. **模式设置校验**：在 `ssf_mems_acquisition_set_mode` 中增加对 `SSF_MEMS_ACQ_MODE_SOUND` 的检查（校验传感器是否具有 `collect_sound` 能力，无则返回 `-EOPNOTSUPP`）。
3. **主调度逻辑 `collect`**：
   * 检查 `time_after_eq(jiffies, acq->phase_until)`，触发环形切相；
   * 增加 `case SSF_MEMS_ACQ_SOUND:` 分支，调用 `data->sensor_ops->collect_sound(data)`；
   * 执行前加入 `iio_buffer_enabled` 防御检查。
4. **解绑单次触发限制**：修改 `ssf_mems_acquisition_capture_sound`，移除原先 609 行的 `acq->phase != POLL` 硬编码报错，改为在互斥锁保护下等待当前批次收尾后插入执行。

### 5.3 `iio/ssf_mems_iio.c`
1. 更新属性模式静态列表：
   `static IIO_CONST_ATTR(sensor_acquisition_mode_available, "raw feature sound alternating");`
2. 增加 Sysfs 属性定义：
   * `sensor_sound_duration_ms`（读写）
   * `sensor_alternating_feature_en`（读写）
   * `sensor_alternating_raw_en`（读写）
   * `sensor_alternating_sound_en`（读写）
3. 挂入 `ssf_mems_iio_attributes` 数组。
4. 在 `ssf_mems_iio_attr_visible` 回调中对没有声音能力的型号过滤上述属性。

---

## 6. 对 EdgeSense 后端带来的重构收益

改造完成后，EdgeSense 后端的复杂度将大幅下降：

| 维度 | 改造前 (Current) | 改造后 (Proposed) |
| :--- | :--- | :--- |
| **声音采集触发** | 必须由上位机手动写 `sensor_sound_capture = 1` | 内核自动按时分复用流水线轮流采集 |
| **总线竞争规避** | 上位机强行将全局模式切为 `feature` 并等待振动停止 | 内核驱动线程原生无锁等待与平滑切相 |
| **数据读取方式** | 单次发命令后通过 poll 等待单次流 | 像读振动一样，常驻打开声音 Buffer 连续消费 |
| **系统稳定性** | 上位机崩溃会导致驱动永久卡在 `feature` 模式 | 上位机无任何配置副作用，内核自闭环调度 |
| **采集组合** | 无法配置“纯声音监测”或“特征+声音交替” | 4 种模式 + 3 项子使能自由组合，灵活配置 |

