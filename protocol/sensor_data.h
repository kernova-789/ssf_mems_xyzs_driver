#pragma once

#include <linux/types.h>

/* 仅定义传感器结果数据，不依赖协议业务、请求或接收模块。 */
#define SSF_MEMS_CENTI_SCALE 100U
#define SSF_MEMS_DECI_SCALE 10U

/* 定长批次输出的一个同步 XYZ 样本，保留有符号 ADC 计数。 */
struct ssf_mems_raw_sample {
  s16 xyz[3]; /* 固件定义的 X、Y、Z，满量程 ±16 g。 */
  u32 packet_sequence; /* 传感器包号；每包包含 64 个样本，可用于识别丢包。 */
  u16 sample_index; /* 样本在包内的位置，范围 0..63。 */
};

/*
 * 特征值按内部统一的定点单位保存，协议层按型号表换算。_x100 表示原物理量
 * 放大 100 倍，_x10 表示放大 10 倍；无后缀字段保留寄存器自身单位。
 * 当前型号无需再次缩放；内部只使用整数，避免内核浮点运算。
 */
struct ssf_mems_axis_features {
  u16 high_freq_acc_rms_x100; /* 高频加速度有效值，g × 100。 */
  u16 low_freq_velocity_rms_x100; /* 低频速度有效值，mm/s × 100。 */
  u16 acc_peak_to_peak_x100; /* 加速度峰峰值，g × 100。 */
  u16 acc_peak_x100; /* 加速度峰值，g × 100。 */
  u16 acc_rms_x100; /* 加速度有效值，g × 100。 */
  u16 kurtosis_x100; /* 峭度，无量纲 × 100。 */
  u16 velocity_rms_x100; /* 速度有效值，mm/s × 100。 */
};

/* 声音特征，保留固件的 dB 定点表示。 */
struct ssf_mems_sound_features {
  u16 rms_db_x100; /* 声音有效值，dB × 100。 */
  u16 peak_db_x100; /* 声音峰值，dB × 100。 */
  u16 peak_to_peak_db_x100; /* 声音峰峰值，dB × 100。 */
};

/* 一次完整特征快照；各字段来自同一轮寄存器读取。 */
struct ssf_mems_sensor_data {
  struct ssf_mems_axis_features x; /* X 轴振动特征。 */
  struct ssf_mems_axis_features y; /* Y 轴振动特征。 */
  struct ssf_mems_axis_features z; /* Z 轴振动特征。 */

  s16 temperature_x100; /* 摄氏度，固件负温以二补码传输 */
  struct ssf_mems_sound_features sound; /* 声音特征。 */
  u16 zero_crossing_rate_percent; /* 百分数，寄存器 1 = 1% */
  u16 spectral_centroid_hz_x10; /* 频谱重心，Hz × 10。 */
  u16 spectral_flux_x100;         /* 无标准物理单位的频谱差平方和 */

  /* 位 0..2 为 XYZ 运行判定：去直流加速度峰值 >= 0.06 g 时置位。
   * MEMS2.0 modebusInBuff 输出 motor_run_flag，0 不代表采集被禁用。 */
  u8 startup_flags; /* XYZ 运行判定位，定义见上方说明。 */
};
