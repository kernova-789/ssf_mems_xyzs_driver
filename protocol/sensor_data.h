#pragma once

#include <linux/types.h>

/* 仅定义传感器结果数据，不依赖协议业务、请求或接收模块。 */
#define SSF_MEMS_CENTI_SCALE 100U
#define SSF_MEMS_DECI_SCALE 10U

/* 连续模式输出的一个同步 XYZ 样本，保留有符号 ADC 计数。 */
struct ssf_mems_raw_sample {
  s16 xyz[3]; /* 固件定义的 X、Y、Z，满量程 ±16 g。 */
  u32 packet_sequence; /* 传感器包号；每包包含 64 个样本，可用于识别丢包。 */
  u16 sample_index; /* 样本在包内的位置，范围 0..63。 */
};

/*
 * 特征值按固件实际写入寄存器的定点格式保存。_x100 表示原物理量
 * 放大 100 倍，_x10 表示放大 10 倍；无后缀字段保留寄存器自身单位。
 * 保留原始定点表示既可避免在内核中进行浮点运算，也能避免二次量化。
 */
struct ssf_mems_axis_features {
  u16 high_freq_acc_rms_x100;     /* g */
  u16 low_freq_velocity_rms_x100; /* mm/s */
  u16 acc_peak_to_peak_x100; /* g */
  u16 acc_peak_x100;         /* g */
  u16 acc_rms_x100;          /* g */
  u16 kurtosis_x100;
  u16 velocity_rms_x100; /* mm/s */
};

struct ssf_mems_sound_features {
  u16 rms_db_x100;          /* dB */
  u16 peak_db_x100;         /* dB */
  u16 peak_to_peak_db_x100; /* dB */
};

struct ssf_mems_sensor_data {
  struct ssf_mems_axis_features x;
  struct ssf_mems_axis_features y;
  struct ssf_mems_axis_features z;

  s16 temperature_x100; /* 摄氏度，固件负温以二补码传输 */
  struct ssf_mems_sound_features sound;
  u16 zero_crossing_rate_percent; /* 百分数，寄存器 1 = 1% */
  u16 spectral_centroid_hz_x10;   /* Hz */
  u16 spectral_flux_x100;         /* 无标准物理单位的频谱差平方和 */

  /* 位 0..2 为 XYZ 运行判定：去直流加速度峰值 >= 0.06 g 时置位。
   * MEMS2.0 modebusInBuff 输出 motor_run_flag，0 不代表采集被禁用。 */
  u8 startup_flags;
};
