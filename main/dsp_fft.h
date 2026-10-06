/*
 * 纯 C 信号处理库：radix-2 快速傅里叶变换 + 窗函数 + 幅度谱 + 频域声源定位
 *
 * 无任何第三方依赖，全部用标准 C 数学库手写，代码逐行注释，便于课程讲解：
 *   1) 离散傅里叶变换(DFT) 把时域 N 点 x[n] 变成频域 N 点 X[k]：
 *        X[k] = Σ x[n]·e^{-j·2π·k·n/N}
 *      直接算需要 O(N²) 次运算；FFT 用“分治”把 N 点拆成两半递推，
 *      运算量降到 O(N·log₂N)，这就是“快速”的含义。
 *   2) 蝴蝶运算(butterfly)是 FFT 的核心：把两点的 a、b 合并成
 *        a' = a + W·b，b' = a - W·b（W 是旋转因子 e^{-j·2π/N 的幂}）。
 *   3) 变换后的 X[k] 下标的含义：k = 0 是直流分量，k = N/2 是奈奎斯特
 *      (采样率的一半，即最高可测频率)；|X[k]| 是该频率成分的“大小”。
 *
 * 本工程用它演示两个“信号处理的作用”：
 *   a) 从幅度谱读出声音的主频率（音高）——时域波形看不出频率，FFT 后一眼可见；
 *   b) 用左右两个麦克风同一频率成分的相位差做声源方向估计（频域定位），
 *      和时域互相关(TDOA)互相印证。
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FFT 点数（必须为 2 的幂）。
 * 1024 点 @48kHz：频率分辨率 Δf = 48000/1024 ≈ 46.9Hz，正频率 bin 数 512 个。
 * 选 1024 是为控制静态 RAM 占用（ESP32 无 PSRAM 时 DRAM 较紧）；若板子带
 * PSRAM 且愿意增加内存，可改成 2048（分辨率翻倍到 23.4Hz）。 */
#ifndef FFT_N
#define FFT_N 1024
#endif

/* 频域定位只用中低频（相位差在该频段无 ±π 混叠、语音/音调能量也集中于此），
 * 上限取 1500Hz；再高的 bin 时延歧义大，不参与加权。 */
#define FFT_LOC_MAX_HZ   1500.0f

/**
 * @brief 初始化：预计算一张旋转因子表和汉宁窗（上电后调用一次）。
 *        窗函数用于抑制“频谱泄漏”：真实信号被截断成 N 点后，能量会
 *        泄漏到旁瓣；乘上两端趋零的窗可大幅压低旁瓣，让峰值更干净。
 */
void fft_init(void);

/**
 * @brief 原地 radix-2 FFT：把 re/im 中的复数序列变换到频域。
 *        先做二进制位反转重排，再逐级做蝴蝶运算。
 */
void fft_transform(float *re, float *im);

/**
 * @brief 一路麦克风信号的完整分析流程：
 *        ① 加汉宁窗并转为 float；② 执行 FFT；③ 求幅度谱 |X[k]|。
 * @param in  输入 int16 样本，长度 n（必须等于 FFT_N）
 * @param n   样本数
 * @param re  输出：频谱实数部，长度 n
 * @param im  输出：频谱虚数部，长度 n
 * @param mag 输出：幅度谱，长度 n/2+1（含下标 0..n/2）
 */
void fft_analyze(const int16_t *in, int n, float *re, float *im, float *mag);

/**
 * @brief 从幅度谱找主频率（忽略直流），用相邻三点抛物线插值把峰值
 *        细化到亚 bin 精度，返回主频(Hz)。
 */
float fft_peak_freq(const float *mag, float fs);

/**
 * @brief 频域声源定位（相位差法）。
 *
 * 原理：声音从一侧传来时，先后到达两个麦克风，两路信号的同一个频率
 *      成分之间存在固定相位差 Δφ[k] = angle(X_R[k]) - angle(X_L[k])。
 *      相位差和到达时间差 τ 满足 Δφ = -2π·f_k·τ，因此
 *          τ = -Δφ / (2π·f_k)
 *      得到 τ 后按照和时域互相关相同的几何关系换算方向角。
 *
 * @param mag_l/re_r... 左右两路的幅度谱与频谱实/虚部（来自 fft_analyze）
 * @param fs          采样率
 * @param mic_dist    两麦克风间距(m)
 * @param speed       声速(m/s)
 * @param angle       输出方向角(度，正=声源在左侧)
 * @param quality     输出可信度(0..1)：参与加权的信号能量占中低频总能量的比例
 * @return 估计时延(采样点，正=右声道滞后=声源在左)
 */
float fft_delay_estimate(const float *mag_l, const float *re_l, const float *im_l,
                         const float *mag_r, const float *re_r, const float *im_r,
                         float fs, float mic_dist, float speed,
                         float *angle, float *quality);

#ifdef __cplusplus
}
#endif