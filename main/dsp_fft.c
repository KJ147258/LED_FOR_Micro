/*
 * 纯 C 信号处理库实现：radix-2 FFT + 汉宁窗 + 幅度谱 + 频域声源定位
 * 详见 dsp_fft.h 顶部的原理说明。
 */
#include <math.h>
#include "dsp_fft.h"

#define TWO_PI 6.28318530717958647692

/* 旋转因子表：W[k] = e^{-j·2π·k/N}，k = 0..N/2-1。
 * 蝴蝶运算里 W_size^j = W_N^{j·(N/size)}，只需这一张表按步长查即可。 */
static float s_tw_re[FFT_N / 2];
static float s_tw_im[FFT_N / 2];

/* 汉宁窗：w[n] = 0.5 - 0.5·cos(2π·n/N)，两端趋零，抑制截断引起的频谱泄漏 */
static float s_win[FFT_N];

void fft_init(void)
{
    for (int k = 0; k < FFT_N / 2; k++) {
        double a = -TWO_PI * k / FFT_N;
        s_tw_re[k] = (float)cos(a);
        s_tw_im[k] = (float)sin(a);
    }
    for (int n = 0; n < FFT_N; n++) {
        s_win[n] = (float)(0.5 - 0.5 * cos(TWO_PI * n / FFT_N));
    }
}

void fft_transform(float *re, float *im)
{
    const int n = FFT_N;

    /* ---- 第 1 步：二进制位反转重排 ----
     * 蝶形合并要求数据按“倒位序”排列（如 8 点的下标 0,4,2,6,1,5,3,7）。
     * 这里用经典的 in-place 位反转交换。 */
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; (j & bit) != 0; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {          /* 每对只交换一次 */
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }

    /* ---- 第 2 步：逐级蝴蝶运算 ----
     * 级长 size = 2,4,8,...,n。每一级把两个 size/2 点的“子变换”合并成
     * size 点：a' = a + W·b，b' = a - W·b。 */
    for (int size = 2; size <= n; size <<= 1) {
        int half = size >> 1;
        int step = n / size;              /* 旋转因子查表步长 */
        for (int i = 0; i < n; i += size) {
            for (int j = 0; j < half; j++) {
                int a = i + j;
                int b = a + half;
                float tr = s_tw_re[j * step];
                float ti = s_tw_im[j * step];
                float br = re[b] * tr - im[b] * ti;   /* 复数乘 W */
                float bi = re[b] * ti + im[b] * tr;
                re[b] = re[a] - br;                    /* b' = a - W·b */
                im[b] = im[a] - bi;
                re[a] += br;                           /* a' = a + W·b */
                im[a] += bi;
            }
        }
    }
}

void fft_analyze(const int16_t *in, int n, float *re, float *im, float *mag)
{
    /* ① 加汉宁窗并转 float（虚部清零） */
    for (int i = 0; i < n; i++) {
        re[i] = s_win[i] * (float)in[i];
        im[i] = 0.0f;
    }

    /* ② FFT */
    fft_transform(re, im);

    /* ③ 幅度谱：|X[k]| = sqrt(re² + im²)，只取 k = 0..n/2。
     *    负频率部分(n/2+1..n-1)对实信号是共轭对称的，无需重复显示。 */
    for (int k = 0; k <= n / 2; k++) {
        mag[k] = sqrtf(re[k] * re[k] + im[k] * im[k]);
    }
}

float fft_peak_freq(const float *mag, float fs)
{
    /* 从 k=1 开始（跳过直流分量），找幅度最大的 bin */
    int kp = 1;
    float best = mag[1];
    for (int k = 2; k < FFT_N / 2; k++) {
        if (mag[k] > best) {
            best = mag[k];
            kp = k;
        }
    }

    /* 抛物线插值：用峰值及左右邻点拟合抛物线，把峰定位到亚 bin 精度，
     * 得到 delta ∈ [-0.5, 0.5] 的偏移。 */
    float a = mag[kp - 1];
    float b = mag[kp];
    float c = mag[kp + 1];
    float denom = a - 2.0f * b + c;
    float delta = 0.0f;
    if (fabsf(denom) > 1e-6f) {
        delta = 0.5f * (a - c) / denom;
        if (delta > 0.5f) delta = 0.5f;
        if (delta < -0.5f) delta = -0.5f;
    }

    return (kp + delta) * fs / FFT_N;
}

float fft_delay_estimate(const float *mag_l, const float *re_l, const float *im_l,
                         const float *mag_r, const float *re_r, const float *im_r,
                         float fs, float mic_dist, float speed,
                         float *angle, float *quality)
{
    /* 只统计中低频 bin，见 FFT_LOC_MAX_HZ 的注释 */
    int kmax = (int)(FFT_LOC_MAX_HZ * FFT_N / fs);
    if (kmax < 2) kmax = 2;
    if (kmax > FFT_N / 2 - 1) kmax = FFT_N / 2 - 1;

    /* 以中低频的最大双路能量为参考，忽略能量过低的噪声 bin */
    double peak = 0.0;
    for (int k = 1; k <= kmax; k++) {
        double m = (double)mag_l[k] * (double)mag_r[k];
        if (m > peak) peak = m;
    }
    if (peak < 1e-9) {
        *quality = 0.0f;
        return 0.0f;
    }
    const double thresh = peak * 0.05;

    double w_used = 0.0, w_total = 0.0, dsum = 0.0;
    for (int k = 1; k <= kmax; k++) {
        double w = (double)mag_l[k] * (double)mag_r[k];
        w_total += w;
        if (w < thresh) {
            continue;
        }

        /* R·conj(L) 的相位 = angle(X_R) - angle(X_L) = 两路相位差 Δφ */
        double cr = (double)re_r[k] * re_l[k] + (double)im_r[k] * im_l[k];
        double ci = (double)im_r[k] * re_l[k] - (double)re_r[k] * im_l[k];
        double phi = atan2(ci, cr);   /* Δφ ∈ (-π, π] */

        /* Δφ = -2π·f_k·τ，f_k = k·fs/N  =>  τ = -Δφ·N / (2π·k)（单位：采样点） */
        double tau = -phi * FFT_N / (TWO_PI * k);

        w_used += w;
        dsum += w * tau;
    }

    if (w_used < 1e-9) {
        *quality = 0.0f;
        return 0.0f;
    }

    /* 加权平均时延：能量越强的频率成分越可信 */
    float tau_samp = (float)(dsum / w_used);

    /* 和时域互相关同样的几何换算：路程差 -> 方向角 */
    float path = speed * tau_samp / fs;
    float s = path / mic_dist;
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    *angle = asinf(s) * 180.0f / (float)M_PI;

    /* 可信度：参与定位的能量占中低频总能量的比例（0..1） */
    *quality = (w_total > 1e-9) ? (float)(w_used / w_total) : 0.0f;
    return tau_samp;
}