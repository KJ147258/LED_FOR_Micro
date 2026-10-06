/*
 * 双 INMP441 声源定位 —— TDOA（到达时间差）快速验证（改进版）
 *
 * 原理：两个麦克风共用 SCK/WS 同步采样（一个 L/R 接地占左声道，
 *       一个 L/R 接 3V3 占右声道，DOUT 并联到同一根线），
 *       对左右两路信号做互相关求时间差，再换算方向角。
 *
 * 相对 v1 的改进：
 *  1. 采样率 16k -> 48k：时延分辨率提升 3 倍（16k 时 1 个采样点 ≈ 12°，
 *     48k 时 ≈ 4°），方向敏感度大幅提高；
 *  2. 自适应噪声底：自动跟踪环境噪声，触发阈值自动适应，更灵敏且不误报；
 *  3. 时延中值滤波：连续 7 帧取中位数，角度稳定不跳变；
 *  4. 归一化互相关 + 去 DC 偏置：相关系数不随音量变化，弱信号也能得到
 *     稳定、有物理意义的可信度判据（而非随幅度漂移的"峰均比"）；
 *  5. 抛物线亚采样插值：时延精度突破整数采样点，角度分辨率进一步提升。
 *
 * 硬件接线（默认，可按实际修改下方宏）：
 *   ESP32             INMP441(1)     INMP441(2)
 *   GPIO14 (BCK)  ->  SCK           SCK      (两片并联)
 *   GPIO15 (WS)   ->  WS            WS       (两片并联)
 *   GPIO22 (DIN)  ->  DOUT          DOUT     (两片 DOUT 并联到同一根线)
 *   GND           ->  L/R                   (麦克风1：占左声道)
 *   3V3           ->                L/R      (麦克风2：占右声道)
 *   3V3           ->  VDD           VDD
 *   GND           ->  GND           GND
 *
 * 注意：
 *  - 两片 DOUT 必须并联共用一根线（靠 L/R 分时输出），才能同步采样；
 *  - MAX_LAG 至少应 >= MIC_DIST_M / 343 * SAMPLE_RATE，例如
 *    0.1m 间距 @48kHz -> 14 个采样点，代码里 32 留了裕量；
 *  - 两只麦克风只能判断方向（左/右及角度），测不出距离。
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_err.h"
#include "st7789.h"
#include "max98357.h"
#include "dsp_fft.h"

static const char *TAG = "inmp441_doa";

/* ================= 引脚配置（按你的实际接线修改） ================= */
#define I2S_NUM        I2S_NUM_0
#define I2S_BCK_GPIO   14   /* 位时钟，接两片 SCK */
#define I2S_WS_GPIO    15  /* 左右声道时钟，接两片 WS */
#define I2S_DIN_GPIO   22   /* 数据输入，接两片 DOUT(并联) */

/* ================= 采样与定位参数 ================= */
#define SAMPLE_RATE    48000      /* 采样率 Hz：越高时延分辨率越高 */
#define SAMPLES        4096       /* 每通道一次分析的采样点数（48k 下约 85ms） */
#define MAX_LAG        32         /* 时延搜索范围（采样点），0.1m@48k 理论最大 14 */
#define MIC_DIST_M     0.10f      /* 两麦克风中心间距（米）——务必按实际修改！ */
#define SPEED_SOUND    343.0f     /* 声速 m/s */
#define MIN_NOISE_FLOOR 20.0f     /* 噪声底下限 */
#define TRIG_RATIO     2.0f       /* 触发倍数：RMS > 噪声底×2 即判为有声（更灵敏） */
#define HIST_N         7          /* 时延中值滤波窗口（帧） */
#define CORR_MIN       0.35f      /* 归一化互相关系数门限，低于此值视为噪声/弱信号 */
#define PI_F           3.14159265f
#define LAG_BINS       (2 * MAX_LAG + 1)   /* lag 扫描的离散点数 */

/* ===== 采集→回放/DOA 分流（解决“声音一段一段”断续的根本原因） =====
 * 麦克风采集独立成高优先级任务，持续以实时速率读 I2S，并把数据通过两个
 * FreeRTOS 队列分发给回放任务和 DOA：回放任务按实时速率写功放，DOA 主循环
 * 攒够 4096 点再算互相关+显示。这样即使 DOA/屏幕线程占用几十毫秒，回放数据
 * 也不会断流，声音连续。 */
#define CAP_BLOCK     512            /* 每次读的帧数（约 10.7ms @48k） */
#define DOA_Q_DEPTH   16             /* DOA 队列深度（缓冲约 0.17s） */
#define PLAY_Q_DEPTH  8              /* 回放队列深度 */

typedef struct {                     /* 一块立体声数据（给 DOA） */
    int     n;                       /* 有效帧数 */
    int16_t l[CAP_BLOCK];
    int16_t r[CAP_BLOCK];
} mic_blk_t;

typedef struct {                     /* 一块单声道数据（给回放） */
    int     n;
    int16_t s[CAP_BLOCK];
} play_blk_t;

static QueueHandle_t s_doa_q  = NULL;
static QueueHandle_t s_play_q = NULL;
static mic_blk_t     s_doa_blk;     /* DOA 主循环收块用的固定缓冲（避免占主任务栈） */

static i2s_chan_handle_t s_rx_chan = NULL;
static int16_t s_l[SAMPLES];
static int16_t s_r[SAMPLES];
static float s_noise_floor = 100.0f;  /* 自适应噪声底（起始猜测值） */
static float s_delay_hist[HIST_N];    /* 时延中值滤波窗口（浮点，支持小数时延） */
static int s_hist_idx = 0;

/* ===== FFT 频域分析工作缓冲 =====
 * 静态分配（不进主任务栈）：左右两路各一组频谱实部/虚部，以及各自的幅度谱。
 * mag 数组长度 N/2+1（含奈奎斯特点）。FFT_N=1024 时约占 20KB DRAM。 */
static float s_fft_re_l[FFT_N];
static float s_fft_im_l[FFT_N];
static float s_fft_re_r[FFT_N];
static float s_fft_im_r[FFT_N];
static float s_fft_mag_l[FFT_N / 2 + 1];
static float s_fft_mag_r[FFT_N / 2 + 1];

/* 频域定位/频谱的结果状态（供屏幕显示） */
static bool  s_fft_has = false;       /* 是否已得到首帧有效的频域角度 */
static float s_fft_angle = 0.0f;      /* 频域相位差算出的方向角（度） */
static float s_fft_angle_ema = 0.0f;  /* 上述角度的 EMA 平滑值（防跳变） */
static float s_fft_delay = 0.0f;      /* 频域估计时延（采样点） */
static float s_fft_quality = 0.0f;    /* 频域定位可信度 0..1 */
static float s_fft_peak_hz = 0.0f;    /* 左声道幅度谱主频（Hz，音高） */

static void i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_GPIO,
            .ws   = I2S_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_DIN_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx_chan));
    ESP_LOGI(TAG, "I2S rx ready: BCK=%d WS=%d DIN=%d fs=%dHz",
             I2S_BCK_GPIO, I2S_WS_GPIO, I2S_DIN_GPIO, SAMPLE_RATE);
}

/* 归一化互相关（NCC）搜索右声道相对左声道的延迟。
 * 先对两路减各自的均值（去 DC 偏置），再对每个 lag 求零均值互相关系数，
 * 取相关系数最大的 lag，并用相邻三点抛物线插值得到亚采样时延。
 *
 * 相关系数已归一化到约 [-1,1] 且与信号强弱无关：弱信号只要信噪比尚可，
 * 相关系数仍会接近 1；纯噪声则接近 0。因此它可以稳定地作为可信度判据。
 *
 * 返回：延迟（采样点，可为小数）；*out_corr 为该延迟处的峰值相关系数。 */
static float find_delay(const int16_t *l, const int16_t *r, int n, int max_lag,
                        double *out_corr)
{
    double ml = 0.0, mr = 0.0;

    /* 1. 去 DC 偏置，并预计算两路全段能量。
     *    不再保存 zl/zr 零均值数组：直接在互相关内层按 (l[i]-ml) 边算边用，
     *    省掉 2×SAMPLES 个 float（约 32KB）静态 DRAM；代价仅内层多一次减法。 */
    for (int i = 0; i < n; i++) { ml += l[i]; mr += r[i]; }
    ml /= n; mr /= n;

    double el = 0.0, er = 0.0;
    for (int i = 0; i < n; i++) {
        double dl = (double)l[i] - ml;
        double dr = (double)r[i] - mr;
        el += dl * dl;
        er += dr * dr;
    }
    double norm = sqrt(el * er) + 1e-12;     /* 统一归一化分母 */

    /* 2. 计算各 lag 的相关系数（MAX_LAG=32 << n=4096，重叠段≈全段，
     *    用全段能量做统一归一化即可，误差远小于 1%） */
    double corr[LAG_BINS];
    int nlags = 2 * max_lag + 1;
    for (int lag = -max_lag; lag <= max_lag; lag++) {
        double cross = 0.0;
        if (lag >= 0) {
            for (int i = 0; i + lag < n; i++)
                cross += ((double)l[i] - ml) * ((double)r[i + lag] - mr);
        } else {
            int k = -lag;
            for (int i = 0; i + k < n; i++)
                cross += ((double)l[i + k] - ml) * ((double)r[i] - mr);
        }
        corr[lag + max_lag] = cross / norm;
    }

    /* 3. 找相关系数最大点 */
    int best_idx = 0;
    double best_corr = corr[0];
    for (int k = 1; k < nlags; k++) {
        if (corr[k] > best_corr) {
            best_corr = corr[k];
            best_idx = k;
        }
    }

    /* 4. 相邻三点抛物线插值，细化到亚采样精度 */
    double prev = (best_idx > 0) ? corr[best_idx - 1] : best_corr;
    double next = (best_idx < nlags - 1) ? corr[best_idx + 1] : best_corr;
    double denom = prev - 2.0 * best_corr + next;
    double frac = 0.0;
    if (fabs(denom) > 1e-12) {
        frac = 0.5 * (prev - next) / denom;
        if (frac > 1.0) frac = 1.0;
        if (frac < -1.0) frac = -1.0;
    }

    *out_corr = best_corr;
    return (float)((best_idx - max_lag) + frac);
}

/* 中值滤波：把新值放入环形窗口，返回窗口内中位数（去抖） */
static float median_filter(float new_val)
{
    s_delay_hist[s_hist_idx] = new_val;
    s_hist_idx = (s_hist_idx + 1) % HIST_N;

    float tmp[HIST_N];
    memcpy(tmp, s_delay_hist, sizeof(tmp));
    for (int i = 1; i < HIST_N; i++) {        /* 插入排序 */
        float v = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > v) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = v;
    }
    return tmp[HIST_N / 2];
}

/* ===================== ST7789 显示布局 =====================
 *
 * 上下对比布局，直接展示“信号处理运算前 -> 运算后”的变化：
 *   0              顶部方向指示区：方向指针条 + 时域(DIR-T)/频域(DIR-F)对照文字
 *   TIME_LABEL_TOP 波形标签行（左半屏 L / 右半屏 R 各自显示 RMS）
 *   TIME_WAVE_TOP   波形区：左右声道时域波形左右并排（左红 / 右蓝）
 *   TIME_BOT
 *   SPEC_LABEL_TOP 频域标签行 (FFT mag  peak=...Hz)
 *   SPEC_WAVE_TOP   下半屏：FFT 幅度谱（运算后，青色柱状图，占全屏宽）
 *   SPEC_BOT = 320
 *
 * 波形区由同一段麦克风信号产生：左右两路是时间波形，下面是左声道频谱，
 * 一眼就能看出“时域杂乱无章的波纹，在频域变成了清晰的主频尖峰”，
 * 并排的两路波形则直观展示两麦之间的到达时间差。
 * 区域坐标全部由宏推导、无缝衔接，无重叠也不留空隙。
 */
#define HUD_H        38                             /* 顶部方向指示区高度 */
#define LABEL_H      12                             /* 每区标签行高度 */
#define WAVE_H       129                            /* 波形/频谱区高度(同高) */

#define TIME_LABEL_TOP  HUD_H                        /* 38 波形标签上边界 */
#define TIME_WAVE_TOP   (TIME_LABEL_TOP + LABEL_H)   /* 50 波形区上边界 */
#define TIME_BOT        (TIME_WAVE_TOP + WAVE_H)     /* 179 波形区下边界 */
#define SPEC_LABEL_TOP  TIME_BOT                     /* 179 频谱标签上边界 */
#define SPEC_WAVE_TOP   (SPEC_LABEL_TOP + LABEL_H)   /* 191 频谱柱状图上边界 */
#define SPEC_BOT        (SPEC_WAVE_TOP + WAVE_H)     /* 320 = ST7789_V_RES */

/* 波形区左右并排：左声道占左半屏、右声道占右半屏，同一水平线对齐同一时刻，
 * 便于直观对比两路信号的到达时间差。 */
#define WAVE_PANEL_W    120                           /* 每路波形面板宽度 */

/* 滚动波形：scroll[0]=最新(画在最左)，scroll[SCROLL_W-1]=最旧(画在最右)。
 * 每帧把旧数据整体右移 SCROLL_COLS_PER_FRAME 列、新数据从左进入，形成
 * “随时间向右移动、从右侧移出屏幕”的连续滚动。列数越小滚得越慢。 */
#define SCROLL_W              WAVE_PANEL_W            /* 滚动缓冲列数 = 每路面板宽 */
#define SCROLL_COLS_PER_FRAME 8                       /* 每帧推入的新列数 */

/* 悬空检测：拆掉一个麦克风后，它的 DOUT 悬空，对应声道会读到接近满幅的
 * 白噪声(RMS 很高)。用 RMS 阈值 + 连续帧数判定该路离线，避免满屏噪声污染
 * 画面，也让另一路能不受干扰地继续显示。 */
#define OFFLINE_RMS     14000.0f   /* 某路 RMS 持续超过此值即判为悬空 */
#define OFFLINE_FRAMES  8          /* 连续帧数阈值，抗拍手等瞬时大信号误判 */

/* 一路声道的完整显示状态：坐标、颜色、标签、滚动数据、幅度刻度全部独立，
 * 左右两路各有一个实例，绘制时互不共享任何可变状态。 */
typedef struct {
    int         x0;         /* 波形区起始列（左路 0 / 右路 WAVE_PANEL_W） */
    int         y_top;      /* 波形区上边界 */
    int         y_bot;      /* 波形区下边界 */
    int         label_y;    /* 标签行上边界 */
    uint16_t    color;      /* 波形/标签颜色 */
    const char *name;       /* 标签名 "L" / "R" */
    int16_t     scroll[SCROLL_W];  /* 滚动缓冲 */
    int32_t     peak;       /* EMA 平滑后的幅度刻度 */
    float       rms;        /* 本路上一帧 RMS */
    int         offline;    /* 1=该路被判定为悬空/离线，波形清空 */
    int         noise_cnt;  /* 连续超阈值帧计数(进入/退出离线用) */
} wave_view_t;

/* 在帧缓冲里画 Bresenham 线段 */
static void fb_line(uint16_t *fb, int fw, int y_top, int y_bot,
                    int x0, int y0, int x1, int y1, uint16_t c)
{
    int dx = x1 - x0, dy = y1 - y0;
    int sx = (dx > 0) ? 1 : -1;
    int sy = (dy > 0) ? 1 : -1;
    dx = (dx > 0) ? dx : -dx;
    dy = (dy > 0) ? dy : -dy;
    int err = dx - dy;
    int x = x0, y = y0;
    for (;;) {
        if (x >= 0 && x < fw && y >= y_top && y < y_bot) {
            fb[(y - y_top) * fw + x] = c;
        }
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 <  dx) { err += dx; y += sy; }
    }
}

/* 工作帧缓冲：仅供单次绘制临时使用。draw_wave / draw_spectrum 每次先
 * 整块清零再画，因此时域波形与频谱先后绘制互不影响，也绝不残留上一帧内容。
 *
 * 绘制改为逐“条带”(strip) 进行：每次只渲染 FB_STRIP_COLS 列、整条写出后再
 * 处理下一列条。这样缓冲只需覆盖 64×WAVE_H 个像素（约 17KB），而不是整屏
 * 240 列（约 64KB）——这是本项目静态 DRAM 最大的一项，条带化后即可在无 PSRAM
 * 的 ESP32 上链接（原实现会导致 .dram0.bss 溢出）。 */
#define FB_STRIP_COLS 64
static uint16_t s_fb[FB_STRIP_COLS * WAVE_H];

/* 把本帧数据分段平均成 SCROLL_COLS_PER_FRAME 个新列，旧数据整体右移 */
static void scroll_push(int16_t *hist, const int16_t *buf, int n)
{
    memmove(&hist[SCROLL_COLS_PER_FRAME], hist,
            (SCROLL_W - SCROLL_COLS_PER_FRAME) * sizeof(int16_t));
    for (int k = 0; k < SCROLL_COLS_PER_FRAME; k++) {
        int i0 = k * n / SCROLL_COLS_PER_FRAME;
        int i1 = (k + 1) * n / SCROLL_COLS_PER_FRAME;
        if (i1 <= i0) i1 = i0 + 1;
        int64_t acc = 0;
        for (int i = i0; i < i1; i++) acc += buf[i];
        hist[k] = (int16_t)(acc / (i1 - i0));
    }
}

/* 悬空/离线状态更新：RMS 连续 OFFLINE_FRAMES 帧超过阈值判为离线(悬空)，
 * 连续多帧回落到阈值以下才恢复。带迟滞，避免拍手之类瞬时大信号误判。 */
static void update_online_state(wave_view_t *v, float rms_ch)
{
    if (v->offline) {
        if (rms_ch < OFFLINE_RMS) {
            if (++v->noise_cnt >= OFFLINE_FRAMES) {
                v->offline   = 0;
                v->noise_cnt = 0;
                v->peak      = 600;
            }
        } else {
            v->noise_cnt = 0;
        }
    } else {
        if (rms_ch > OFFLINE_RMS) {
            if (++v->noise_cnt >= OFFLINE_FRAMES) {
                v->offline   = 1;
                v->noise_cnt = 0;
            }
        } else {
            v->noise_cnt = 0;
        }
    }
}

/* 把一路声道波形画到它的波形区 [y_top, y_bot)：每列一个点、相邻列连成
 * 折线，改写为逐条带 draw_bitmap，覆盖本路面板宽度。每次进入都先整块清零再画，
 * 配合独立清零的工作缓冲 s_fb，两路先后绘制互不影响。 */
static int wave_y(const wave_view_t *v, int y_mid, int half, int peak, int32_t val)
{
    int y = y_mid - (int)((val * half) / peak);
    if (y < v->y_top) y = v->y_top;
    if (y >= v->y_bot) y = v->y_bot - 1;
    return y;
}

static void draw_wave(const wave_view_t *v)
{
    const int width  = SCROLL_W;
    const int height = v->y_bot - v->y_top;
    const int y_mid  = v->y_top + height / 2;
    const int half   = height / 2;
    const int peak   = (v->peak > 0) ? v->peak : 1;   /* 防御：避免除零 */
    const uint16_t zero_color = ST7789_RGB(48, 48, 48);

    /* 逐条带渲染，s_fb 只需覆盖 FB_STRIP_COLS 列（其内 x 为条带局部列号）。 */
    for (int x0 = 0; x0 < width; x0 += FB_STRIP_COLS) {
        int cols = FB_STRIP_COLS;
        if (x0 + cols > width) cols = width - x0;

        memset(s_fb, 0, (size_t)cols * (size_t)height * sizeof(uint16_t));
        for (int x = 0; x < cols; x++) {
            s_fb[(y_mid - v->y_top) * cols + x] = zero_color;
        }

        /* 上一列的折线起点；跨条带时上一列在上一列条的右边界，局部 x 记为 -1，
         * 由 fb_line 的 x 限幅丢弃越界点，只保留落入当前条带的部分。 */
        int px = (x0 > 0) ? -1 : 0;
        int py = (x0 > 0) ? wave_y(v, y_mid, half, peak, v->scroll[x0 - 1]) : y_mid;

        for (int x = 0; x < cols; x++) {
            int y = wave_y(v, y_mid, half, peak, v->scroll[x0 + x]);
            if (x0 + x > 0) {
                fb_line(s_fb, cols, v->y_top, v->y_bot, px, py, x, y, v->color);
            }
            px = x;
            py = y;
        }

        st7789_draw_bitmap(v->x0 + x0, v->y_top, v->x0 + x0 + cols, v->y_bot, s_fb);
    }
}

/* 把角度(度，范围 ±90)线性映射到指针条的水平像素坐标 */
static int angle_to_x(float angle)
{
    int x = ST7789_H_RES / 2 + (int)(angle * (ST7789_H_RES / 2.0f / 90.0f));
    if (x < 3) x = 3;
    if (x > ST7789_H_RES - 3) x = ST7789_H_RES - 3;
    return x;
}

/* 画一个顶点朝下(在 y_bot)、底边在上(y_top)的实心等腰三角形指示箭头 */
static void hud_triangle(int cx, int y_top, int y_bot, uint16_t color)
{
    int h = y_bot - y_top;
    for (int y = y_top; y <= y_bot; y++) {
        int halfw = h - (y - y_top);   /* 顶部最宽 h，底部收窄到 0 */
        st7789_fill_rect(cx - halfw, y, cx + halfw + 1, y + 1, color);
    }
}

/* 空心三角（只描边），与时域实心三角叠加时仍能分辨 */
static void hud_triangle_outline(int cx, int y_top, int y_bot, uint16_t color)
{
    int h = y_bot - y_top;
    if (cx - h < 0) cx = h;                       /* 防御：保证描边不出屏 */
    if (cx + h >= ST7789_H_RES) cx = ST7789_H_RES - 1 - h;
    st7789_draw_line(cx - h, y_top, cx + h, y_top, color);
    st7789_draw_line(cx - h, y_top, cx, y_bot, color);
    st7789_draw_line(cx + h, y_top, cx, y_bot, color);
}

/* 顶部方向指示区：上半是指针条(-90°..0°..+90°)，实心红三角=时域定位角、
 * 空心青三角=频域定位角，两者重叠即两种算法结果一致；下半是文字对照。 */
static void draw_hud(bool have_t, float t_angle, double t_rho,
                     bool have_f, float f_angle, float f_quality, float peak_hz)
{
    char line[52];

    st7789_fill_rect(0, 0, ST7789_H_RES, HUD_H, ST7789_COLOR_BLACK);

    /* ---- 指针条：水平量程 -90°(左) .. 0°(正前) .. +90°(右) ---- */
    st7789_draw_line(0, 19, ST7789_H_RES - 1, 19, ST7789_RGB(80, 80, 80));
    st7789_draw_line(ST7789_H_RES / 2, 15, ST7789_H_RES / 2, 19, ST7789_COLOR_WHITE);
    st7789_draw_line(0, 17, 0, 19, ST7789_RGB(80, 80, 80));
    st7789_draw_line(ST7789_H_RES - 1, 17, ST7789_H_RES - 1, 19, ST7789_RGB(80, 80, 80));
    st7789_draw_line(60, 17, 60, 19, ST7789_RGB(80, 80, 80));
    st7789_draw_line(180, 17, 180, 19, ST7789_RGB(80, 80, 80));

    st7789_draw_text(2, 1, "L", ST7789_COLOR_RED, 1);
    st7789_draw_text(ST7789_H_RES / 2 - 5, 1, "0", ST7789_COLOR_WHITE, 1);
    st7789_draw_text(ST7789_H_RES - 8, 1, "R", ST7789_COLOR_CYAN, 1);

    if (have_t) {
        hud_triangle(angle_to_x(t_angle), 9, 19, ST7789_COLOR_RED);
    }
    if (have_f) {
        hud_triangle_outline(angle_to_x(f_angle), 10, 18, ST7789_COLOR_CYAN);
    }
    if (!have_t && !have_f) {
        hud_triangle(ST7789_H_RES / 2, 14, 19, ST7789_RGB(90, 90, 90));
    }

    /* ---- 文字对照 ---- */
    if (have_t || have_f) {
        snprintf(line, sizeof(line), "pk %5.0fHz T%+4.0f F%+4.0f",
                 peak_hz, have_t ? t_angle : 0.0f, have_f ? f_angle : 0.0f);
        st7789_draw_text(2, 23, line, ST7789_COLOR_GREEN, 1);
        snprintf(line, sizeof(line), "xcorr r=%.2f | phase q=%.2f",
                 have_t ? t_rho : 0.0, have_f ? f_quality : 0.0f);
        st7789_draw_text(2, 31, line, ST7789_COLOR_GREEN, 1);
    } else {
        st7789_draw_text(2, 23, "silent / noise ...", ST7789_COLOR_YELLOW, 1);
        st7789_draw_text(2, 31, "time vs freq domain", ST7789_COLOR_YELLOW, 1);
    }
}

/* 波形标签行（各声道在各自半屏显示 RMS），只清它自己的那 LABEL_H 像素行 */
static void draw_label(const wave_view_t *v)
{
    char line[24];

    st7789_fill_rect(v->x0, v->label_y, v->x0 + SCROLL_W, v->label_y + LABEL_H,
                     ST7789_COLOR_BLACK);

    if (v->offline) {
        snprintf(line, sizeof(line), "%s OFFLINE", v->name);
        st7789_draw_text(v->x0 + 2, v->label_y + 2, line, ST7789_COLOR_YELLOW, 1);
    } else {
        snprintf(line, sizeof(line), "%s RMS=%5.0f", v->name, v->rms);
        st7789_draw_text(v->x0 + 2, v->label_y + 2, line, v->color, 1);
    }
}

/* 频谱标签行：标注频率轴范围与主频——展示 FFT 的“作用”：读出音高 */
static void draw_spec_label(void)
{
    char line[48];

    st7789_fill_rect(0, SPEC_LABEL_TOP, ST7789_H_RES, SPEC_LABEL_TOP + LABEL_H,
                     ST7789_COLOR_BLACK);
    snprintf(line, sizeof(line), "FFT mag 0-24kHz  peak %4.0f Hz", s_fft_peak_hz);
    st7789_draw_text(2, SPEC_LABEL_TOP + 2, line, ST7789_COLOR_CYAN, 1);
}

/* 把左声道幅度谱画成频谱柱状图（运算后）：横轴频率(0..24kHz)、纵轴幅度。
 * FFT_N/2 个正频率 bin 按列聚合(取最大，峰值保持)，柱高按本帧峰值归一化；
 * 主频所在列高亮为黄色，其余为青色。 */
static void draw_spectrum(const float *mag)
{
    const int width  = ST7789_H_RES;               /* 频谱占全屏宽(左右声道共用) */
    const int height = SPEC_BOT - SPEC_WAVE_TOP;   /* 129 */
    const int base   = height - 1;
    const int nbins  = FFT_N / 2;                  /* 正频率 bin 数(不含 Nyquist) */

    /* 本帧峰值幅度用于柱高归一化（忽略 DC） */
    float peak = 0.0f;
    for (int k = 1; k < nbins; k++) {
        if (mag[k] > peak) peak = mag[k];
    }
    if (peak < 1e-6f) peak = 1.0f;

    /* 主频换算成列号，用于高亮；频率轴 0..fs/2 线性映射到 0..width */
    int peak_col = (int)(s_fft_peak_hz / (0.5f * SAMPLE_RATE) * width);
    if (peak_col < 0) peak_col = 0;
    if (peak_col >= width) peak_col = width - 1;

    /* 逐条带渲染，s_fb 只需覆盖 FB_STRIP_COLS 列（其内 col 为条带局部列号）。 */
    for (int x0 = 0; x0 < width; x0 += FB_STRIP_COLS) {
        int cols = FB_STRIP_COLS;
        if (x0 + cols > width) cols = width - x0;

        memset(s_fb, 0, (size_t)cols * (size_t)height * sizeof(uint16_t));

        for (int col = 0; col < cols; col++) {
            int gcol = x0 + col;
            /* 该列覆盖的 bin 范围，取频段内最大幅度，保留窄峰 */
            int k0 = gcol * nbins / width;
            int k1 = (gcol + 1) * nbins / width;
            if (k1 <= k0) k1 = k0 + 1;
            float m = 0.0f;
            for (int k = k0; k < k1; k++) {
                if (mag[k] > m) m = mag[k];
            }

            int h = (int)(m / peak * base);
            if (h < 1) h = 1;       /* 至少 1 像素，让噪声底可见 */
            if (h > base) h = base;

            uint16_t c = (gcol == peak_col) ? ST7789_COLOR_YELLOW : ST7789_COLOR_CYAN;
            for (int y = base; y > base - h; y--) {
                s_fb[y * cols + col] = c;
            }
        }

        /* 底部基线 */
        for (int x = 0; x < cols; x++) {
            s_fb[base * cols + x] = ST7789_RGB(64, 64, 64);
        }

        st7789_draw_bitmap(x0, SPEC_WAVE_TOP, x0 + cols, SPEC_BOT, s_fb);
    }
}

/* 左声道：时域波形视图（波形区左半屏，红色）—— “运算前”的输入信号 */
static wave_view_t s_view_l = {
    .x0      = 0,
    .y_top   = TIME_WAVE_TOP,
    .y_bot   = TIME_BOT,
    .label_y = TIME_LABEL_TOP,
    .color   = ST7789_COLOR_RED,
    .name    = "L",
    .peak    = 600,
};

/* 右声道：时域波形视图（波形区右半屏，蓝色），与左声道水平对齐、便于对比时延。
 * 其 RMS/离线状态同时用于频域定位与噪声判断。 */
static wave_view_t s_view_r = {
    .x0      = WAVE_PANEL_W,
    .y_top   = TIME_WAVE_TOP,
    .y_bot   = TIME_BOT,
    .label_y = TIME_LABEL_TOP,
    .color   = ST7789_COLOR_BLUE,
    .name    = "R",
    .peak    = 600,
};

/* 右移 PLAY_GAIN_SHIFT 位降低“喇叭→麦克风”声学反馈引起的啸叫。
 * 除以 4 ≈ -12dB；移位越大声音越小、越不易啸叫。 */
#define PLAY_GAIN_SHIFT  2

/* 回放任务：从回放队列持续取右声道块，衰减后写功放。i2s 写以 48kHz 被 DMA
 * 节流，因此天然按实时速率输出；队列空时阻塞，有数据就连续放。 */
static void amp_playback_task(void *arg)
{
    (void)arg;
    play_blk_t blk;
    for (;;) {
        if (xQueueReceive(s_play_q, &blk, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        for (int i = 0; i < blk.n; i++) {
            blk.s[i] >>= PLAY_GAIN_SHIFT;
        }
        max98357_write(blk.s, (size_t)blk.n * sizeof(int16_t), UINT32_MAX);
    }
}

/* 采集任务（高优先级）：只做 I2S 读 + 解包 + 入队，不被 DOA/屏幕阻塞，
 * 因此能以 48kHz 实时速率持续产数据，回放不会断流。 */
static void mic_capture_task(void *arg)
{
    (void)arg;
    int32_t *raw = malloc(CAP_BLOCK * 2 * sizeof(int32_t));
    if (raw == NULL) {
        ESP_LOGE(TAG, "capture: no mem");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(s_rx_chan, raw,
                                         CAP_BLOCK * 2 * sizeof(int32_t),
                                         &bytes_read, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "i2s read failed: %s", esp_err_to_name(ret));
            continue;
        }
        int nb = (int)(bytes_read / sizeof(int32_t) / 2);
        if (nb < 1) {
            continue;
        }

        mic_blk_t  mb;
        play_blk_t pb;
        mb.n = nb;
        pb.n = nb;
        for (int i = 0; i < nb; i++) {
            mb.l[i] = (int16_t)(raw[i * 2] >> 16);
            mb.r[i] = (int16_t)(raw[i * 2 + 1] >> 16);
            pb.s[i] = mb.r[i];
        }

        /* 非阻塞入队：消费者一时来不及就丢弃当前块，保证采集永不被阻塞 */
        if (s_doa_q != NULL) {
            xQueueSend(s_doa_q, &mb, 0);
        }
        if (s_play_q != NULL) {
            xQueueSend(s_play_q, &pb, 0);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "init two INMP441 + ST7789 ...");
    i2s_init();
    fft_init();   /* 预计算 FFT 旋转因子与汉宁窗 */

    /* 功放初始化失败不影响麦克风定位与屏幕显示，只是没有声音 */
    esp_err_t amp_ret = max98357_init();
    ESP_LOGI(TAG, "max98357 init: %s", esp_err_to_name(amp_ret));

    /* 创建“采集→回放 / DOA”两个数据队列 */
    s_doa_q  = xQueueCreate(DOA_Q_DEPTH, sizeof(mic_blk_t));
    s_play_q = xQueueCreate(PLAY_Q_DEPTH, sizeof(play_blk_t));
    if (s_doa_q == NULL || s_play_q == NULL) {
        ESP_LOGE(TAG, "create queue failed");
        return;
    }

    /* 回放任务：消费右声道队列，实时写功放 */
    if (amp_ret == ESP_OK) {
        xTaskCreate(amp_playback_task, "amp_playback", 4096, NULL, 9, NULL);
    }

    /* 高优先级采集任务：持续实时读 I2S，喂给回放与 DOA 两个队列 */
    xTaskCreate(mic_capture_task, "mic_capture", 6144, NULL, 10, NULL);

    /* 屏幕初始化失败不影响麦克风定位，只是不显示波形 */
    esp_err_t lcd_ret = st7789_init();
    ESP_LOGI(TAG, "st7789 init: %s", esp_err_to_name(lcd_ret));

    ESP_LOGI(TAG, "sampling... 上屏=时域波形(红) 下屏=FFT频谱(青) HUD=时/频域定位对照");
    int silent_cnt = 0;
    const TickType_t hud_period = pdMS_TO_TICKS(200);   /* 顶部信息条节流，避免文字闪烁 */
    TickType_t last_hud_tick = 0;
    while (1) {
        /* 从采集队列收满约 SAMPLES 帧再做一次分析；暂时收不满也用已有样本 */
        int n = 0;
        while (n < SAMPLES) {
            if (xQueueReceive(s_doa_q, &s_doa_blk, pdMS_TO_TICKS(200)) != pdTRUE) {
                break;
            }
            int room = SAMPLES - n;
            int take = (s_doa_blk.n < room) ? s_doa_blk.n : room;
            memcpy(&s_l[n], s_doa_blk.l, (size_t)take * sizeof(int16_t));
            memcpy(&s_r[n], s_doa_blk.r, (size_t)take * sizeof(int16_t));
            n += take;
        }
        if (n < 64) {
            continue;
        }

        double sum_l = 0.0, sum_r = 0.0;
        for (int i = 0; i < n; i++) {
            sum_l += (double)s_l[i] * s_l[i];
            sum_r += (double)s_r[i] * s_r[i];
        }
        float rms_l = sqrtf((float)(sum_l / n));
        float rms_r = sqrtf((float)(sum_r / n));
        float rms   = sqrtf((float)((sum_l + sum_r) / (2.0 * n)));

        /* 更新滚动幅度刻度（EMA 抗抖，避免波形上下跳动）。
         * 左右两路各自独立计算：拔掉任意一个麦克风后，悬空那一路会读到
         * 满幅乱噪声，绝不能再让它把另一路的刻度一并抬到满量程。 */
        {
            int frame_peak_l = 0, frame_peak_r = 0;
            for (int i = 0; i < n; i++) {
                int v = s_l[i]; if (v < 0) v = -v;
                if (v > frame_peak_l) frame_peak_l = v;
                v = s_r[i]; if (v < 0) v = -v;
                if (v > frame_peak_r) frame_peak_r = v;
            }
            if (frame_peak_l < 600) frame_peak_l = 600;
            if (frame_peak_r < 600) frame_peak_r = 600;
            s_view_l.peak = (s_view_l.peak * 7 + frame_peak_l) / 8;
            s_view_r.peak = (s_view_r.peak * 7 + frame_peak_r) / 8;
            s_view_l.rms  = rms_l;
            s_view_r.rms  = rms_r;

            /* 悬空检测：拆掉一个麦克风后对应声道会读到满幅噪声，判为离线 */
            update_online_state(&s_view_l, rms_l);
            update_online_state(&s_view_r, rms_r);
        }

        /* 静音跟踪（不再 continue，静音也照常刷新屏幕） */
        bool silent = false;
        if (rms < s_noise_floor * 1.5f) {
            s_noise_floor = s_noise_floor * 0.95f + rms * 0.05f;
            if (s_noise_floor < MIN_NOISE_FLOOR) {
                s_noise_floor = MIN_NOISE_FLOOR;
            }
            silent = true;
            if (++silent_cnt >= 12) {
                printf("RMS=%.0f (L=%.0f R=%.0f) [静音] noise_floor=%.0f\n",
                       rms, rms_l, rms_r, s_noise_floor);
                silent_cnt = 0;
            }
        } else {
            silent_cnt = 0;
        }

        /* ================= 定位：时域互相关 + 频域相位差 =================
         * 两种方法用的是同一段麦克风信号，结果应当一致——这正是傅里叶变换的
         * “作用”之一：很多时域算不直观的量（相位差），换到频域一目了然。 */
        bool triggered = (!silent && rms >= s_noise_floor * TRIG_RATIO);
        bool both_ok   = !s_view_l.offline && !s_view_r.offline;   /* 两麦都在线 */

        /* ---- 频域分析：FFT + 主频 + 相位差定位 ----
         * 只要收满 FFT_N 点就做（静音时也做，频谱/主频照常刷新）。 */
        if (n >= FFT_N) {
            fft_analyze(s_l, FFT_N, s_fft_re_l, s_fft_im_l, s_fft_mag_l);
            s_fft_peak_hz = fft_peak_freq(s_fft_mag_l, (float)SAMPLE_RATE);
            if (both_ok) {
                fft_analyze(s_r, FFT_N, s_fft_re_r, s_fft_im_r, s_fft_mag_r);
                s_fft_delay = fft_delay_estimate(s_fft_mag_l, s_fft_re_l, s_fft_im_l,
                                                 s_fft_mag_r, s_fft_re_r, s_fft_im_r,
                                                 (float)SAMPLE_RATE, MIC_DIST_M, SPEED_SOUND,
                                                 &s_fft_angle, &s_fft_quality);
            }
        }

        /* ---- 时域互相关定位（原逻辑保留） ---- */
        bool have_t = false;
        float t_angle = 0.0f;
        float t_delay = 0.0f;
        double t_rho = 0.0;

        if (triggered && both_ok) {
            float delay = find_delay(s_l, s_r, n, MAX_LAG, &t_rho);
            if (t_rho >= CORR_MIN) {
                t_delay = median_filter(delay);             /* 去抖 */
                float dt = t_delay / SAMPLE_RATE;
                float path_diff = SPEED_SOUND * dt;         /* 路程差 */
                float sin_th = path_diff / MIC_DIST_M;      /* 方向角 sin（相对连线法线） */
                if (sin_th > 1.0f) sin_th = 1.0f;
                if (sin_th < -1.0f) sin_th = -1.0f;
                t_angle = asinf(sin_th) * 180.0f / PI_F;    /* 正角 = 声源在左侧 */
                have_t = true;
            }
        }

        /* ---- 频域定位有效性判定 + EMA 平滑（防角度跳变） ---- */
        bool have_f = false;
        if (triggered && both_ok && s_fft_quality > 0.1f) {
            if (!s_fft_has) {           /* 首帧直接把当前值作为初值 */
                s_fft_angle_ema = s_fft_angle;
                s_fft_has = true;
            } else {
                s_fft_angle_ema = s_fft_angle_ema * 0.6f + s_fft_angle * 0.4f;
            }
            have_f = true;
        }

        /* 串口打印两种定位结果对照，便于观察两者一致性 */
        if (triggered) {
            if (have_t && have_f) {
                printf("RMS=%.0f(L=%.0f R=%.0f) TIME delay=%+6.2f ang=%+6.1f(r=%.2f) | "
                       "FREQ delay=%+6.2f ang=%+6.1f(q=%.2f) peak=%.0fHz\n",
                       rms, rms_l, rms_r, t_delay, t_angle, t_rho,
                       s_fft_delay, s_fft_angle_ema, s_fft_quality, s_fft_peak_hz);
            } else if (have_t) {
                printf("RMS=%.0f 时域 ang=%+6.1f(r=%.2f) | 频域不可靠\n",
                       rms, t_angle, t_rho);
            } else if (have_f) {
                printf("RMS=%.0f 频域 ang=%+6.1f(q=%.2f) peak=%.0fHz | 时域不可靠\n",
                       rms, s_fft_angle_ema, s_fft_quality, s_fft_peak_hz);
            } else {
                printf("RMS=%.0f 弱信号/噪声，定位不可靠\n", rms);
            }
        }

        /* ================= 屏幕显示：上下对比 ================= */
        if (lcd_ret == ESP_OK) {
            TickType_t now_tick = xTaskGetTickCount();

            /* 波形区：左右声道时域波形并排（运算前）。离线则清空画直线。 */
            if (s_view_l.offline) {
                memset(s_view_l.scroll, 0, sizeof(s_view_l.scroll));
                s_view_l.peak = 600;
            } else {
                scroll_push(s_view_l.scroll, s_l, n);
            }
            if (s_view_r.offline) {
                memset(s_view_r.scroll, 0, sizeof(s_view_r.scroll));
                s_view_r.peak = 600;
            } else {
                scroll_push(s_view_r.scroll, s_r, n);
            }
            draw_wave(&s_view_l);
            draw_wave(&s_view_r);

            /* 下半屏：FFT 幅度谱（运算后），展示时域波纹如何变成频域主峰 */
            draw_spectrum(s_fft_mag_l);

            /* 文字信息(方向指示 + 各标签)单独节流，避免频繁重画闪烁 */
            if (now_tick - last_hud_tick >= hud_period) {
                last_hud_tick = now_tick;
                draw_hud(have_t, t_angle, t_rho, have_f, s_fft_angle_ema,
                         s_fft_quality, s_fft_peak_hz);
                draw_label(&s_view_l);
                draw_label(&s_view_r);
                draw_spec_label();
            }
        }
    }
}