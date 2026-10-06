/*
 * MAX98357A 数字功放 —— 引脚定义
 *
 * MAX98357 是 I2S 数字输入的单声道 D 类功放：ESP32 用三根 I2S 线把音频
 * 数据发给它，它解码、放大后直接推喇叭（BTL 差分输出）。
 *
 * 引脚规划（已避让本项目现有占用）：
 *   已占用：屏幕 SPI  = 18/23/5/4/3/2
 *           麦克风 I2S_0 = 14(BCK) / 15(WS) / 22(DIN)
 *   故功放用 I2S_NUM_1 和另一组脚。
 */
#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2s_std.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================= 引脚配置（按你的实际接线修改） ================= */
#define AMP_I2S_NUM     I2S_NUM_1   /* 麦克风已占用 I2S_NUM_0，功放用第 2 个外设 */
#define AMP_BCK_GPIO    26          /* 位时钟 BCLK，接模块 BCLK */
#define AMP_WS_GPIO     25          /* 帧时钟 LRC / WS，接模块 LRC */
#define AMP_DIN_GPIO    27          /* 数据输出，接模块 DIN */
#define AMP_SD_GPIO     32          /* 可选：关断/使能，>1.4V 使能；不用可悬空(模块默认使能) */

/* ================= 增益说明（硬件配置，非代码） =================
 * GAIN 引脚默认悬空即可（9dB）。播放中不要改 GAIN，否则会爆音。
 *   悬空             -> 9dB   (默认)
 *   直接接 GND        -> 12dB
 *   100kΩ 电阻接 GND  -> 15dB
 *   直接接 VDD        -> 6dB
 *   100kΩ 电阻接 VDD  -> 3dB
 */

/* ================= 音频格式 ================= */
#define AMP_SAMPLE_RATE      48000    /* 采样率 Hz */
#define AMP_BITS_PER_SAMPLE  I2S_DATA_BIT_WIDTH_16BIT  /* 每采样位数 */
#define AMP_CHANNELS         I2S_SLOT_MODE_MONO         /* 单声道功放 */
/* 说明：ESP32 的 I2S 外设一个 WS 周期始终按左右两个 slot 出时钟，16bit 数据
 * 对应 BCLK = 32×fs，恰好在 MAX98357A 支持范围(32/48/64×fs)内。MONO 模式下
 * 只填充左声道 slot，功放只取左声道(LRCLK 低电平半周期)，右声道 slot 被忽略。 */

/* ================= 驱动接口 ================= */

/**
 * @brief 初始化 MAX98357：创建 I2S TX 通道、配置 BCK/WS/DIN、控制 SD 使能。
 *        上电顺序为“SD 拉低 → I2S 启动并灌静音 → SD 拉高”，避免爆音。
 *        重复调用前需先 max98357_deinit()。
 * @return ESP_OK 成功，否则为错误码
 */
esp_err_t max98357_init(void);

/**
 * @brief 反初始化：先拉低 SD 关断，再停 I2S 并释放通道。
 * @return ESP_OK
 */
esp_err_t max98357_deinit(void);

/**
 * @brief 使能/关断功放输出（控制 SD 引脚；AMP_SD_GPIO 为 -1 时仅记录状态）。
 * @param enabled true=使能(SD 拉高)，false=关断(SD 拉低)
 * @return ESP_OK；未初始化时返回 ESP_ERR_INVALID_STATE
 */
esp_err_t max98357_set_enable(bool enabled);

/**
 * @brief 查询功放是否已初始化且处于使能状态。
 */
bool max98357_is_ready(void);

/**
 * @brief 向功放写入 PCM 数据（阻塞，直到写完或超时）。
 *        数据格式：16bit 有符号、单声道(每个采样一个 int16_t)，与 AMP_* 宏一致。
 * @param data       样本数据指针
 * @param bytes      字节数，须为 sizeof(int16_t) 的整数倍
 * @param timeout_ms 超时(ms)；传 UINT32_MAX 表示无限等待
 * @return ESP_OK 成功；未就绪返回 ESP_ERR_INVALID_STATE；超时返回 ESP_ERR_TIMEOUT
 */
esp_err_t max98357_write(const void *data, size_t bytes, uint32_t timeout_ms);

/**
 * @brief 播放固定频率、固定时长的正弦波测试音（阻塞，供上电自检/听声验证）。
 * @param freq_hz   频率(Hz)，须为正数
 * @param dur_ms    持续时长(ms)
 * @param amplitude 幅度(1..32767)
 * @return ESP_OK 成功
 */
esp_err_t max98357_play_tone(uint16_t freq_hz, uint32_t dur_ms, int16_t amplitude);

#ifdef __cplusplus
}
#endif