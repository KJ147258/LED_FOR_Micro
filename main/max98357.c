/*
 * MAX98357A 数字功放驱动 —— I2S 输出实现
 *
 * 接线（对应 max98357.h 中的宏）：
 *   ESP32                    MAX98357 模块
 *   GPIO26 (AMP_BCK_GPIO) -> BCLK
 *   GPIO25 (AMP_WS_GPIO)  -> LRC / WS
 *   GPIO27 (AMP_DIN_GPIO) -> DIN
 *   GPIO32 (AMP_SD_GPIO)  -> SD(使能，>1.4V 使能；不用可改宏为 -1)
 *   3V3 / GND             -> VDD / GND
 *
 * 时序：MAX98357A 接收标准 I2S(Philips) 16bit 数据，在 BCLK 上升沿采样，
 * LRCLK 低电平为左声道。ESP32 的 I2S 外设一个 WS 周期固定出左右两个 slot，
 * 16bit 时 BCLK = 32×fs，落在芯片支持范围(32/48/64×fs)内；MONO 模式只发
 * 左声道数据，功放只取左声道，右声道 slot 被忽略。
 */
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_log.h"

#include "max98357.h"

static const char *TAG = "max98357";

/* 播放测试音时单次写入的采样块大小（栈上分配，避免过大） */
#define TONE_CHUNK_SAMPLES  512

static i2s_chan_handle_t s_tx_chan = NULL;
static bool s_enabled = false;

/* ================= 初始化 / 反初始化 ================= */

esp_err_t max98357_init(void)
{
    if (s_tx_chan != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 1. 先把 SD 拉低(关断)，并配置为推挽输出；初始化完成前不让功放出声，
     *    也避免 I2S 时钟尚未稳定时功放直接输出爆音。 */
    if (AMP_SD_GPIO >= 0) {
        gpio_config_t sd_cfg = {
            .pin_bit_mask = 1ULL << AMP_SD_GPIO,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = 0,
            .pull_down_en = 0,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        esp_err_t ret = gpio_config(&sd_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "config SD gpio failed");
            return ret;
        }
        gpio_set_level(AMP_SD_GPIO, 0);
    }

    /* 2. 创建 I2S TX 通道（麦克风占用 I2S_NUM_0，功放用 I2S_NUM_1，互不干扰） */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(AMP_I2S_NUM, I2S_ROLE_MASTER);
    esp_err_t ret = i2s_new_channel(&chan_cfg, &s_tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "create tx channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 3. 标准 I2S(Philips) 模式：16bit 单声道，无 MCLK */
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AMP_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(AMP_BITS_PER_SAMPLE, AMP_CHANNELS),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = AMP_BCK_GPIO,
            .ws   = AMP_WS_GPIO,
            .dout = AMP_DIN_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ret = i2s_channel_init_std_mode(s_tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init std mode failed: %s", esp_err_to_name(ret));
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
        return ret;
    }

    /* 4. 启动 I2S 并先灌一小段静音，再拉高 SD 使能：保证功放一放行，总线上
     *    的已经是 0 数据，消除上电瞬间的噪声。 */
    ret = i2s_channel_enable(s_tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "enable channel failed: %s", esp_err_to_name(ret));
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
        return ret;
    }

    int16_t silence[TONE_CHUNK_SAMPLES] = {0};
    size_t written = 0;
    i2s_channel_write(s_tx_chan, silence, sizeof(silence), &written, pdMS_TO_TICKS(200));

    if (AMP_SD_GPIO >= 0) {
        gpio_set_level(AMP_SD_GPIO, 1);
    }
    s_enabled = true;

    ESP_LOGI(TAG, "ready: BCK=%d WS=%d DIN=%d SD=%d fs=%dHz",
             AMP_BCK_GPIO, AMP_WS_GPIO, AMP_DIN_GPIO, AMP_SD_GPIO, AMP_SAMPLE_RATE);
    return ESP_OK;
}

esp_err_t max98357_deinit(void)
{
    if (s_tx_chan == NULL) {
        return ESP_OK;
    }

    /* 先关断功放，再停时钟/释放通道，避免移除 BCLK/WS 时的异常输出 */
    if (AMP_SD_GPIO >= 0) {
        gpio_set_level(AMP_SD_GPIO, 0);
    }
    i2s_channel_disable(s_tx_chan);
    i2s_del_channel(s_tx_chan);
    s_tx_chan = NULL;
    s_enabled = false;

    return ESP_OK;
}

/* ================= 运行时控制 ================= */

esp_err_t max98357_set_enable(bool enabled)
{
    if (s_tx_chan == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (AMP_SD_GPIO >= 0) {
        gpio_set_level(AMP_SD_GPIO, enabled ? 1 : 0);
    }
    s_enabled = enabled;
    return ESP_OK;
}

bool max98357_is_ready(void)
{
    return (s_tx_chan != NULL) && s_enabled;
}

/* ================= 数据输出 ================= */

esp_err_t max98357_write(const void *data, size_t bytes, uint32_t timeout_ms)
{
    if (s_tx_chan == NULL || !s_enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || bytes == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    TickType_t ticks = (timeout_ms == UINT32_MAX)
                       ? portMAX_DELAY
                       : pdMS_TO_TICKS(timeout_ms);

    size_t written = 0;
    return i2s_channel_write(s_tx_chan, data, bytes, &written, ticks);
}

/* ================= 测试音 ================= */

esp_err_t max98357_play_tone(uint16_t freq_hz, uint32_t dur_ms, int16_t amplitude)
{
    if (!max98357_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (freq_hz == 0 || amplitude <= 0 || dur_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const double two_pi = 6.283185307179586;
    const double phase_inc = two_pi * (double)freq_hz / (double)AMP_SAMPLE_RATE;
    const uint32_t total = (uint32_t)(((uint64_t)AMP_SAMPLE_RATE * dur_ms) / 1000);

    int16_t buf[TONE_CHUNK_SAMPLES];
    double phase = 0.0;
    uint32_t remaining = total;

    while (remaining > 0) {
        int n = (remaining < TONE_CHUNK_SAMPLES) ? (int)remaining : TONE_CHUNK_SAMPLES;
        for (int i = 0; i < n; i++) {
            buf[i] = (int16_t)((double)amplitude * sin(phase));
            phase += phase_inc;
            if (phase >= two_pi) {
                phase -= two_pi;
            }
        }

        size_t written = 0;
        esp_err_t ret = i2s_channel_write(s_tx_chan, buf,
                                          (size_t)n * sizeof(int16_t),
                                          &written, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK) {
            return ret;
        }
        remaining -= (uint32_t)n;
    }

    return ESP_OK;
}