/*
 * ST7789 屏幕驱动演示程序
 * 初始化屏幕后循环显示测试图案，验证驱动是否正常工作。
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "st7789.h"

static const char *TAG = "app_main";

/* 纯色切换测试 */
static void demo_solid_colors(void)
{
    static const uint16_t colors[] = {
        ST7789_COLOR_RED, ST7789_COLOR_GREEN, ST7789_COLOR_BLUE,
        ST7789_COLOR_YELLOW, ST7789_COLOR_CYAN, ST7789_COLOR_MAGENTA,
        ST7789_COLOR_WHITE, ST7789_COLOR_BLACK,
    };
    for (size_t i = 0; i < sizeof(colors) / sizeof(colors[0]); i++) {
        st7789_fill_rect(0, 0, ST7789_H_RES, ST7789_V_RES, colors[i]);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

/* 渐变测试：从上到下红色渐变 + 从左到右绿色渐变 */
static void demo_gradient(void)
{
    uint16_t line[ST7789_H_RES];
    for (int y = 0; y < ST7789_V_RES; y++) {
        uint8_t r = (uint8_t)(y * 255 / ST7789_V_RES);
        for (int x = 0; x < ST7789_H_RES; x++) {
            uint8_t g = (uint8_t)(x * 255 / ST7789_H_RES);
            line[x] = ST7789_RGB(r, g, 0);
        }
        st7789_draw_bitmap(0, y, ST7789_H_RES, y + 1, line);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}

/* 横向色条测试 */
static void demo_color_bars(void)
{
    static const uint16_t bars[] = {
        ST7789_COLOR_WHITE, ST7789_COLOR_YELLOW, ST7789_COLOR_CYAN,
        ST7789_COLOR_GREEN, ST7789_COLOR_MAGENTA, ST7789_COLOR_RED,
        ST7789_COLOR_BLUE, ST7789_COLOR_BLACK,
    };
    const int bar_w = ST7789_H_RES / 8;
    for (int i = 0; i < 8; i++) {
        st7789_fill_rect(i * bar_w, 0, (i + 1) * bar_w, ST7789_V_RES, bars[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}

/* 文字 + 图形展示画面 */
static void demo_info_screen(void)
{
    st7789_fill_rect(0, 0, ST7789_H_RES, ST7789_V_RES, ST7789_COLOR_BLACK);

    /* 白色边框 */
    st7789_draw_rect(2, 2, ST7789_H_RES - 3, ST7789_V_RES - 3, ST7789_COLOR_WHITE);

    /* 标题文字（scale=2 放大两倍） */
    st7789_draw_text(24, 24, "Hello ST7789!", ST7789_COLOR_WHITE, 2);
    st7789_draw_text(24, 52, "240 x 320", ST7789_COLOR_CYAN, 2);
    st7789_draw_text(24, 74, "ESP32 + esp_lcd", ST7789_COLOR_GREEN, 1);

    /* 图形：实心矩形 + 空心矩形边框 */
    st7789_fill_rect(24, 100, 104, 160, ST7789_COLOR_RED);
    st7789_draw_rect(24, 100, 104, 160, ST7789_COLOR_YELLOW);
    st7789_fill_rect(116, 100, 196, 160, ST7789_COLOR_BLUE);
    st7789_draw_rect(116, 100, 196, 160, ST7789_COLOR_WHITE);

    /* 斜线（Bresenham） */
    st7789_draw_line(24, 200, 216, 280, ST7789_COLOR_MAGENTA);
    st7789_draw_line(24, 280, 216, 200, ST7789_COLOR_YELLOW);

    /* 底部小字 */
    st7789_draw_text(24, 300, "ST7789 OK", ST7789_COLOR_GREEN, 1);

    vTaskDelay(pdMS_TO_TICKS(3000));
}

void app_main(void)
{
    ESP_LOGI(TAG, "initializing st7789 (%dx%d)...", ST7789_H_RES, ST7789_V_RES);

    esp_err_t ret = st7789_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "st7789 init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "st7789 ready, starting demo");

    while (1) {
        demo_solid_colors();
        demo_gradient();
        demo_color_bars();
        demo_info_screen();
    }
}
