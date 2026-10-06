/*
 * ST7789 点亮测试 — 使用 jbrilha/esp_lcd_st7789 库
 *
 * 通过 jbrilha 库提供的 esp_lcd_new_panel_st7789() 初始化屏幕，
 * 然后循环显示纯色，用于快速验证屏幕是否能亮。
 *
 * 引脚沿用 main/st7789.h 中的接线定义，请按你的实际接线修改下方宏。
 */
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7789.h" /* jbrilha/esp_lcd_st7789 */

static const char *TAG = "jbrilha_demo";

#define LCD_HOST SPI2_HOST

#define LCD_H_RES 240
#define LCD_V_RES 320
#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)

/* ================= 引脚配置（按你的实际接线修改） ================= */
#define PIN_CLK  18   /* SPI 时钟 */
#define PIN_MOSI 23   /* SPI 数据 (SDA) */
#define PIN_CS    5   /* 片选 */
#define PIN_DC    4   /* 数据/命令选择 */
#define PIN_RST   3   /* 复位，-1 表示不使用 */
#define PIN_BL    2   /* 背光，-1 表示不使用 */

static esp_lcd_panel_handle_t s_panel = NULL;

/* 以单色填充整个屏幕 */
static void fill_screen(uint16_t color);

static esp_err_t lcd_init(void)
{
    /* 先关闭背光，初始化完成后再打开 */
    if (PIN_BL >= 0) {
        gpio_config_t bl_cfg = {
            .pin_bit_mask = 1ULL << PIN_BL,
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&bl_cfg);
        gpio_set_level(PIN_BL, 0);
    }

    /* 1. 初始化 SPI 总线（MISO 未接，置 -1） */
    spi_bus_config_t bus_cfg = {
        .sclk_io_num = PIN_CLK,
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * 2, /* 一帧 RGB565 */
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_HOST, &bus_cfg, SPI_DMA_CH_AUTO),
                        TAG, "spi bus initialize failed");

    /* 2. 创建 SPI panel IO（命令/参数 8bit） */
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = PIN_DC,
        .cs_gpio_num = PIN_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0, /* ST7789 使用 SPI Mode 0 */
        .trans_queue_depth = 10,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                                 &io_cfg, &io),
                        TAG, "create panel io failed");

    /* 3. 使用 jbrilha 库创建 ST7789 面板 */
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16, /* RGB565 */
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel),
                        TAG, "create st7789 panel failed");

    /* 4. 复位并初始化面板 */
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(s_panel, true), TAG, "invert color failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "display on failed");

    /* 清屏为黑色 */
    fill_screen(0x0000);

    /* 5. 打开背光 */
    if (PIN_BL >= 0) {
        gpio_set_level(PIN_BL, 1);
    }

    ESP_LOGI(TAG, "st7789 initialized (%dx%d) via jbrilha/esp_lcd_st7789",
             LCD_H_RES, LCD_V_RES);
    return ESP_OK;
}

/* 以单色填充整个屏幕 */
static void fill_screen(uint16_t color)
{
    if (s_panel == NULL) {
        return;
    }
    uint16_t *line = malloc(LCD_H_RES * sizeof(uint16_t));
    if (line == NULL) {
        ESP_LOGE(TAG, "no mem for line buffer");
        return;
    }
    for (int i = 0; i < LCD_H_RES; i++) {
        line[i] = color;
    }
    for (int y = 0; y < LCD_V_RES; y++) {
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_H_RES, y + 1, line);
    }
    free(line);
}

/* 填充矩形区域 (x0,y0)-(x1,y1)，坐标左闭右开 */
static void fill_rect(int x0, int y0, int x1, int y1, uint16_t color)
{
    if (s_panel == NULL) {
        return;
    }
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > LCD_H_RES) x1 = LCD_H_RES;
    if (y1 > LCD_V_RES) y1 = LCD_V_RES;
    if (x1 <= x0 || y1 <= y0) {
        return;
    }
    uint16_t *line = malloc((size_t)(x1 - x0) * sizeof(uint16_t));
    if (line == NULL) {
        ESP_LOGE(TAG, "no mem for line buffer");
        return;
    }
    for (int i = 0; i < x1 - x0; i++) {
        line[i] = color;
    }
    for (int y = y0; y < y1; y++) {
        esp_lcd_panel_draw_bitmap(s_panel, x0, y, x1, y + 1, line);
    }
    free(line);
}

/* 静态彩色画面：四色分屏 + 白色十字，肉眼一眼可辨 */
static void demo_static_screen(void)
{
    fill_screen(0x001F);                  /* 蓝底 */
    fill_rect(0, 0, 120, 160, 0xF800);    /* 左上红 */
    fill_rect(120, 0, 240, 160, 0x07E0);  /* 右上绿 */
    fill_rect(0, 160, 120, 320, 0xFFE0);  /* 左下黄 */
    fill_rect(120, 160, 240, 320, 0xF81F);/* 右下品红 */
    fill_rect(110, 0, 130, 320, 0xFFFF);  /* 垂直白十字 */
    fill_rect(0, 150, 240, 170, 0xFFFF);  /* 水平白十字 */
}

void app_main(void)
{
    ESP_LOGI(TAG, "initializing st7789 via jbrilha/esp_lcd_st7789...");

    if (lcd_init() != ESP_OK) {
        ESP_LOGE(TAG, "lcd init failed");
        return;
    }

    /* 循环显示纯色，检查屏幕是否能亮 */
    static const uint16_t colors[] = {
        0xF800, /* red    */
        0x07E0, /* green  */
        0x001F, /* blue   */
        0xFFE0, /* yellow */
        0x07FF, /* cyan   */
        0xF81F, /* magenta */
        0xFFFF, /* white  */
        0x0000, /* black  */
    };

    while (1) {
        /* 静态彩色画面，停留 3 秒便于观察 */
        demo_static_screen();
        vTaskDelay(pdMS_TO_TICKS(3000));

        /* 循环显示纯色，检查屏幕是否能亮 */
        for (size_t i = 0; i < sizeof(colors) / sizeof(colors[0]); i++) {
            fill_screen(colors[i]);
            vTaskDelay(pdMS_TO_TICKS(800));
        }
    }
}
