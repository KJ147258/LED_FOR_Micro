/*
 * ST7789 屏幕驱动 (240x320, SPI 接口)
 * 基于 ESP-IDF 原生 esp_lcd 组件，无需第三方库。
 *
 * 接线默认值参考 ESP-IDF 官方示例，请按你的实际接线修改下面的宏。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================= 引脚配置（按你的实际接线修改） ================= */
#define ST7789_PIN_SCLK   18          /*!< SPI 时钟 */
#define ST7789_PIN_MOSI   23          /*!< SPI 数据 (SDA) */
#define ST7789_PIN_CS      5          /*!< 片选 */
#define ST7789_PIN_DC      4          /*!< 数据/命令选择 */
#define ST7789_PIN_RST     3          /*!< 复位，-1 表示不使用 */
#define ST7789_PIN_BL      2          /*!< 背光，-1 表示不使用 */

/* ================= 屏幕参数 ================= */
#define ST7789_H_RES       240        /*!< 水平分辨率 */
#define ST7789_V_RES       320        /*!< 垂直分辨率 */
#define ST7789_SPI_CLK_HZ  (40 * 1000 * 1000)  /*!< SPI 时钟频率 */
#define ST7789_INVERT_COLOR 1         /*!< ST7789 通常需要颜色反转；若颜色异常可改为 0 */

/* ================= 常用 RGB565 颜色 ================= */
#define ST7789_COLOR_BLACK    0x0000
#define ST7789_COLOR_WHITE    0xFFFF
#define ST7789_COLOR_RED      0xF800
#define ST7789_COLOR_GREEN    0x07E0
#define ST7789_COLOR_BLUE     0x001F
#define ST7789_COLOR_YELLOW   0xFFE0
#define ST7789_COLOR_CYAN     0x07FF
#define ST7789_COLOR_MAGENTA  0xF81F

/* 由 8bit RGB 分量合成 RGB565 颜色 */
#define ST7789_RGB(r, g, b) \
    ((((uint16_t)(r) & 0xF8) << 8) | (((uint16_t)(g) & 0xFC) << 3) | (((uint16_t)(b) & 0xF8) >> 3))

/**
 * @brief 初始化 SPI 总线、创建 ST7789 面板并开启背光
 * @return ESP_OK 成功，否则为错误码
 */
esp_err_t st7789_init(void);

/**
 * @brief 以单色填充矩形区域 (x0,y0)-(x1,y1)，区间左闭右开
 */
esp_err_t st7789_fill_rect(int x0, int y0, int x1, int y1, uint16_t color);

/**
 * @brief 绘制 RGB565 位图，data 长度需为 (x1-x0)*(y1-y0)
 */
esp_err_t st7789_draw_bitmap(int x0, int y0, int x1, int y1, const uint16_t *data);

/**
 * @brief 开关背光
 */
void st7789_set_backlight(bool on);

/* ================= 基本绘图原语 ================= */

/**
 * @brief 画单个像素
 */
esp_err_t st7789_draw_pixel(int x, int y, uint16_t color);

/**
 * @brief 画直线（Bresenham）
 */
void st7789_draw_line(int x0, int y0, int x1, int y1, uint16_t color);

/**
 * @brief 画空心矩形边框，坐标左闭右开
 */
void st7789_draw_rect(int x0, int y0, int x1, int y1, uint16_t color);

/**
 * @brief 画单个 ASCII 字符（5x7 点阵），scale 为放大倍数，透明背景
 */
void st7789_draw_char(int x, int y, char ch, uint16_t color, int scale);

/**
 * @brief 画字符串，逐字符排列（字符宽 6*scale 像素）
 */
void st7789_draw_text(int x, int y, const char *str, uint16_t color, int scale);

#ifdef __cplusplus
}
#endif
