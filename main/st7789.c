/*
 * ST7789 屏幕驱动实现 (240x320, SPI 接口)
 * 基于 ESP-IDF 原生 esp_lcd 组件。
 */
#include "st7789.h"
#include "st7789_font.h"

#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"

static const char *TAG = "st7789";

#define ST7789_SPI_HOST SPI2_HOST

static esp_lcd_panel_handle_t s_panel = NULL;

/* 填充矩形时分块高度，避免一次分配过大缓冲 */
#define FILL_CHUNK_ROWS 16

esp_err_t st7789_init(void)
{
    esp_err_t ret = ESP_OK;

    /* 先关闭背光，初始化完成后再打开 */
    if (ST7789_PIN_BL >= 0) {
        gpio_config_t bl_cfg = {
            .pin_bit_mask = 1ULL << ST7789_PIN_BL,
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&bl_cfg), TAG, "backlight gpio config failed");
        ESP_RETURN_ON_ERROR(gpio_set_level(ST7789_PIN_BL, 0), TAG, "backlight off failed");
    }

    /* 1. 初始化 SPI 总线（MISO 未接，置 -1） */
    spi_bus_config_t bus_cfg = {
        .sclk_io_num = ST7789_PIN_SCLK,
        .mosi_io_num = ST7789_PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ST7789_H_RES * ST7789_V_RES * 2, /* 一帧 RGB565 */
    };
    ret = spi_bus_initialize(ST7789_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    ESP_RETURN_ON_ERROR(ret, TAG, "spi bus initialize failed");

    /* 2. 创建 SPI panel IO（命令/参数 8bit，DC 脚控制数据/命令） */
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = ST7789_PIN_CS,
        .dc_gpio_num = ST7789_PIN_DC,
        .spi_mode = 0,                    /* ST7789 使用 SPI Mode 0 */
        .pclk_hz = ST7789_SPI_CLK_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)ST7789_SPI_HOST, &io_cfg, &io);
    ESP_RETURN_ON_ERROR(ret, TAG, "create panel io failed");

    /* 3. 创建 ST7789 面板 */
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = ST7789_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,   /* RGB565 大端 */
        .bits_per_pixel = 16,
        .flags.reset_active_high = false,         /* ST7789 低电平复位 */
    };
    ret = esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel);
    ESP_RETURN_ON_ERROR(ret, TAG, "create st7789 panel failed");

    /* 4. 复位并初始化面板 */
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(s_panel), err, TAG, "panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(s_panel), err, TAG, "panel init failed");

#if ST7789_INVERT_COLOR
    ESP_GOTO_ON_ERROR(esp_lcd_panel_invert_color(s_panel, true), err, TAG, "invert color failed");
#endif

    ESP_GOTO_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), err, TAG, "display on failed");

    /* 清屏为黑色 */
    ESP_GOTO_ON_ERROR(st7789_fill_rect(0, 0, ST7789_H_RES, ST7789_V_RES, ST7789_COLOR_BLACK),
                      err, TAG, "clear screen failed");

    /* 5. 打开背光 */
    if (ST7789_PIN_BL >= 0) {
        ESP_GOTO_ON_ERROR(gpio_set_level(ST7789_PIN_BL, 1), err, TAG, "backlight on failed");
    }

    ESP_LOGI(TAG, "st7789 initialized, %dx%d @ %d MHz", ST7789_H_RES, ST7789_V_RES,
             ST7789_SPI_CLK_HZ / 1000000);
    return ESP_OK;

err:
    return ret;
}

esp_err_t st7789_fill_rect(int x0, int y0, int x1, int y1, uint16_t color)
{
    if (s_panel == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > ST7789_H_RES) x1 = ST7789_H_RES;
    if (y1 > ST7789_V_RES) y1 = ST7789_V_RES;
    if (x1 <= x0 || y1 <= y0) {
        return ESP_OK;
    }

    const int width = x1 - x0;
    const int height = y1 - y0;
    uint16_t *chunk = malloc((size_t)width * FILL_CHUNK_ROWS * sizeof(uint16_t));
    if (chunk == NULL) {
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < width * FILL_CHUNK_ROWS; i++) {
        chunk[i] = color;
    }

    esp_err_t ret = ESP_OK;
    for (int y = y0; y < y1; y += FILL_CHUNK_ROWS) {
        int rows = (y + FILL_CHUNK_ROWS <= y1) ? FILL_CHUNK_ROWS : (y1 - y);
        ret = esp_lcd_panel_draw_bitmap(s_panel, x0, y, x1, y + rows, chunk);
        if (ret != ESP_OK) {
            break;
        }
    }

    free(chunk);
    return ret;
}

esp_err_t st7789_draw_bitmap(int x0, int y0, int x1, int y1, const uint16_t *data)
{
    if (s_panel == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_lcd_panel_draw_bitmap(s_panel, x0, y0, x1, y1, data);
}

void st7789_set_backlight(bool on)
{
    if (ST7789_PIN_BL >= 0) {
        gpio_set_level(ST7789_PIN_BL, on ? 1 : 0);
    }
}

/* ================= 基本绘图原语 ================= */

esp_err_t st7789_draw_pixel(int x, int y, uint16_t color)
{
    return st7789_draw_bitmap(x, y, x + 1, y + 1, &color);
}

void st7789_draw_line(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        st7789_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void st7789_draw_rect(int x0, int y0, int x1, int y1, uint16_t color)
{
    st7789_draw_line(x0, y0, x1, y0, color);
    st7789_draw_line(x1, y0, x1, y1, color);
    st7789_draw_line(x1, y1, x0, y1, color);
    st7789_draw_line(x0, y1, x0, y0, color);
}

void st7789_draw_char(int x, int y, char ch, uint16_t color, int scale)
{
    int idx = (unsigned char)ch - 0x20;
    if (idx < 0 || idx >= 95) {
        idx = '?' - 0x20;   /* 不支持的字形回退为 '?' */
    }
    if (scale < 1) {
        scale = 1;
    }

    const uint8_t *glyph = font_5x7[idx];
    for (int row = 0; row < 7; row++) {
        for (int col = 0; col < 5; col++) {
            if ((glyph[row] & (1 << (4 - col))) == 0) {
                continue;   /* 透明背景，只画前景 */
            }
            if (scale == 1) {
                st7789_draw_pixel(x + col, y + row, color);
            } else {
                st7789_fill_rect(x + col * scale, y + row * scale,
                                 x + (col + 1) * scale, y + (row + 1) * scale, color);
            }
        }
    }
}

void st7789_draw_text(int x, int y, const char *str, uint16_t color, int scale)
{
    if (scale < 1) {
        scale = 1;
    }
    while (*str != '\0') {
        st7789_draw_char(x, y, *str, color, scale);
        x += 6 * scale;   /* 5x7 字体，字符间留 1 像素间距 */
        str++;
    }
}
