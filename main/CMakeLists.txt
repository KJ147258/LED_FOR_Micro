# 当前入口：双 INMP441 声源定位 + ST7789 波形/FFT 频谱显示
idf_component_register(SRCS "dsp_fft.c" "max98357.c" "inmp441_doa.c" "st7789.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_lcd)

# 切换回 ST7789 屏幕 demo 时：注释掉上面，启用下面两行
# idf_component_register(SRCS "esp_lcd_jbrilha_demo.c"
#                     INCLUDE_DIRS "."
#                     REQUIRES esp_lcd jbrilha__esp_lcd_st7789)
