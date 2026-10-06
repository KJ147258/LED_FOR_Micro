| Supported Targets | ESP32 |
| ----------------- | ----- |

# 双麦克风声源定位 + ST7789 频谱显示

这是一个 **ESP-IDF (ESP32)** 嵌入式项目：用两片 **INMP441** 数字麦克风做到达时间差（TDOA）
声源定位，在 **ST7789 (240x320 SPI LCD)** 上实时显示时域波形与 FFT 频谱，并可选通过
**MAX98357A** 功放回放采集到的声音。

定位同时给出两种互相印证的算法结果：

- **时域**：两路信号的归一化互相关（NCC），再用抛物线插值得到亚采样时延；
- **频域**：radix-2 FFT 后，用左右两路同一频率成分的相位差估计时延。

## 硬件接线

| 功能 | ESP32 GPIO |
| ---- | ---------- |
| 麦克风 I2S BCK（两片 SCK 并联） | 14 |
| 麦克风 I2S WS（两片 WS 并联）    | 15 |
| 麦克风 I2S DIN（两片 DOUT 并联） | 22 |
| 功放 BCK（MAX98357 BCLK）        | 26 |
| 功放 WS（MAX98357 LRC）          | 25 |
| 功放 DIN（MAX98357 DIN）         | 27 |
| 功放 SD（MAX98357 使能，可选）   | 32 |
| LCD SCLK                         | 18 |
| LCD MOSI (SDA)                   | 23 |
| LCD CS                           | 5  |
| LCD DC                           | 4  |
| LCD RST                          | 3  |
| LCD BL（背光）                   | 2  |

> 引脚全部为宏定义，可按实际接线在 `main/inmp441_doa.c`、`main/max98357.h`、
> `main/st7789.h` 中修改。

两片麦克风的 **SCK/WS 并联**，**DOUT 并联到同一根线**；一片的 `L/R` 接地占左声道，
另一片的 `L/R` 接 3V3 占右声道，从而在一条 I2S 数据线上同步采样左右两路。

## 目录结构

```
main/
├── inmp441_doa.c       # 主程序：I2S 采集、定位、显示、回放调度
├── dsp_fft.c/.h        # 手写 radix-2 FFT + 汉宁窗 + 频域相位差定位
├── st7789.c/.h         # ST7789 驱动（基于 ESP-IDF 原生 esp_lcd）与绘图原语
├── max98357.c/.h       # MAX98357A I2S 功放驱动
└── CMakeLists.txt      # 编译上述 4 个源文件
```

## 构建与烧录

本项目目标芯片为 **ESP32**（默认无 PSRAM）。使用 ESP-IDF 5.x：

```bash
idf.py set-target esp32
idf.py build
idf.py -p COM15 flash monitor
```

串口输出会实时打印左右声道 RMS、时域/频域定位结果对照；屏幕顶部为方向指针
HUD（时域/频域定位角对照），波形区左右并排显示左右声道时域波形，下半屏为 FFT 幅度谱。

## 备注

- 程序里有 3 个 `app_main` 的候选入口（`inmp441_doa.c`、`hello_world_main.c`、
  `esp_lcd_jbrilha_demo.c`），当前构建只编译 `inmp441_doa.c`（见
  `main/CMakeLists.txt`）。切换 demo 时请保证一次只启用一个入口文件，避免重复符号。
- FFT 点数 `FFT_N`、采样率 `SAMPLE_RATE`、麦克风间距 `MIC_DIST_M` 等关键参数均在
  源文件顶部用宏定义，可按需调整。