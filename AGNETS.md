# AGNETS.md

本文件记录 `reference/STM32_H743_M核心板-原理图-1909M.pdf` 中会影响代码、CubeMX 配置、引脚分配和调试方式的核心板信息。后续修改固件时优先遵守这些板级约束。

## 核心板基本信息

- 核心板：`H743VI 核心板（M版）`
- 当前工程目标芯片：`STM32H743VIT6`
  - `H743VI`：片内 Flash 2048 KB，RAM 1024 KB。
  - 改链接脚本、分区、下载算法前先确认实物芯片丝印；本工程按 `H743VI` 使用。

## 时钟与启动

- HSE 外部晶振：`25 MHz`，连接 `PH0-OSC_IN` / `PH1-OSC_OUT`。
  - 现有 `SystemClock_Config()` 使用 `PLLM=5, PLLN=192, PLLP=2`，等价于 25 MHz HSE 输入下生成 480 MHz SYSCLK；不要按 8 MHz HSE 改 PLL。
- LSE 外部 32.768 kHz 晶振：连接 `PC14-OSC32_IN` / `PC15-OSC32_OUT`。
  - 若启用 RTC，应配置 LSE，不要把 `PC14/PC15` 当普通 GPIO 使用。
- `BOOT0` 通过 `R7=10K` 默认下拉到 GND，上电默认从用户 Flash 启动。
  - 需要 ISP/系统 Bootloader 时必须显式拉高 `BOOT0`。
- 板上有 `NRST/RST` 复位按键；调试或低功耗代码不要占用复位功能。

## 供电与电平


- USB 口可供电，也连接到 MCU USB FS 数据线；启用 USB 设备功能时需同时考虑供电、时钟和 PA11/PA12 引脚占用。

## 下载调试保留脚

- SWD 调试接口使用：
  - `PA13` = `SWDIO`
  - `PA14` = `SWCLK`
- 固件和 CubeMX 不要把 `PA13/PA14` 重分配给业务外设，否则会影响下载和在线调试。

## 板载/核心板接口占用

这些资源在核心板原理图上已经连到板载器件或固定接口。除非明确不使用对应板载功能，否则写代码和分配引脚时应默认保留。

### 板载 QSPI Flash：W25QXX

外部 Flash `U3=W25QXX` 连接到 QUADSPI：

| 信号 | MCU 引脚 |
|---|---|
| `QSPI_BK1_NCS` | `PB6` |
| `QUADSPI_CLK` | `PB2` |
| `QUADSPI_BK1_IO0` | `PD11` |
| `QUADSPI_BK1_IO1` | `PD12` |
| `QUADSPI_BK1_IO2` | `PE2` |
| `QUADSPI_BK1_IO3` | `PD13` |

代码影响：

- 如需使用板载 W25QXX，必须启用 QUADSPI 并保留以上引脚。
- STM32H7 开启 D-Cache 后，QSPI 存储映射、DMA 读写、文件系统缓存需要同步考虑 MPU/Cache 一致性。
- 当前工程中 `PB2` 被用作 `CAR_RIGHT_BIN1`，`PB6` 被用作 `CAR_BEEP`，会与板载 QSPI Flash 的 `CLK/NCS` 冲突；若要使用板载 Flash，应重新分配这些业务引脚。

### SD/TF 卡座：SDMMC1

板载 MiniSD/TF 卡座 `U5` 连接到 `SDMMC1`：

| 信号 | MCU 引脚 |
|---|---|
| `SDIO_D0` | `PC8` |
| `SDIO_D1` | `PC9` |
| `SDIO_D2` | `PC10` |
| `SDIO_D3` | `PC11` |
| `SDIO_CK` | `PC12` |
| `SDIO_CMD` | `PD2` |

代码影响：

- 启用 SD 卡/FatFs 时保留 `PC8-PC12` 和 `PD2`，配置 `SDMMC1`。
- SDMMC + DMA 在 H7 上同样需要检查 D-Cache 维护和缓冲区内存区域。

### USB FS Device

Micro USB 接口 `J5` 连接：

| 信号 | MCU 引脚 |
|---|---|
| `USB_DM` | `PA11` |
| `USB_DP` | `PA12` |

代码影响：

- 启用 USB Device 时保留 `PA11/PA12`，并配置满足 USB 的 48 MHz 时钟。
- 不使用 USB 数据功能时，Micro USB 仍可能作为 5 V 供电入口。

### DCMI 摄像头接口

FPC 摄像头接口 `J6` 支持 OV2640/OV5640 类模块，主要连接如下：

| 摄像头信号 | MCU 引脚 |
|---|---|
| `DCMI_D0` | `PC6` |
| `DCMI_D1` | `PC7` |
| `DCMI_D2` | `PE0` |
| `DCMI_D3` | `PE1` |
| `DCMI_D4` | `PE4` |
| `DCMI_D5` | `PD3` |
| `DCMI_D6` | `PE5` |
| `DCMI_D7` | `PE6` |
| `DCMI_PCLK` | `PA6` |
| `DCMI_XCLK` | `PA8` |
| `DCMI_HREF/HSYNC` | `PA4` |
| `DCMI_VSYNC` | `PB7` |
| `DCMI_PWDN` | `PA7` |
| `DCMI_RESET` | `PC4` |
| `DCMI_SCL` | `PB10` |
| `DCMI_SDA` | `PB11` |

代码影响：

- 启用摄像头/图像识别时，应优先保留这组 DCMI + I2C/SCCB 引脚。
- 当前工程中 `PA6/PA7` 用作 TIM3 电机 PWM，`PC6/PC7` 后续可能用作编码器，均会与摄像头接口冲突。

### SPI TFT/OLED 接口

板上预留 SPI 屏接口，原理图标注可接 OLED/TFT。相关信号包括：

| 屏接口信号 | MCU 引脚 |
|---|---|
| `SCK/SCL` | `PB13` |
| `CS` | `PB12` |
| `MISO/SDO` | `PB14` |
| `MOSI/SDI` | `PB15` |
| `D/C` / `RS` | `PB1` |
| `BLK` | `PB0` |

代码影响：

- 若启用板载屏接口，优先使用 `SPI2` 相关引脚并保留 `PB0/PB1/PB12-PB15`。
- 当前工程把 `PB0/PB1` 用作左电机方向，把 `PB14/PB15` 用作调试串口相关引脚；接屏时需要重新规划。

## 板载按键与 LED

- 电源指示灯 `D1` 接 `3V3`，只表示板上供电状态，软件不可控。
- 用户 LED `D2` 原理图连接到 `PA1`，可作为软件控制 LED 使用。
- 用户按键：
  - `K1`：原理图标注连接 `PE3`。
  - `K2`：原理图标注连接 `PC5`。
  - 编写按键驱动前，应按实物验证按下电平；原理图表现为按键到 GND，通常按输入上拉、低电平有效处理。

## 当前工程特别注意

- 当前项目已经启用 MPU、I-Cache、D-Cache。后续新增 SDMMC、QSPI、USB、DCMI、UART DMA 等外设时，需要同步处理 DMA 缓冲区所在内存区域和 Cache 清理/失效。
- `PC2_C` / `PC3_C` 在 H743 上属于特殊 `_C` 引脚；当前工程中 `PC2_C` 作为灰度备用输入，`PC3_C` 用于感为八路灰度传感器串行 `GRAY_DAT`。保持 CubeMX 生成的相关 SYSCFG/Analog Switch 配置，不要手写删改。
- 规划新增外设前，先检查本文件中的板载占脚，再检查 `2021f.ioc` 和 `当前主程序功能与引脚总表.md` 中的业务占脚，避免把核心板固定资源和小车外接模块叠在同一引脚上。
