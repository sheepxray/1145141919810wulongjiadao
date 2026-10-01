# wulong_fw - STM32G474VE 主控固件工程

CubeMX 配置 + MDK-ARM 编译通过，已接入 FreeRTOS 的工程骨架；外设按
`../docs/STM32G474_TB6612_引脚规划.md` 配置。

## 编译状态

```
Program Size: Code=17114  RO-data=570  RW-data=132  ZI-data=21972
"wulong_fw\wulong_fw.axf" - 0 Error(s), 0 Warning(s)
```

- 工具链：Keil MDK 5.23（已授权 mdk_std）
- 器件包：Keil.STM32G4xx_DFP 1.6.0
- HAL：STM32Cube FW_G4 V1.5.2（与 DFP 1.6.0 对应）
- RTOS：FreeRTOS Kernel V10.3.1，CMSIS-RTOS v1 工程接口 + 原生 FreeRTOS API
- 系统时钟：HSI 16 MHz -> PLL 170 MHz（Boost 模式，Flash Latency 4）
- HAL 时基：TIM6，1 ms 中断；SysTick 专供 FreeRTOS 调度器
- 打开方式：`MDK-ARM/wulong_fw.uvprojx`
- 引脚表：`docs/引脚分配表.md` + `docs/引脚分配表_pinout.csv`

> ⚠️ CubeMX 重新生成时，`.ioc` 与输出路径必须是全英文，否则生成报错。
> 工程内路径含中文（`乌龙驾到`），已实测**编译可用**；
> 但重新生成代码建议在纯英文目录做，再拷回来。

## 已配置外设

| 外设 | 用途 | 引脚 |
|---|---|---|
| TIM1 | 4 路电机 PWM，10 kHz，ARR=16999 | PA8/PA9/PA10/PA11 |
| TIM2/3/4/20 | 4 路编码器（硬件正交解码 TI12） | PA0-1 / PA6-7 / PB6-7 / PE2-3 |
| USART1 | 调试串口 115200 | PC4(TX)/PC5(RX) |
| USART2 | K230D 通信 921600 + DMA | PA2(TX)/PA3(RX) |
| SPI2 | ICM-20602 IMU（主模式） | PB13/14/15，CS=PB12，INT=**PB9** |
| ADC2 | 电池电压（IN17） | PA4 |
| TIM6 | HAL 时基 1 ms（FreeRTOS 下替代 SysTick） | 内部 |
| FreeRTOS | 控制 / 通信 / 策略三任务骨架 | Cortex-M4F 端口 |
| GPIO 输出 | 8 路方向线 + STBY + 蜂鸣器 + 自检 LED | 见 gpio.c 标签 |
| GPIO 输入 | 拨码 DIP1-4 | PB4/PB5/PC7/PB3 |

方向线标签：`AIN1/AIN2`=PB0/PB1，`BIN1/BIN2`=PE4/PE5，
`CIN1/CIN2`=PB10/PB11，`DIN1/DIN2`=PD2/PC12，`STBY`=PC6。

## K230D 接线（立创庐山派 Lite-K230D）

GH1.25-4P 座子线序（官方）：`5V / RXD / TXD / GND`

| 座子脚 | 信号 | 接到 STM32 |
|---|---|---|
| 1 | 5V | 不接（K230D 由独立 DC-DC 供电） |
| 2 | RXD | PA2（USART2_TX） |
| 3 | TXD | PA3（USART2_RX） |
| 4 | GND | GND（必须共地） |

K230D 侧对应 **UART2 = GPIO11(TXD) / GPIO12(RXD)**。
⚠️ **不要用 UART0**（GPIO38/39）——被系统 RT-Smart 占用作调试台。

## FreeRTOS 接入

- `configTICK_RATE_HZ = 1000`，抢占式调度。
- 控制任务 1 kHz，通信任务 5 ms 周期，策略任务 10 ms 周期。
- `FreeRTOS` 的 `SVC_Handler`、`PendSV_Handler`、`SysTick_Handler` 由
  `port.c` 接管；HAL 的 `HAL_IncTick()` 由 TIM6 回调执行。
- 调用 `...FromISR()` 的中断优先级必须不高于
  `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`（数值 >= 5）。

## 待办

1. **板载占用核对**：PE2-PE5 是否被 LCD/FSMC 占用，需对照 M 板原理图确认。
2. 业务代码：`bsp/ control/ localization/ strategy/ comms/` 骨架尚未创建；
   任务入口已经在 `Core/Src/main.c` 建立。

> PB3 冲突已解决：拨码占 PB3（DIP4），IMU INT 改到 **PB9**（EXTI9_5）。
