# STM32G474 固件骨架

> 对应文档：[docs/03-电控与驱动系统](../../docs/03-电控与驱动系统.md)、
> [docs/05-定位与导航](../../docs/05-定位与导航.md)、
> [docs/07-软件架构与决策策略](../../docs/07-软件架构与决策策略.md)

## 目录结构

```
firmware/stm32/
  ├─ main.c                  初始化 + 主循环
  ├─ bsp/                    板级支持
  │    ├─ gpio.c/.h
  │    ├─ tim.c/.h           PWM + 编码器
  │    ├─ uart.c/.h          DMA + 空闲中断收帧
  │    └─ spi.c/.h
  ├─ control/
  │    ├─ pid.c/.h           通用 PID（位置式 + 抗积分饱和）
  │    ├─ speed.c/.h         速度环 1 kHz
  │    ├─ heading.c/.h       航向环 500 Hz
  │    └─ kinematics.c/.h    差速运动学分解
  ├─ localization/
  │    ├─ imu.c/.h           ICM20602 驱动 + 航向积分
  │    ├─ odometry.c/.h      航迹推算
  │    └─ relocalize.c/.h    撞墙重定位
  ├─ strategy/
  │    ├─ fsm.c/.h           状态机
  │    ├─ planner.c/.h       两段式路径生成
  │    └─ scoring.c/.h       目标打分函数
  └─ comms/
       └─ k230_link.c/.h      串口协议（docs/11）
```

## 配套资料

| 目录 | 内容 |
|---|---|
| datasheets/ | STM32G474VE 数据手册、M 板原理图 / 接口结构图、Altium 封装库（SCHLIB / PcbLib） |
| docs/can/ | FDCAN 外设、CAN_FD 配置、收发器芯片（SN65HVD230 / TJA1051 / NXP）手册 |
| docs/cubemx/ | STM32CubeMX 官方 UM 文档 |
| xamples/ | 厂商 DevEBox-G474 例程 22 个（LED/ADC/DAC/DMA/SPI/USB/LCD/CAN_FD/综合例程），本地参考，约 2.7GB，未入库（见 .gitignore） |

> 主控已由 **STM32G431 改为 STM32G474VE**（Cortex-M4，带 FDCAN / USB / LCD 接口）。外设引脚映射需按 G474 重新核对后再落地 sp/。

## 参考实现

`reference/` 目录下有两类参考：

### A. Python 算法参考（可直接运行）

用于验证算法后再移植到 C：

| 文件 | 内容 |
|---|---|
| `reference/kinematics.py` | 差速运动学、PID、航迹推算 |
| `reference/scoring.py` | 目标打分函数（含帅/将支配项与送分规避） |

**先跑 Python 确认算法正确，再移植** —— 在 MCU 上调试数学公式代价高得多。

### B. ICM20602 驱动参考（⚠️ 需移植）

`reference/icm20602/` 是厂商提供的三种接口实现：

| 目录 | 接口 |
|---|---|
| `hardware_spi/` | 硬件 SPI |
| `software_spi/` | 软件模拟 SPI |
| `software_iic/` | 软件模拟 I²C |

> ### ⚠️ 这份代码是 **STM32F103ZET6** 工程，不是 G474
>
> 三套 `.ioc` 的 `Mcu.Name` 均为 `STM32F103Z(C-D-E)Tx`。
> **直接照抄到 G474 会编译失败或行为异常。**
>
> 移植要点：
>
> | 项目 | F103（参考代码） | G474（本项目） |
> |---|---|---|
> | HAL 头文件 | `stm32f1xx_hal*.h` | `stm32g4xx_hal*.h` |
> | 时钟树 | 72MHz | 170MHz，需重配 |
> | GPIO 初始化 | `__HAL_RCC_GPIOx_CLK_ENABLE` 写法相同 | 同 |
> | SPI 实例 | SPI1/2 | G474 的 SPI 引脚映射不同，**必须用 CubeMX 重新分配** |
> | I²C 时序 | 软件延时基于 72MHz | **延时参数需按 170MHz 重算** |
>
> **建议做法**：用 CubeMX 新建 G474VE 工程、配好 SPI/I²C 与时钟，
> 再把 `SEEKFREE_ICM20602.c` 的**寄存器操作部分**（与 MCU 无关）搬过去，
> 底层读写函数按 G474 的 HAL 重写。
>
> 注意 [docs/03](../../docs/03-电控与驱动系统.md) 已指出：
> **IMU 应挂在 SPI2**（SPI1 的 PA6/PA7 与编码器 TIM3 冲突）。

## 关键实现要点

### 1. 速度环必须闭环

3S 电池从 12.6V 放到 9.0V，同一 PWM 下轮速变化约 30%。
**开环 PWM 会让里程计和推程全部不准。**

### 2. 堵转是"任务信息"不只是故障

推棋子时顶着墙，电机本来就该堵转。正确语义：

```
堵转 → 降 PWM 到维持值（约 30%）→ 通知决策层"已到位"或"受阻"
```

不要简单停机。

### 3. 引脚冲突

原设计 IMU 在 SPI1（PA5/6/7），与 TIM3 编码器（PA6/7）冲突，**已改到 SPI2**。
见 [hardware/electrical/接口定义.md](../../hardware/electrical/接口定义.md) §2.3。

### 4. UART 用 DMA + 空闲中断

921600 下逐字节中断开销过大。

### 5. 串口解析器用"缓冲区 + 重扫描"

**不要用朴素状态机**（失败就重置）—— 会被噪声中的假帧头吃掉真帧。
详见 [docs/11](../../docs/11-通信协议.md) §6。

### 6. K230 失联必须降级，不能停

心跳超时 200ms → 进降级策略 B。**视觉失效不能让整车瘫痪。**

## 调试接口（有线，禁无线）

| 接口 | 用途 |
|---|---|
| USART1 (PA14/PA15) | 调试串口，接 USB-TTL |
| SWD | 下载 + 实时变量观察 |

⚠️ **不要用带蓝牙/WiFi 的模块做"临时调试"** —— C11 禁止遥控，
即使不用，模块的存在也可能被判定为具备遥控能力。
