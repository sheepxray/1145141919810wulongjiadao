# ICM20602 逐飞(SEEKFREE)例程移植参考

来源：商家资料包 `ICM20602六轴模块.zip` → `ICM20602例程/STM32F103VETx.zip`（逐飞开源库）。
从 STM32F103 HAL 例程中提取核心驱动，供本项目 STM32G474VET6 移植参考。

| 子目录 | 接口 | 说明 |
|---|---|---|
| `hardware_spi/` | **硬件 SPI** | 对应本项目 SPI2 方案，**首选参考** |
| `software_spi/` | 软件 SPI（GPIO 模拟） | 引脚紧张时的备选，可任意引脚 |
| `software_iic/` | 软件 I²C | 接口最省(3 线)，文档 docs/03 提到的退路 |

每个目录内含：
- `SEEKFREE_ICM20602.c / .h`：平台无关的 ICM20602 驱动（核心，逐飞开源协议）
- `SEEKFREE_IIC.c / .h`（仅 software_iic）：软件 I²C 底层
- `main.c / main.h`：示例用法（GPIO/SPI 初始化 + 读陀螺/加速度）
- `_proj/*.ioc`：原 STM32F103 CubeMX 工程配置，可对照引脚/外设设置

## 移植到 STM32G474VET6 要点

1. **外设映射**：SPI2 = PB13(SCK)/PB15(MOSI)/PB14(MISO) + PB12(软件 CS)，与 docs/03 §4.2.1 一致。
2. **只改底层**：`SEEKFREE_ICM20602.c` 里读写单字节/多字节的底层函数改为主板 SPI2 HAL 调用即可，寄存器逻辑不用动。
3. **SPI 时序**：≤10 MHz，Mode 0 或 3，MSB first，8-bit；首字节 MSB = R/W + 低 7 位地址。
4. **CS**：由 MCU GPIO 控制，上电拉低为 SPI 模式，不可悬空。
5. **HAL 版本差异**：F103(G4 同代 HAL 结构差异小) 例程直接用即可，注意 `stm32f1xx_hal_conf.h` 仅参考、不可照搬。
6. 例程默认采集后串口/屏显搬运，本项目落地时改为 **1 kHz 定时器采样 + 航向积分**（见 docs/03 §4.3）。