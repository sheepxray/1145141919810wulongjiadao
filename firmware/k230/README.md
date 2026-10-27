# K230 视觉端骨架

> 对应文档：[docs/04-视觉识别系统](../../docs/04-视觉识别系统.md)、
> [docs/11-通信协议](../../docs/11-通信协议.md)

## 目录结构

```
firmware/k230/
  ├─ main.py                主循环 + 模式切换
  ├─ vision/
  │    ├─ color.py          颜色阈值 + 连通域（找棋子）
  │    ├─ wall.py           挡板边缘检测 + 测距
  │    └─ servo.py          推入段视觉伺服
  ├─ nn/
  │    ├─ classify.py       kmodel 推理（兵种分类）
  │    └─ preprocess.py     裁切 / 归一化
  └─ comms/
       └─ stm32_link.py      串口协议
```

## 可直接复用的代码

| 文件 | 说明 |
|---|---|
| [`tools/protocol.py`](../../tools/protocol.py) | **完整的协议实现**，K230 侧可直接 `import` |
| [`tools/geometry.py`](../../tools/geometry.py) | 相机几何计算（测距反解、可见区间） |

`tools/protocol.py` 是可直接使用的实现（编码 + 流式解析），
已通过 CRC 已知向量、round-trip、粘包/半包/噪声等自测：

```bash
python tools/protocol.py
```

## 关键设计要点

### 1. 分阶段识别（不要一上来就 CNN）

```
第一级（移动中，前视相机）：颜色阈值 → 圆形筛选 → 候选 + 红/黑
    ↓ 只保留己方颜色（32 → 16 枚）
第二级（停车时，下视相机）：小 CNN → 兵种（7 类）
```

**颜色是免费的高价值先验** —— 红方字红、黑方字黑，颜色阈值即可分，
不需要 CNN、不需要停车。这让需要读字的候选**立刻减半**。

### 2. 相机安装角度不能错

| 相机 | 俯角 | 原因 |
|---|---|---|
| 前视 | **15°** | 兼顾近场测墙与地面棋子 |
| 下视 | **60°** | ⚠️ **不是 90°** |

**下视必须前倾 60°**：垂直下视时可见地面区间只有 0–76mm，
**完全看不到推板前方 100–250mm 处的棋子**。详见 [docs/02](../../docs/02-机械系统设计.md) §5。

### 3. 读字必须停车

**绝不在移动中读汉字** —— 透视压缩 + 卷帘错位 + 运动模糊三杀。

停车后卷帘错位和运动模糊**都是零**，所以不需要全局快门相机。

### 4. 测距用完整投影反解

**不要用 `d = H·f/Δy`** —— 它只在相机水平时精确，
俯角 30° / 相机高 250mm 时误差达 17.3%。用 `tools/geometry.py` 里的
`solve_range_from_pixels`（完整投影数值反解）。

### 5. 模式切换省电

不要在 IDLE 时也跑全部处理。用 `MODE_CMD` 切换 SEARCH / CLASSIFY / SERVO，
可显著降低功耗与发热 —— 对 1.5kg 车体和 K230 稳定性都有好处。

### 6. 启动/重启后必须发 HELLO

`HELLO` 让 STM32 重新下发 `STATE_INFO` 同步上下文。

## 开发顺序建议

⚠️ **第 1 周就打通"采集 → 训练 → nncase → 推理"的端到端链路。**

nncase 工具链是本项目最大的技术风险（[docs/09](../../docs/09-待确认事项与风险.md) R1）。
用一个空模型先跑通流程，**不要留到 M3 才发现转不过去**。

## 降级要求

K230 必须能回答一个问题：**"如果我完全不工作，车还能不能得分？"**

答案必须是"能" —— STM32 侧的降级策略 B 不依赖视觉识别分值。

## 水罐视觉接近

入口脚本：

```text
firmware/k230/vision_approach.py
```

运行前把 `water_can_detect.py` 与 `vision_approach.py` 两个文件都放到 K230
的 `/sdcard` 目录，`vision_approach.py` 会导入检测器模块。

K230 使用 **UART3**、`921600 8N1` 以 20Hz 发送 `APPROACH_TARGET`。
40Pin 排针使用 `Pin 37/GPIO32(TXD)` 与 `Pin 40/GPIO33(RXD)`；
该组合已在实机上完成 30 帧连续接收验证。

UART3 TX 必须显式配置 FPIOA 输出使能：

```python
fpioa.set_function(32, fpioa.UART3_TXD, ie=1, oe=1, pu=1, pd=0, ds=7)
fpioa.set_function(33, fpioa.UART3_RXD, ie=1, oe=0, pu=1, pd=0, ds=7)
```

STM32 负责视觉转向、前进、限速、到距离停车和 200ms 断链停车，K230 不
直接驱动电机。目标偏离画面中心时优先原地转向，回到中央附近后才前进；
达到停车尺寸后仍可原地转向保持目标居中。

首次调试顺序：

1. 车轮架空，先烧录包含视觉接近模式的 STM32 固件。
2. 先运行 `uart3_link_test.py`，不接摄像头；在 USART1 发送 `VI`，
   确认 `rx_bytes` 和 `frames` 持续增加。
3. 链路通过后，在 USART1 发送 `VA`，开启视觉接近。
4. 在 CanMV IDE 运行 `vision_approach.py`，再用 `VI` 检查
   `valid/err_pm/size_pm/conf/age_ms`。
5. 确认目标偏左/偏右时的转向方向正确，再把车放地面。
6. 测出安全停止距离下目标框高度，必要时修改
   `firmware/stm32/wulong_fw/Core/Src/main.c` 的
   `VISION_STOP_SIZE_PM`（默认 580）。

安全命令：

```text
VS   关闭视觉接近
S    停车并解除控制
E    急停
```

默认速度上限为前进 `100 mm/s`、转向 `350 mrad/s`。K230 侧对误差和尺寸
做低通平滑，并在短暂丢失时保留目标 250ms。第一次实车必须从架空状态验证
方向，并确保 `E` 急停随时可到达。

### UART2 备选

如果改用 40Pin 的 UART2：

```text
K230 Pin 11 / GPIO05 / UART2_TXD -> STM32 PA3
K230 Pin 13 / GPIO06 / UART2_RXD <- STM32 PA2
K230 Pin 14 / GND                <-> STM32 GND
```

UART2 引出在当前板卡上未通过实机连续帧验证，默认不使用。UART3 为当前
已验证方案。
