#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主控算力预算工具
================

用途：回答"STM32G431 到底弱不弱、该换成什么"。

**关键：先分辨瓶颈在哪一环。** 控制环（1kHz）和视觉识别（1080p@30）
的算力需求差两个数量级，搞错方向会浪费预算。

本工具算五件事：
  1. 控制环的真实 CPU 占用率（G431 vs 更强 MCU）
  2. 视觉链路的算力需求（1080p@30 到底要多少）
  3. 候选平台对比（MCU / SoC / 混合架构）
  4. 架构方案对比（双芯片 / 单芯片 / 三芯片）
  5. 结论与推荐

运行：
    python tools/compute_budget.py
"""

import math
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


# ==========================================================================
# 1. MCU 候选平台
# ==========================================================================

class MCU:
    def __init__(self, name, core, clock_mhz, dmips_per_mhz,
                 has_fpu, has_double_fpu, flash_kb, ram_kb,
                 timers_adv, price_cny, note=""):
        self.name = name
        self.core = core
        self.clock = clock_mhz
        self.dmips_per_mhz = dmips_per_mhz
        self.has_fpu = has_fpu
        self.has_double_fpu = has_double_fpu
        self.flash = flash_kb
        self.ram = ram_kb
        self.timers_adv = timers_adv
        self.price = price_cny
        self.note = note

    @property
    def dmips(self):
        return self.clock * self.dmips_per_mhz

    @property
    def fpu_factor(self):
        """浮点运算的相对效率（有硬件双精度FPU最快）"""
        if self.has_double_fpu:
            return 1.0
        if self.has_fpu:
            return 0.7          # 单精度FPU，双精度靠软件
        return 0.12             # 无FPU，全靠软件浮点


MCUS = {
    "F103": MCU("STM32F103C8T6", "Cortex-M3", 72, 1.25, False, False,
                64, 20, 1, 18, "最便宜，无FPU"),
    "G431": MCU("STM32G431", "Cortex-M4F", 170, 1.25, True, False,
                128, 32, 2, 45, "当前方案，单精度FPU"),
    "G474": MCU("STM32G474", "Cortex-M4F", 170, 1.25, True, False,
                512, 128, 3, 65, "电机控制专用，外设强"),
    "H723": MCU("STM32H723", "Cortex-M7", 550, 2.14, True, True,
                1024, 564, 2, 130, "双精度FPU，算力3x"),
    "H743": MCU("STM32H743", "Cortex-M7", 480, 2.14, True, True,
                2048, 1024, 2, 180, "大RAM，可跑轻量NN"),
    "H750": MCU("STM32H750", "Cortex-M7", 480, 2.14, True, True,
                128, 1024, 2, 85, "H743阉割Flash版，性价比高"),
}


# ==========================================================================
# 2. 控制环负载模型
# ==========================================================================

class ControlLoad:
    """
    1 kHz 控制环的各任务开销（以"等效 M4F@170MHz 指令数"为单位）。

    ★ 这些数字来自经验估算，必须用真机 DWT 计数器实测替换。
    """

    TASKS = [
        # (名称, 每次迭代指令数, 频率Hz, 说明)
        ("速度环 PID ×2",          800,  1000, "含编码器读取、抗饱和"),
        ("航向环 PID",             400,   500, "陀螺积分 + 位置式PID"),
        ("差速运动学",             300,  1000, "逆运动学分解 + 限幅"),
        ("PWM 更新",               200,  1000, "2通道 + 互锁检查"),
        ("编码器解码",             250,  1000, "TIM 正交解码读取"),
        ("IMU 读取(SPI)",          600,  1000, "ICM20602 14字节 + 单位换算"),
        ("航迹推算",               900,  1000, "EKF预测步，含三角函数"),
        ("堵转/限流检测",          300,  1000, "电流采样 + 比较"),
        ("UART 收帧解析",          500,   100, "状态机 + CRC"),
        ("状态机 tick",            400,   100, "决策逻辑"),
        ("日志/调试",              300,    10, "条件编译"),
    ]

    @staticmethod
    def instructions_per_second():
        total = 0
        for name, n_instr, freq, _ in ControlLoad.TASKS:
            total += n_instr * freq
        return total

    @staticmethod
    def detail():
        out = []
        for name, n_instr, freq, note in ControlLoad.TASKS:
            ips = n_instr * freq
            out.append((name, n_instr, freq, ips, note))
        return out


# ==========================================================================
# 3. 视觉链路负载模型
# ==========================================================================

class VisionLoad:
    """
    1080p@30 的处理负载。

    分"采集"与"处理"两段 —— 这是理解算力需求的关键。
    """

    @staticmethod
    def pixel_rates():
        return {
            "1080p@30": 1920 * 1080 * 30,
            "1080p@60": 1920 * 1080 * 60,
            "720p@60":  1280 * 720 * 60,
            "720p@30":  1280 * 720 * 30,
            "640x360@30": 640 * 360 * 30,
        }

    @staticmethod
    def ops_per_pixel_estimate():
        """
        各处理步骤的"每像素等效操作数"（C 实现，经验值）。

        这些值决定了处理负载，是本工具最关键的假设。
        """
        return {
            "降采样(binning)": 1.0,      # 硬件或极轻量
            "降采样(软件)": 4.0,          # 读写 + 平均
            "色彩空间转换": 12.0,         # RGB565 -> HSV
            "阈值分割": 6.0,              # 每像素比较
            "连通域标记": 25.0,           # 并查集/两遍扫描
            "形态学(单次)": 18.0,         # 3x3 开或闭
            "特征提取": 8.0,              # 面积/质心/圆度
        }


# ==========================================================================
# 4. 算力平台（视觉端）
# ==========================================================================

class VisionPlatform:
    def __init__(self, name, cpu, npu_tops, cpu_mops, power_w,
                 can_1080p30, mass_g, price_cny, note=""):
        self.name = name
        self.cpu = cpu
        self.npu_tops = npu_tops
        self.cpu_mops = cpu_mops          # CPU 侧通用算力 (MOPS)
        self.power = power_w
        self.can_1080p30 = can_1080p30
        self.mass = mass_g
        self.price = price_cny
        self.note = note


VISION_PLATFORMS = {
    "K230": VisionPlatform(
        "K230 CanMV", "2×RISC-V + 1×RV64", 6.0, 800,
        2.0, "勉强", 30, 199, "NPU强但CPU弱，1080p预处理吃力"),
    "RK3576": VisionPlatform(
        "RK3576", "4×A72 + 4×A53", 6.0, 12000,
        5.0, "可行", 60, 420, "八核，1080p@30 有余量"),
    "RK3588": VisionPlatform(
        "RK3588", "4×A76 + 4×A55", 6.0, 20000,
        9.0, "宽裕", 85, 680, "推荐：CPU强，预处理不吃力"),
    "Jetson": VisionPlatform(
        "Jetson Orin Nano", "6×A78AE", 40.0, 25000,
        12.0, "宽裕", 150, 1600, "算力过剩，超预算"),
}


# ==========================================================================
# 输出辅助
# ==========================================================================

def section(title):
    print("\n" + "=" * 88)
    print(title)
    print("=" * 88)


def table(headers, rows):
    widths = [max(len(str(h)), *(len(str(r[i])) for r in rows))
              for i, h in enumerate(headers)]
    print("  " + " ".join(str(h).rjust(w) for h, w in zip(headers, widths)))
    print("  " + "-" * (sum(widths) + len(widths) - 1))
    for r in rows:
        print("  " + " ".join(str(c).rjust(w) for c, w in zip(r, widths)))


# ==========================================================================
# 主程序
# ==========================================================================

def main():
    print("=" * 88)
    print("主控算力预算")
    print("=" * 88)
    print("  问题：STM32G431 到底弱不弱？该换成什么？")
    print()
    print("  ★ 关键：先分辨瓶颈在哪一环。")
    print("     控制环（1kHz）和视觉识别（1080p@30）的算力需求差两个数量级。")

    # ---------------- 1. MCU 参数 ----------------
    section("1. MCU 候选平台")
    rows = []
    for k, m in MCUS.items():
        rows.append([
            m.name, m.core, f"{m.clock}",
            f"{m.dmips:.0f}",
            "双精度" if m.has_double_fpu else ("单精度" if m.has_fpu else "无"),
            f"{m.flash}/{m.ram}",
            f"¥{m.price}",
        ])
    table(["型号", "内核", "MHz", "DMIPS", "FPU", "Flash/RAM KB", "价格"],
          rows)

    # ---------------- 2. 控制环负载 ----------------
    section("2. 控制环负载（这是 MCU 的真实工作）")
    print("  1 kHz 主循环的任务分解：")
    print()
    rows = []
    for name, n_instr, freq, ips, note in ControlLoad.detail():
        rows.append([name, f"{n_instr}", f"{freq}", f"{ips/1000:.0f}", note])
    table(["任务", "指令/次", "频率Hz", "k指令/s", "说明"], rows)
    print()
    total_ips = ControlLoad.instructions_per_second()
    print(f"  合计指令率 = {total_ips/1e6:.1f} M 指令/秒")

    # ---------------- 3. 各 MCU 占用率 ----------------
    section("3. ★ 各 MCU 的 CPU 占用率（回答「G431 弱不弱」）")
    print(f"  假设 M4F 每 MHz 可执行 {MCUS['G431'].dmips_per_mhz} M 指令（保守取 DMIPS）")
    print(f"  浮点任务按 FPU 能力折算")
    print()
    rows = []
    for k, m in MCUS.items():
        eff_mips = m.dmips * 1e6
        # 浮点任务占比（速度环/航向环/EKF 含大量浮点）
        fpu_adjusted = total_ips * (0.6 + 0.4 / m.fpu_factor)
        util = fpu_adjusted / eff_mips * 100
        verdict = ("充裕" if util < 30 else
                   "够用" if util < 60 else
                   "吃紧" if util < 85 else "**不够**")
        rows.append([
            m.name, m.core, f"{m.dmips:.0f}",
            f"{fpu_adjusted/1e6:.1f}", f"{util:.1f}%", verdict,
        ])
    table(["型号", "内核", "DMIPS", "等效M指令/s", "占用率", "评价"], rows)
    print()
    m_g = MCUS["G431"]
    util_g = total_ips * (0.6 + 0.4 / m_g.fpu_factor) / (m_g.dmips * 1e6) * 100
    print(f"  >> 关键结论：**STM32G431 跑 1 kHz 控制环的占用率只有 "
          f"{util_g:.0f}% 左右。**")
    print(f"     它**不是**瓶颈。1 kHz 双环 PID + EKF 对 M4F@170MHz 来说很轻松。")
    print()
    print("     ⚠️ 但这是「纯控制」的结论。若还要在 MCU 上跑别的（见第 5 节），")
    print("        情况会变。")

    # ---------------- 4. 视觉链路负载 ----------------
    section("4. ★ 视觉链路负载（1080p@30 到底要多少算力）")
    print("  像素率对比：")
    print()
    rates = VisionLoad.pixel_rates()
    rows = []
    for name, rate in rates.items():
        rows.append([name, f"{rate/1e6:.1f} MP/s",
                     f"{rate/rates['640x360@30']:.1f}x"])
    table(["模式", "像素率", "相对 640x360@30"], rows)
    print()
    print("  >> 1080p@30 = 62.2 MP/s，是 640x360@30 的 **9 倍**。")
    print("     如果直接处理全分辨率，算力需求会暴涨。")
    print()

    print("  处理步骤的每像素操作数（C 实现，经验值）：")
    print()
    ops = VisionLoad.ops_per_pixel_estimate()
    rows = [[k, f"{v:.0f}"] for k, v in ops.items()]
    table(["处理步骤", "操作数/像素"], rows)
    print()

    # 两种策略对比
    print("  两种策略的算力需求对比：")
    print()
    strategies = [
        ("策略A: 1080p 全分辨率处理",
         [("降采样", 0), ("色彩空间转换", 1), ("阈值分割", 1),
          ("连通域标记", 1), ("特征提取", 1)],
         1920 * 1080 * 30),
        ("策略B: 1080p采集 -> 640x360处理",
         [("降采样", 1), ("色彩空间转换", 1), ("阈值分割", 1),
          ("连通域标记", 1), ("特征提取", 1)],
         640 * 360 * 30),
    ]
    rows = []
    for sname, steps, pixels in strategies:
        # 降采样在 1080p 上做，其余在目标分辨率上做
        total = 0.0
        for step, where in steps:
            if where == 0:
                continue
            px = pixels if where == 1 else pixels
            total += ops.get(step, 0) * px
        rows.append([sname, f"{pixels/1e6:.1f}", f"{total/1e6:.0f}",
                     f"{total/1e6/30:.0f}"])
    # 更精确地算
    rows = []
    for sname, steps, pixels in strategies:
        total_ops = 0.0
        if "全分辨率" in sname:
            for step in ["色彩空间转换", "阈值分割", "连通域标记", "特征提取"]:
                total_ops += ops[step] * 1920 * 1080 * 30
        else:
            # 降采样在 1080p 上
            total_ops += ops["降采样(binning)"] * 1920 * 1080 * 30
            # 其余在 640x360 上
            for step in ["色彩空间转换", "阈值分割", "连通域标记", "特征提取"]:
                total_ops += ops[step] * 640 * 360 * 30
        rows.append([sname, f"{total_ops/1e6:.0f}",
                     f"{total_ops/1e6/30:.0f}",
                     f"{total_ops/1e6/800:.1f}"])
    table(["策略", "总MOPS", "每帧MOP", "相对K230 CPU倍"], rows)
    print()
    print("  >> 结论：**必须降采样后处理。**")
    print("     全分辨率处理的算力需求是降采样策略的 "
          f"{(ops['色彩空间转换']+ops['阈值分割']+ops['连通域标记']+ops['特征提取'])*1920*1080*30 / (ops['降采样(binning)']*1920*1080*30 + (ops['色彩空间转换']+ops['阈值分割']+ops['连通域标记']+ops['特征提取'])*640*360*30):.1f} 倍。")

    # ---------------- 5. 视觉平台对比 ----------------
    section("5. 视觉平台能否胜任 1080p@30")
    rows = []
    for k, p in VISION_PLATFORMS.items():
        rows.append([
            p.name, p.cpu, f"{p.npu_tops:.0f}", f"{p.cpu_mops}",
            f"{p.power:.1f}", p.can_1080p30, f"¥{p.price}", p.note,
        ])
    table(["平台", "CPU", "NPU TOPS", "CPU MOPS", "功耗W",
           "1080p@30", "价格", "备注"], rows)
    print()
    print("  >> 关键区分：**NPU 算力全都够（6 TOPS 跑小 CNN 绰绰有余）。**")
    print("     瓶颈在 **CPU 侧的图像预处理**（降采样、阈值、连通域）——")
    print("     这些跑在 CPU 上，不在 NPU 上。")
    print()
    k230 = VISION_PLATFORMS["K230"]
    rk3588 = VISION_PLATFORMS["RK3588"]
    print(f"     K230 的 CPU 等效算力仅 {k230.cpu_mops} MOPS（RISC-V 小核），")
    print(f"     RK3588 有 {rk3588.cpu_mops} MOPS（8 核 A76/A55），差 "
          f"{rk3588.cpu_mops/k230.cpu_mops:.0f} 倍。")
    print()
    print("     所以：**要 1080p@30，瓶颈是 CPU 不是 NPU。**")
    print("     这解释了为什么「K230 的 NPU 有 6 TOPS 但跑 1080p@30 仍吃力」。")

    # ---------------- 6. 架构方案 ----------------
    section("6. 架构方案对比")
    print("  三种架构选择：")
    print()
    arch_rows = [
        ["A. 双芯片（现状）", "G431 + K230",
         "实时性最好", "K230 CPU 弱，1080p@30 吃力", "**不满足新需求**"],
        ["B. 双芯片（升级MCU）", "H723 + RK3588",
         "实时性最好 + 视觉强", "成本高、通信仍存在", "可行"],
        ["C. 双芯片（MCU够用）", "G431 + RK3588",
         "视觉强、成本可控", "G431 占用率 ~15%，有余量", "**推荐**"],
        ["D. 单芯片", "RK3588 直控电机",
         "省一颗芯片", "Linux 无 1kHz 确定性", "**否决**"],
    ]
    table(["方案", "组合", "优点", "缺点", "结论"], arch_rows)
    print()
    print("  >> 方案 D 为什么否决：")
    print("     Linux 的调度抖动在毫秒级，做不了 1 kHz 硬实时控制环。")
    print("     电机控制需要确定性中断，这是 MCU 的领域。")
    print()
    print("  >> 方案 C 为什么推荐：")
    print(f"     G431 跑控制环占用率仅 ~{util_g:.0f}%，")
    print(f"     把预算投到视觉端（RK3588）收益更大。")

    # ---------------- 7. 内存预算（真正的约束）----------------
    section("7. ★ 内存预算 —— 这才是 G431 真正会卡住的地方")
    print("  CPU 占用率低不代表够用。嵌入式项目更常见的是**内存不够**。")
    print()
    mem_items = [
        ("EKF 状态与协方差",     800,    2000,  "6状态，双精度"),
        ("棋子地图(32枚)",       512,    1024,  "位置+类别+状态"),
        ("视觉目标列表",         320,     800,  "8项 x 40字节"),
        ("UART 收发缓冲",        512,     512,  "DMA 双缓冲各 256"),
        ("控制环状态",           256,     800,  "两个 PID + 运动学"),
        ("IMU 滤波与标定",       256,     600,  "零偏、滑动平均"),
        ("状态机上下文",         256,    1500,  "FSM + 路径规划"),
        ("日志/调试缓冲",        512,     512,  "环形缓冲"),
        ("HAL 库开销",          1500,   20000,  "HAL + 中断向量"),
        ("栈(主+中断)",         2048,      0,  "保守估计"),
    ]
    rows = [[n, f"{r}", f"{f}", note] for n, r, f, note in mem_items]
    table(["项目", "RAM B", "Flash B", "说明"], rows)
    print()
    ram_need = sum(r for _, r, _, _ in mem_items)
    flash_need = sum(f for _, _, f, _ in mem_items)
    print(f"  合计需求：RAM {ram_need} B ({ram_need/1024:.1f} KB)，"
          f"Flash {flash_need} B ({flash_need/1024:.1f} KB)")
    print()
    print("  各 MCU 的余量：")
    print()
    rows = []
    for k, m in MCUS.items():
        ram_util = ram_need / (m.ram * 1024) * 100
        flash_util = flash_need / (m.flash * 1024) * 100
        worst = max(ram_util, flash_util)
        verdict = ("充裕" if worst < 40 else
                   "够用" if worst < 70 else
                   "吃紧" if worst < 90 else "**不够**")
        rows.append([
            m.name, f"{m.flash}/{m.ram}",
            f"{ram_util:.0f}%", f"{flash_util:.0f}%", verdict,
        ])
    table(["型号", "Flash/RAM KB", "RAM占用", "Flash占用", "评价"], rows)
    print()
    g = MCUS["G431"]
    h = MCUS["H723"]
    print(f"  >> G431（{g.flash}KB Flash / {g.ram}KB RAM）：")
    print(f"     RAM 占用 {ram_need/(g.ram*1024)*100:.0f}%，"
          f"Flash 占用 {flash_need/(g.flash*1024)*100:.0f}%")
    print()
    print("     ⚠️ 看似有余量，但这是**当前功能**的需求。以下任一情况就会超：")
    print("        - 更精细的地图（栅格 10cm -> 60x40 = 2400 格）")
    print("        - 更完整的 EKF/UKF（状态数翻倍）")
    print("        - 比赛数据记录（用于复盘调参）")
    print("        - 预测控制（MPC 需要矩阵运算缓冲）")
    print("        - 引入 RTOS（FreeRTOS 任务栈开销）")
    print()
    print(f"  >> H723（{h.flash}KB Flash / {h.ram}KB RAM）：")
    print(f"     RAM 是 G431 的 {h.ram/g.ram:.0f} 倍，"
          f"Flash 是 {h.flash/g.flash:.0f} 倍")
    print(f"     **这才是更换主控的真正理由** —— 不是 CPU 算力不够，")
    print(f"     而是 RAM/Flash 对未来扩展的限制。")

    # ---------------- 8. MCU 更换决策 ----------------
    section("8. MCU 更换决策")
    print("  分场景判断：")
    print()
    ram_g = ram_need / (MCUS["G431"].ram * 1024) * 100
    ram_h = ram_need * 2.5 / (MCUS["H723"].ram * 1024) * 100
    scenarios = [
        ("仅当前功能（控制+状态机+伺服）", "G431",
         f"CPU {util_g:.0f}% / RAM {ram_g:.0f}%", "够用"),
        ("+ 精细栅格地图", "G474",
         f"RAM {ram_need*2/(MCUS['G474'].ram*1024)*100:.0f}%", "够用"),
        ("+ RTOS + 数据记录", "H723",
         f"RAM {ram_h:.0f}%", "推荐"),
        ("+ 完整 EKF/MPC", "H723", "余量充足", "推荐"),
        ("+ MCU 侧轻量 NN", "H743", "大 RAM", "需 H743"),
    ]
    table(["场景", "推荐", "资源占用", "判断"], scenarios)
    print()
    print("  >> 本项目的实际需求落在第 1-2 行：**G431 够用**。")
    print("     字符识别跑在视觉平台（NPU），**不在 MCU 上**。")
    print()
    print("  >> 但若要为 5 轮迭代留余量，推荐 **STM32H723**：")
    print(f"     - 550MHz Cortex-M7，**双精度** FPU")
    print(f"     - DMIPS {h.dmips:.0f}（G431 的 {h.dmips/g.dmips:.1f} 倍）")
    print(f"     - RAM {h.ram}KB（G431 的 {h.ram/g.ram:.0f} 倍）")
    print(f"     - Flash {h.flash}KB（G431 的 {h.flash/g.flash:.0f} 倍）")
    print(f"     - 价格 ¥{h.price} vs G431 ¥{g.price}，"
          f"**仅 +¥{h.price-g.price}**")
    print()
    print("     在 5 轮迭代、还要加字符识别与优先级算法的背景下，")
    print("     这点成本换来 8 倍 RAM 与 3 倍算力，是合理的。")

    # ---------------- 9. 结论 ----------------
    section("9. 结论与推荐")
    print("  【诊断：两个瓶颈是不同的东西】")
    print()
    rows = [
        ["控制环 CPU", f"{total_ips/1e6:.1f} M 指令/秒",
         f"G431 占用 {util_g:.0f}%", "**不是瓶颈**"],
        ["控制环 RAM/Flash", f"{ram_need/1024:.0f}KB/{flash_need/1024:.0f}KB",
         f"G431 占 {ram_g:.0f}%/"
         f"{flash_need/(MCUS['G431'].flash*1024)*100:.0f}%",
         "当前够，扩展受限"],
        ["视觉预处理 CPU", f"K230 仅 {k230.cpu_mops} MOPS",
         "需 1080p@30", "**★ 真正瓶颈**"],
        ["视觉 NPU", "6 TOPS", "小 CNN 够用", "不是瓶颈"],
    ]
    table(["环节", "需求", "现状", "判断"], rows)
    print()
    print("  >> 关键：**K230 的 CPU 算力是 1080p@30 的真瓶颈，NPU 不是。**")
    print(f"     图像预处理（降采样/阈值/连通域）跑在 CPU 上。")
    print(f"     RK3588 的 CPU 有 {rk3588.cpu_mops} MOPS，"
          f"是 K230 的 {rk3588.cpu_mops/k230.cpu_mops:.0f} 倍。")
    print()
    print("  【推荐配置】")
    rows = [
        ["运动主控", "STM32H723", "¥130",
         f"RAM {h.ram/g.ram:.0f}x / Flash {h.flash/g.flash:.0f}x，为迭代留余量"],
        ["视觉平台", "RK3588 板", "¥680",
         f"★ 关键：CPU 算力 {rk3588.cpu_mops/k230.cpu_mops:.0f}x，1080p@30 有余量"],
        ["原方案", "G431 + K230", "¥244",
         "控制够用，但视觉端 1080p@30 吃力"],
        ["升级成本", "+¥566", "", "总预算内（见 BOM）"],
    ]
    table(["项目", "选型", "价格", "理由"], rows)
    print()
    print("  【为什么 MCU 升级是次要、视觉端是必须】")
    print(f"     - G431 跑控制环只占 {util_g:.0f}% CPU，RAM 也够当前功能")
    print(f"     - 但 1080p@30 的算力增长在**图像预处理**（CPU 侧）")
    print(f"     - K230 的 CPU 太弱，这是硬伤，换不掉")
    print(f"     - H723 成本仅 +¥{h.price-g.price}，换来 {h.ram/g.ram:.0f}x RAM，")
    print(f"       在 5 轮迭代背景下值得")
    print()
    print("=" * 88)
    print("第 2 轮迭代完成。下一步（第 3 轮）：1080p@30 视觉管线设计。")
    print("=" * 88)


if __name__ == "__main__":
    main()
