#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
质量与预算校核工具
==================

赛题硬约束：
    发车质量 <= 1.5 kg
    发车尺寸 <= 250 x 200 x 180 mm

用法：
    python tools/mass_budget.py

新增器件时先在 COMPONENTS 里加一行，再跑一次脚本确认没超限。
设计目标是留 20% 余量（见下 TARGET_UTILIZATION）。

注意：这里列的是**预估值**，采购后必须用厨房秤实测回填 MEASURED 字段。
"""

import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


# --------------------------------------------------------------------------
# 赛题限制
# --------------------------------------------------------------------------
LIMIT_MASS_G = 1500.0
LIMIT_DIM_MM = (250.0, 200.0, 180.0)
LIMIT_BUDGET_CNY = 2000.0

# 设计目标：不使用超过 80% 的质量上限，给后续加装留余地
TARGET_UTILIZATION = 0.80


# --------------------------------------------------------------------------
# 物料表
#   (名称, 单件质量g, 数量, 单价元, 分类)
# --------------------------------------------------------------------------
COMPONENTS = [
    # ---- 动力与驱动 ----
    ("JGB37-520 12V 带霍尔编码器电机", 95.0, 2, 68.0, "动力"),
    ("橡胶轮 65mm（含联轴器）",          35.0, 2, 18.0, "动力"),
    ("BTS7960 / IBT-2 驱动模块",          15.0, 2, 12.0, "动力"),

    # ---- 控制 ----
    ("STM32G431 核心板",                  15.0, 1, 45.0, "控制"),
    ("ICM20602 IMU 模块",                  5.0, 1, 25.0, "控制"),
    ("K230 视觉模块（含底板）",           30.0, 1, 199.0, "视觉"),

    # ---- 视觉 ----
    ("OV5640 摄像头模块（前视）",          10.0, 1, 38.0, "视觉"),
    ("OV5640 摄像头模块（下视）",          10.0, 1, 38.0, "视觉"),
    ("CSI 排线 / 延长线",                   6.0, 2, 8.0, "视觉"),

    # ---- 电源 ----
    ("3S LiPo 1500mAh 电池",             130.0, 1, 85.0, "电源"),
    ("降压模块 12V->5V 5A",               12.0, 2, 15.0, "电源"),
    ("降压模块 5V->3.3V 1A",               6.0, 1, 8.0, "电源"),
    ("电解电容/电感/滤波件",              15.0, 1, 15.0, "电源"),
    ("XT60 插头 / 开关 / 保险丝",         20.0, 1, 20.0, "电源"),
    ("LiPo 低压报警器",                    8.0, 1, 10.0, "电源"),

    # ---- 结构 ----
    ("3D 打印件（底板+支架+推板，PETG）", 220.0, 1, 0.0, "结构"),
    ("万向轮 / 支撑球",                   25.0, 1, 12.0, "结构"),
    ("推板柔性贴地组件（弹簧+PTFE）",     30.0, 1, 15.0, "结构"),
    ("螺丝/铜螺母/扎带/线材",            110.0, 1, 30.0, "结构"),

    # ---- 合规 ----
    ("650nm 1mW 一字线激光模组",           8.0, 1, 12.0, "合规"),
    ("激光限流电阻/安装座",                7.0, 1, 5.0, "合规"),
    ("自检 LED + 蜂鸣器 + 拨码开关",      12.0, 1, 8.0, "合规"),
]

# 采购后实测质量（g），键为器件名。未测量的留空。
MEASURED = {
    # "3S LiPo 1500mAh 电池": 138.0,
}


def main():
    total_g = 0.0
    total_cny = 0.0
    by_cat = {}

    print("=" * 86)
    print("质量与预算校核")
    print("=" * 86)
    print()
    print(f"{'器件':<38} {'单件g':>7} {'数量':>5} {'小计g':>8} {'单价':>7} {'小计元':>8}")
    print("-" * 86)

    for name, unit_g, qty, unit_cny, cat in COMPONENTS:
        measured = MEASURED.get(name, unit_g)
        sub_g = measured * qty
        sub_cny = unit_cny * qty
        total_g += sub_g
        total_cny += sub_cny
        by_cat[cat] = by_cat.get(cat, [0.0, 0.0])
        by_cat[cat][0] += sub_g
        by_cat[cat][1] += sub_cny
        mark = "*" if name in MEASURED else " "
        print(f"{name:<38}{mark}{measured:>6.1f} {qty:>5} {sub_g:>8.1f} "
              f"{unit_cny:>7.1f} {sub_cny:>8.1f}")

    print("-" * 86)
    print(f"{'合计':<38} {'':>7} {'':>5} {total_g:>8.1f} {'':>7} {total_cny:>8.1f}")
    print("  (* = 已实测回填，其余为预估)")

    # ---------------- 分类汇总 ----------------
    print()
    print("按类别汇总：")
    print(f"  {'类别':<12} {'质量g':>9} {'占比':>8} {'成本元':>10} {'占比':>8}")
    print("  " + "-" * 50)
    for cat, (g, cny) in sorted(by_cat.items(), key=lambda kv: -kv[1][0]):
        print(f"  {cat:<12} {g:>9.1f} {g/total_g*100:>7.1f}% "
              f"{cny:>10.1f} {cny/total_cny*100:>7.1f}%")

    # ---------------- 质量校核 ----------------
    print()
    print("=" * 86)
    print("质量校核")
    print("=" * 86)
    util = total_g / LIMIT_MASS_G
    print(f"  预估总质量     : {total_g:.1f} g")
    print(f"  赛题上限       : {LIMIT_MASS_G:.1f} g")
    print(f"  余量           : {LIMIT_MASS_G - total_g:.1f} g")
    print(f"  利用率         : {util*100:.1f}%")
    print(f"  设计目标       : <= {TARGET_UTILIZATION*100:.0f}%")

    if total_g > LIMIT_MASS_G:
        print("\n  [超限] 超过赛题上限！必须减重。")
        print("  建议优先检查：电池容量、打印件填充率、结构件是否可镂空。")
    elif util > TARGET_UTILIZATION:
        print(f"\n  [偏重] 未超限但超出设计目标，建议减重 "
              f"{total_g - LIMIT_MASS_G*TARGET_UTILIZATION:.1f} g 以留出余量。")
    else:
        print("\n  [通过] 质量在目标范围内。")

    # 用厨房秤实测后应重新校核
    unmeasured = [n for n, *_ in COMPONENTS if n not in MEASURED]
    if unmeasured:
        print(f"\n  提示：还有 {len(unmeasured)} 项未实测。采购后请用厨房秤逐一称重，")
        print(f"        填入 MEASURED 字典后重跑本脚本。")

    # ---------------- 预算校核 ----------------
    print()
    print("=" * 86)
    print("预算校核")
    print("=" * 86)
    print(f"  预估总价   : ¥{total_cny:.1f}")
    print(f"  预算       : ¥{LIMIT_BUDGET_CNY:.1f}")
    print(f"  余量       : ¥{LIMIT_BUDGET_CNY - total_cny:.1f}")
    if total_cny > LIMIT_BUDGET_CNY:
        print("\n  [超预算] 需要削减。")
    else:
        print("\n  [通过] 预算内。余量建议用于：备件电机1个、备用相机1个、"
              "额外电池1块。")
    print("\n  注意：以上为预估价，实际采购前需实时核价。")

    # ---------------- 体积校核 ----------------
    print()
    print("=" * 86)
    print("尺度校核（发车状态外廓）")
    print("=" * 86)
    # 设计外廓
    design = (225.0, 190.0, 170.0)
    names = ("长", "宽", "高")
    print(f"  {'方向':<6} {'设计(mm)':>10} {'上限(mm)':>10} {'余量(mm)':>10} {'状态':>8}")
    print("  " + "-" * 50)
    for i, (dv, lm) in enumerate(zip(design, LIMIT_DIM_MM)):
        margin = lm - dv
        ok = "OK" if margin > 0 else "超限"
        print(f"  {names[i]:<6} {dv:>10.0f} {lm:>10.0f} {margin:>10.0f} {ok:>8}")

    print()
    print("  提示：'宽'是最关键的一维 —— 它同时决定车体半宽（影响贴墙容错）")
    print("        和推板开口宽度。应尽量用满 200mm 上限。")
    print()
    print("=" * 86)


if __name__ == "__main__":
    main()
