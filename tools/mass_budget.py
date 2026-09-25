#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
质量与预算校核工具（五轮迭代后的最终版）
========================================

赛题硬约束：
    发车质量 <= 1.5 kg
    发车尺寸 <= 250 x 200 x 180 mm

**本表反映五轮迭代后的最终选型**（见 docs/16-最终BOM与交付清单.md）：
  - 第1轮：电源（INA240 限流、5A 保险丝、15 00mAh 电池）
  - 第2轮：主控（H723 替代 G431、RK3588 替代 K230）
  - 第3轮：视觉（IMX219 替代 OV5640）
  - 第4轮：识别（新增，无硬件变化）
  - 第5轮：SketchUp 模型（尺寸校核）

用法：
    python tools/mass_budget.py
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
TARGET_UTILIZATION = 0.80


# --------------------------------------------------------------------------
# 物料表（★ 五轮迭代后的最终选型）
#   (名称, 单件质量g, 数量, 单价元, 分类, 轮次备注)
# --------------------------------------------------------------------------
COMPONENTS = [
    # ---- 动力与驱动（第1轮：新增电流采样）----
    ("JGB37-520 12V 带霍尔编码器", 95.0, 2, 68.0, "动力", "1"),
    ("橡胶轮 65mm（含联轴器）",     35.0, 2, 18.0, "动力", ""),
    ("BTS7960 / IBT-2 驱动模块",   15.0, 2, 12.0, "动力", ""),
    ("★ INA240 电流采样模块",       5.0, 2, 12.0, "动力", "1"),

    # ---- 控制（第2轮：G431 -> H723）----
    ("★ STM32H723 核心板",         20.0, 1, 130.0, "控制", "2"),
    ("ICM20602 IMU 模块",           5.0, 1, 25.0, "控制", ""),
    ("ST-Link V2 下载器",           0.0, 1, 0.0, "控制", "工具"),

    # ---- 视觉（第2/3轮：K230 -> RK3588，OV5640 -> IMX219）----
    ("★ RK3588 核心板",            85.0, 1, 680.0, "视觉", "2"),
    ("★ IMX219 相机模块（前视）",   12.0, 1, 65.0, "视觉", "3"),
    ("★ IMX219 相机模块（下视）",   12.0, 1, 65.0, "视觉", "3"),
    ("MIPI CSI 排线",               6.0, 2, 12.0, "视觉", ""),
    ("散热片 + 风扇",              15.0, 1, 20.0, "视觉", "RK3588 需散热"),

    # ---- 电源（第1轮：1500mAh、5A 保险丝）----
    ("3S LiPo 1500mAh 25C",       130.0, 1, 85.0, "电源", "1"),
    ("DC-DC 12V->5V 5A",           20.0, 1, 25.0, "电源", "RK3588 需 1.8A"),
    ("DC-DC 5V->3.3V 2A",           8.0, 1, 10.0, "电源", ""),
    ("电解电容/电感/磁珠/陶瓷",     25.0, 1, 18.0, "电源", "1"),
    ("XT60/开关/5A慢熔保险丝",      20.0, 1, 15.0, "电源", "1"),
    ("LiPo 低压报警器",             8.0, 1, 10.0, "电源", ""),

    # ---- 结构（第5轮：含 50mm 相机立柱）----
    ("3D 打印件（PETG）",         240.0, 1, 0.0, "结构", "5"),
    ("万向球 ×2",                  25.0, 2, 12.0, "结构", ""),
    ("推板柔性件（弹簧+PTFE）",     30.0, 1, 15.0, "结构", ""),
    ("螺丝/铜螺母/扎带/线材",      115.0, 1, 30.0, "结构", ""),

    # ---- 合规 ----
    ("650nm 1mW 激光模组",          8.0, 1, 12.0, "合规", ""),
    ("限流电阻 + 安装座",            7.0, 1, 5.0, "合规", ""),
    ("LED + 蜂鸣器 + 拨码开关",     12.0, 1, 8.0, "合规", ""),
]

# 采购后实测质量回填
MEASURED = {}


def main():
    total_g = 0.0
    total_cny = 0.0
    by_cat = {}

    print("=" * 94)
    print("质量与预算校核（五轮迭代最终版）")
    print("=" * 94)
    print()
    print(f"{'器件':<34}{'轮':>3}{'单件g':>8}{'数':>4}{'小计g':>8}"
          f"{'单价':>8}{'小计元':>9}")
    print("-" * 94)

    for name, unit_g, qty, unit_cny, cat, rnd in COMPONENTS:
        measured = MEASURED.get(name, unit_g)
        sub_g = measured * qty
        sub_cny = unit_cny * qty
        total_g += sub_g
        total_cny += sub_cny
        by_cat.setdefault(cat, [0.0, 0.0])
        by_cat[cat][0] += sub_g
        by_cat[cat][1] += sub_cny
        mark = "*" if name in MEASURED else " "
        print(f"{name:<34}{rnd:>3}{mark}{measured:>7.1f}{qty:>4}{sub_g:>8.1f}"
              f"{unit_cny:>8.1f}{sub_cny:>9.1f}")

    print("-" * 94)
    print(f"{'合计':<34}{'':>3}{'':>8}{'':>4}{total_g:>8.1f}"
          f"{'':>8}{total_cny:>9.1f}")
    print("  (轮 = 引入该选型的迭代轮次；* = 已实测回填)")

    # ---------------- 分类汇总 ----------------
    print()
    print("按类别汇总：")
    print(f"  {'类别':<10}{'质量g':>9}{'占比':>8}{'成本元':>10}{'占比':>8}")
    print("  " + "-" * 48)
    for cat, (g, cny) in sorted(by_cat.items(), key=lambda kv: -kv[1][0]):
        print(f"  {cat:<10}{g:>9.1f}{g/total_g*100:>7.1f}%"
              f"{cny:>10.1f}{cny/total_cny*100:>7.1f}%")

    # ---------------- 质量校核 ----------------
    print()
    print("=" * 94)
    print("质量校核")
    print("=" * 94)
    util = total_g / LIMIT_MASS_G
    print(f"  预估总质量 : {total_g:.1f} g")
    print(f"  赛题上限   : {LIMIT_MASS_G:.1f} g")
    print(f"  余量       : {LIMIT_MASS_G - total_g:.1f} g")
    print(f"  利用率     : {util*100:.1f}%   (目标 <= {TARGET_UTILIZATION*100:.0f}%)")

    if total_g > LIMIT_MASS_G:
        print("\n  [超限] 必须减重。")
    elif util > TARGET_UTILIZATION:
        print(f"\n  [偏重] 超出设计目标，建议减重 "
              f"{total_g - LIMIT_MASS_G*TARGET_UTILIZATION:.1f} g。")
    else:
        print("\n  [通过] 在目标范围内。")

    unmeasured = [n for n, *_ in COMPONENTS if n not in MEASURED]
    if unmeasured:
        print(f"\n  提示：{len(unmeasured)} 项未实测，采购后用厨房秤称重并"
              f"回填 MEASURED 字典。")

    # ---------------- 预算校核 ----------------
    print()
    print("=" * 94)
    print("预算校核")
    print("=" * 94)
    print(f"  预估总价 : ¥{total_cny:.1f}")
    print(f"  预算     : ¥{LIMIT_BUDGET_CNY:.1f}")
    print(f"  余量     : ¥{LIMIT_BUDGET_CNY - total_cny:.1f}")
    if total_cny > LIMIT_BUDGET_CNY:
        print("\n  [超预算] 需要削减。")
        print("  最大的两项是 RK3588（¥680）与 H723（¥130），"
              "若需削减可考虑降级视觉平台。")
    else:
        print("\n  [通过] 预算内。")

    # ---------------- 尺度校核 ----------------
    print()
    print("=" * 94)
    print("尺度校核（发车状态外廓，与 tools/sketchup_model.py 一致）")
    print("=" * 94)
    design = (223.0, 195.0, 160.0)
    names = ("长", "宽", "高")
    notes = ("含推板弧形段", "★ 推板最宽处", "含 50mm 相机立柱")
    print(f"  {'方向':<6}{'设计mm':>10}{'上限mm':>10}{'余量mm':>10}{'状态':>8}  说明")
    print("  " + "-" * 68)
    for i, (dv, lm) in enumerate(zip(design, LIMIT_DIM_MM)):
        margin = lm - dv
        ok = "OK" if margin > 0 else "超限"
        print(f"  {names[i]:<6}{dv:>10.0f}{lm:>10.0f}{margin:>10.0f}"
              f"{ok:>8}  {notes[i]}")
    print()
    print("  ⚠️ 宽度余量仅 5mm —— 这是最紧的一维，装配时优先保证。")

    # ---------------- 与上一版的差异 ----------------
    print()
    print("=" * 94)
    print("五轮迭代带来的变化（vs 初版方案）")
    print("=" * 94)
    rows = [
        ("主控", "STM32G431 ¥45", "STM32H723 ¥130", "+¥85",
         "RAM 18x，为迭代留余量"),
        ("视觉平台", "K230 ¥199", "RK3588 ¥680", "+¥481",
         "★ CPU 算力 25x，1080p@30 关键"),
        ("相机 ×2", "OV5640 ¥38", "IMX219 ¥65", "+¥54",
         "2x 帧率余量"),
        ("电流采样", "无", "INA240 ×2", "+¥24", "堵转保护必需"),
        ("保险丝", "10A", "5A 慢熔", "−¥0", "按实际电流重算"),
        ("散热", "无", "散热片+风扇", "+¥20", "RK3588 需散热"),
    ]
    print(f"  {'项目':<10}{'初版':<20}{'最终':<20}{'价差':>8}  理由")
    print("  " + "-" * 86)
    for a, b, c, d, e in rows:
        print(f"  {a:<10}{b:<20}{c:<20}{d:>8}  {e}")
    print()
    print(f"  合计增加约 ¥666，仍在 ¥2000 预算内。")

    print()
    print("=" * 94)


if __name__ == "__main__":
    main()
