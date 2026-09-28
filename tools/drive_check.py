#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
驱动校核工具：4 × MG513X + TB6612 四路带稳压板（D24A）
=========================================================

**权威参数来源**：
  hardware/electrical/TB6612_MG513X_规格.md
  （WHEELTEC TB6612 资料包 V3.2 + TB6612FNG / RT8279 / RT9013 芯片手册）

本工具回答 6 件事：
  1. 车速（φ60 麦轮，空载 / 额定点 / 横移）
  2. 编码器分辨率与脉冲频率（定时器选型）
  3. 牵引能力（额定/堵转）与「抓地力 vs 电机」谁先到极限
  4. 四轮电流预算（额定 / 两种堵转读数）
  5. ★ TB6612 发热与 65 °C 整板保护风险
  6. ★ 手册参数一致性核对（找出不自洽的一组）

运行：
    python tools/drive_check.py
"""

import math
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


# ==========================================================================
# 权威参数（唯一口径）
# ==========================================================================

# ---- 电机 MG513X（商品页）----
MOTOR = {
    "name": "MG513X",
    "V": 12.0,               # V
    "ratio": 28,             # 1:28
    "rpm_no_load": 370.0,    # 减速后空载
    "rpm_rated": 300.0,      # 减速后额定
    "i_rated": 0.36,         # A
    "i_stall": 3.2,          # A  ⚠️ 手册值，与堵转转矩不自洽，待实测
    "t_rated_kgfcm": 1.0,
    "t_stall_kgfcm": 4.5,    # ⚠️ 同上
    "w_g": 170.0,            # g
    "enc_lines": 13,         # 线/相（采购记录口径）
}

# ---- 驱动板 D24A ----
DRIVER = {
    "name": "TB6612 D24A (四路带稳压)",
    "channels": 4,
    "i_cont": 1.2,           # A 芯片手册 IOUT(ave)
    "i_peak": 3.2,           # A 芯片手册 IOUT(peak)
    "rds_on_total": 0.5,     # Ω 上下桥合计（典型 @VM>=5V）
    "overtemp_c": 65.0,      # 板级整板过温保护阈值
    "pwm_hz": 10_000,        # 推荐
    "vm_min": 2.5,
    "vm_max": 13.5,          # 推荐工作上限（绝对最大 15V）
    "v_5v_a": 5.0,           # RT8279
    "v_3v3_a": 0.5,          # RT9013 LDO
}

WHEEL_D_MM = 60.0
ROBOT_LIMIT_G = 1500.0
BATT_FULL_V = 12.6           # 3S 满电
KGCM_TO_NM = 0.0980665

# 麦轮横移的相对效率（辊子 45° 分解，经验值 0.70–0.75）
MECANUM_STRAFE_EFF = 0.72


def head(s):
    print()
    print("=" * 78)
    print(f"  {s}")
    print("=" * 78)


def table(header, rows):
    widths = [len(h) for h in header]
    for r in rows:
        for i, c in enumerate(r):
            widths[i] = max(widths[i], len(str(c)))
    sep = "+" + "+".join("-" * (w + 2) for w in widths) + "+"
    print(sep)
    print("| " + " | ".join(str(h).ljust(widths[i]) for i, h in enumerate(header)) + " |")
    print(sep)
    for r in rows:
        print("| " + " | ".join(str(c).ljust(widths[i]) for i, c in enumerate(r)) + " |")
    print(sep)


# ==========================================================================
# 1. 车速
# ==========================================================================

def speed_section():
    head("1. 车速（φ60 麦轮）")
    circ = math.pi * WHEEL_D_MM / 1000.0
    print(f"  轮周长 = π × {WHEEL_D_MM:.0f}mm = {circ:.4f} m")
    print()
    rows = []
    for tag, rpm, note in [("空载", MOTOR["rpm_no_load"], "电池满电、无负载"),
                           ("额定点", MOTOR["rpm_rated"], "标称连续工作点")]:
        v = rpm / 60.0 * circ
        rows.append([tag, f"{rpm:.0f}", f"{v:.3f}", f"{v*MECANUM_STRAFE_EFF:.3f}", note])
    table(["状态", "转速rpm", "直行m/s", "横移m/s", "说明"], rows)
    print()
    v_rated = MOTOR["rpm_rated"] / 60.0 * circ
    v_target = 0.8
    print(f"  >> 额定车速 {v_rated:.3f} m/s vs 设计目标 {v_target:.1f} m/s")
    print(f"     超出 {(v_rated/v_target-1)*100:.1f}% —— 属**速度余量**，用 PWM 限速即可。")
    print(f"  >> 横移速度受辊子效率影响，按 {MECANUM_STRAFE_EFF:.2f} 折减 ≈ {v_rated*MECANUM_STRAFE_EFF:.3f} m/s。")
    print("     全向运动的**速度上限应以横移工况为准**。")
    return circ


# ==========================================================================
# 2. 编码器
# ==========================================================================

def encoder_section():
    head("2. 编码器分辨率与脉冲频率")
    ppr = MOTOR["enc_lines"] * MOTOR["ratio"]
    cpr = ppr * 4
    circ = math.pi * WHEEL_D_MM / 1000.0
    print(f"  线数 {MOTOR['enc_lines']} 线/相 × 减速比 1:{MOTOR['ratio']} = {ppr} P/r（单相）")
    print(f"  四倍频计数 = {cpr} count/r")
    print(f"  距离分辨率 = {circ/cpr*1000:.4f} mm/count")
    print()
    rows = []
    for rpm in (MOTOR["rpm_rated"], MOTOR["rpm_no_load"]):
        f1 = rpm / 60.0 * ppr
        rows.append([f"{rpm:.0f}", f"{f1:.0f}", f"{f1*4:.0f}"])
    table(["转速rpm", "单相频率Hz", "四倍频Hz"], rows)
    print()
    print(f"  >> 最高约 {MOTOR['rpm_no_load']/60*ppr*4:.0f} Hz 四倍频 —— G474(170MHz) 编码器模式无压力。")
    print("  >> 但 4 路编码器会占 4 个定时器或 8 个 GPIO，**必须在 CubeMX 先定死**。")
    print("  >> ⚠️ 13 线/相是采购记录口径，**实物应手转 1 圈验证是否为 %d P/r**。" % ppr)
    return cpr


# ==========================================================================
# 3. 牵引能力
# ==========================================================================

def traction_section():
    head("3. 牵引能力与「谁先到极限」")
    r = WHEEL_D_MM / 2000.0
    rows = []
    for tag, t_kgfcm in [("额定", MOTOR["t_rated_kgfcm"]),
                         ("堵转", MOTOR["t_stall_kgfcm"])]:
        T = t_kgfcm * KGCM_TO_NM
        F = T / r
        rows.append([tag, f"{t_kgfcm:.1f}", f"{T:.4f}", f"{F:.2f}", f"{F*4:.1f}",
                     f"{F*4/1.5:.1f}"])
    table(["状态", "转矩kgf·cm", "N·m", "单轮力N", "四轮合力N", "1.5kg加速度m/s²"], rows)
    print()
    print("  平地匀速需求（μ=0.03，1.5kg）：滚阻 ≈ 0.44 N")
    print("  >> 四轮额定合力 13.1 N 远超需求 —— 正常工况下**轮子先打滑，不是电机先堵转**。")
    print("  >> 只有顶墙 / 顶对方车 / 顶死棋子时才会真正堵转。")
    print("  >> 这决定了**堵转保护针对的是「撞墙工况」，不是常规行驶**。")


# ==========================================================================
# 4. 电流预算
# ==========================================================================

def current_section():
    head("4. 四轮电流预算")
    n = DRIVER["channels"]
    i_a = 1.51     # 读法 A（由额定点线性模型反推，见 tools/motor_consistency.py）
    rows = [
        ["额定（直行）", f"{MOTOR['i_rated']:.2f}", f"{MOTOR['i_rated']*n:.2f}",
         f"{MOTOR['i_rated']*n*MOTOR['V']:.1f}", "正常工况"],
        ["堵转·读法 A", f"{i_a:.2f}", f"{i_a*n:.2f}",
         f"{i_a*n*MOTOR['V']:.1f}", "线性模型推算"],
        ["堵转·读法 B（手册）", f"{MOTOR['i_stall']:.2f}", f"{MOTOR['i_stall']*n:.2f}",
         f"{MOTOR['i_stall']*n*MOTOR['V']:.1f}", "⚠️ 手册值，保守设计用"],
    ]
    table(["工况", "单路A", "四路合计A", "@12V功率W", "备注"], rows)
    print()
    print("  >> 峰值/额定 = %.1f 倍（读法 A）/ %.1f 倍（读法 B）" % (i_a*n/(MOTOR['i_rated']*n), MOTOR['i_stall']/MOTOR['i_rated']))
    print("  >> **设计按保守侧（3.2 A）**, 但必须实测定论（见 §6）。")
    print("  >> ⚠️ 3S 2600mAh 电池：额定 1.44 A 可跑 ~1.8 h；堵转 12.8 A 仅 ~12 min。")
    return i_a


# ==========================================================================
# 5. 驱动热
# ==========================================================================

def thermal_section(i_a):
    head("5. TB6612 发热与 65°C 整板保护风险")
    R = DRIVER["rds_on_total"]
    rows = []
    for tag, i in [("额定 0.36A", MOTOR["i_rated"]),
                   ("持续上限 1.2A", DRIVER["i_cont"]),
                   ("堵转A 1.51A", i_a),
                   ("堵转B 3.2A", MOTOR["i_stall"])]:
        p_ch = i * i * R
        rows.append([tag, f"{i:.2f}", f"{p_ch:.3f}", f"{p_ch*2:.2f}", f"{p_ch*4:.2f}"])
    table(["单路电流工况", "电流A", "每路W", "每片W(2路)", "两片W(4路)"], rows)
    print()
    print("  > TB6612FNG 是 SOP24 **无散热焊盘**封装，靠引线散热。")
    print("  > 经验上 >1 W/片 就要注意，>3 W/片 不可能长期承受。")
    print()
    print("  ★★ 最严重的风险：**板级 65°C 保护切断的是 5V/3.3V 轨**")
    print("     而 STM32 与 K230D 正接在这两轨上。")
    print()
    print("     四路堵转 → 两片合计 %.1f W（读法A）/ %.1f W（读法B）" %
          (i_a*i_a*R*4, MOTOR['i_stall']**2*R*4))
    print("       → 整板升温 → >65°C → 切断 5V+3.3V")
    print("       → K230D 掉电重启 + STM32 复位 → 推入动作丢失、里程计清零 → 场上失控")
    print()
    print("  >> 处置：K230D+相机走**独立 DC-DC**；堵转降 PWM 把发热压住。")

    # 粗略估算：达到 65°C 需要多长时间（仅数量级）
    print()
    print("  数量级估算（假设整板有效热容 ~30 J/K、环境 25°C、散热 ~2 K/W）：")
    for tag, p in [("额定", MOTOR["i_rated"]**2*R*4),
                   ("四路堵转A", i_a*i_a*R*4),
                   ("四路堵转B", MOTOR["i_stall"]**2*R*4)]:
        dT = 65.0 - 25.0
        tau = 30.0 * 2.0
        p_ss = p
        if p_ss * 2.0 < dT:
            print(f"    {tag:10s} P={p_ss:.2f}W -> 平衡温升 {p_ss*2.0:.0f}K < 40K，**到不了 65°C**")
        else:
            t65 = -tau * math.log(max(1e-9, 1.0 - dT / (p_ss * 2.0)))
            print(f"    {tag:10s} P={p_ss:.2f}W -> 平衡温升 {p_ss*2.0:.0f}K，约 {t65:.0f}s 到 65°C")
    print("  （仅为数量级，真实值取决于铜箔面积、风冷与安装方式，**必须实测**）")


# ==========================================================================
# 6. 一致性核对
# ==========================================================================

def consistency_section():
    head("6. ★ 手册参数一致性核对")
    print("  ── 完整核对见独立工具：python tools/motor_consistency.py ──")
    print()
    print("  结论（含空载电流的线性模型）：")
    print()
    table(["取信的一组", "推出", "手册", "判读"],
          [["额定 300rpm / 1kgf·cm / 0.36A", "空载 386rpm、额定 294rpm、绕组 7.9Ω",
            "空载 370rpm", "✅ 吻合（读法 A）"],
           ["堵转 3.2A / 4.5kgf·cm", "空载应为 823rpm", "空载 370rpm", "❌ 差 2.2 倍（读法 B）"]])
    print()
    print("  >> **额定点三参数自洽**（轴功率 3.08 W ↔ 标称 4 W）")
    print("  >> **矛盾只集中在「堵转电流 3.2 A」与「堵转转矩 4.5 kgf·cm」**。")
    print()
    print("  30 秒实测定论：")
    print("    1) 万用表量绕组电阻（转子转多点取平均）")
    print("         ≈ 7.9 Ω  → 堵转 ≈ 1.5 A（读法 A）")
    print("         ≈ 3.75 Ω → 堵转 ≈ 3.2 A（读法 B）")
    print("    2) 电源限流 4A，堵转读电流表 + 热电偶看温升")
    print()
    print("  >> **结论不依赖读数**：两个读法都 >1.2 A 持续能力，")
    print("     所以**堵转保护无论如何都必须做**。")


# ==========================================================================
# 7. 质量
# ==========================================================================

def mass_section():
    head("7. 质量预算（按官方规格）")
    m_motor = MOTOR["w_g"] * 4
    items = [
        ["MG513X 电机 ×4", f"{m_motor:.0f}"],
        ["TB6612 D24A", "19"],
        ["3S 2600mAh 电池（估）", "200"],
        ["小计", f"{m_motor+19+200:.0f}"],
        ["整车上限", f"{ROBOT_LIMIT_G:.0f}"],
        ["余给底板/麦轮/推板/相机/主控/五金", f"{ROBOT_LIMIT_G-(m_motor+19+200):.0f}"],
    ]
    table(["项目", "g"], items)
    print()
    print("  >> 🔴 **电机 + 驱动 + 电池已占 %.0f%% 的上限**，必须**先称重再定结构**。"
          % ((m_motor + 19 + 200) / ROBOT_LIMIT_G * 100))


def main():
    head("驱动校核：4 × MG513X + TB6612 四路 D24A")
    print(f"  电机：{MOTOR['name']}  {MOTOR['V']:.0f}V  1:{MOTOR['ratio']}  "
          f"{MOTOR['rpm_no_load']:.0f}rpm 空载 / {MOTOR['rpm_rated']:.0f}rpm 额定")
    print(f"  驱动：{DRIVER['name']}  {DRIVER['i_cont']}A 持续 / {DRIVER['i_peak']}A 峰值")
    print(f"  电池：3S 锂电，满电 {BATT_FULL_V}V（VM 推荐上限 {DRIVER['vm_max']}V）")
    print(f"  轮径：φ{WHEEL_D_MM:.0f}mm 麦轮 ×4")

    speed_section()
    encoder_section()
    traction_section()
    i_a = current_section()
    thermal_section(i_a)
    consistency_section()
    mass_section()

    head("结论")
    print("  1. ✅ 车速 0.94 m/s（额定）> 0.8 m/s 目标 —— 用 PWM 限速，属余量")
    print("  2. ✅ 牵引力远超需求，正常工况轮子先打滑")
    print("  3. ⚠️ 编码器 1456 count/r、9 kHz 四倍频 —— 但 4 路引脚要先规划")
    print("  4. 🔴 **板级 65°C 保护会切断 5V/3.3V，连坐主控与视觉 → 必须独立供电**")
    print("  5. 🔴 **堵转保护必须做**（无论 1.4A 还是 3.2A 都超 1.2A 持续）")
    print("  6. ⚠️ 手册堵转参数不自洽 2.2 倍 —— 量绕组电阻定论")
    print("  7. 🔴 电机 680 g 占质量上限 45% —— 先称重再定结构")
    print()


if __name__ == "__main__":
    main()
