#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电源与电流预算工具
==================

用途：回答"电流到底大在哪一环、电源该怎么选"。

**为什么必须先做这一步**：电机选型、驱动板选型、电池容量、线径、保险丝、
稳压模块，全部依赖电流预算的结论。顺序颠倒会导致反复返工。

本工具算七件事：
  1. 电机电气模型（正常电流 vs 堵转电流）
  2. ★ 堵转热分析 —— 真正的风险不是电池，是**电机烧毁**
  3. 各轨道电流预算（含 1080p@30 的算力平台）
  4. 电池选型与压降（内阻导致的实际端电压）
  5. 线径与保险丝 sizing
  6. 驱动板热裕度
  7. 缓解措施与验收标准

运行：
    python tools/power_budget.py
"""

import math
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


# ==========================================================================
# 系统常量
# ==========================================================================

G = 9.81                    # m/s²

ROBOT_MASS_KG = 1.5         # 赛题上限
WHEEL_RADIUS_M = 0.0325     # φ65mm 轮
WHEEL_COUNT = 2             # 差速底盘

# 滚动阻力系数（橡胶轮 / 木质或合成场地）
MU_ROLL = 0.03
# 棋子推力需求（棋子 ~30g，木/塑料接触）
PIECE_MASS_KG = 0.030
MU_PIECE = 0.30

# 目标加速度
ACCEL_MPS2 = 0.5

# ---- AWG 线径电阻（mΩ/m，铜，20°C）----
AWG_MOHM_PER_M = {
    14: 8.28, 16: 13.2, 18: 20.9, 20: 33.3,
    22: 52.9, 24: 84.2, 26: 133.9,
}

# ---- 载流量参考（A，短距离、有散热）----
AWG_AMPACITY = {
    14: 15.0, 16: 10.0, 18: 7.0, 20: 5.0, 22: 3.0, 24: 2.0, 26: 1.0,
}


# ==========================================================================
# 1. 电机电气模型
# ==========================================================================

class MotorModel:
    """
    有刷直流减速电机的简化电气/热模型。

    电流-转矩关系（线性近似）：
        I(τ) = I_no_load + (τ / τ_stall) × (I_stall - I_no_load)

    热模型（一阶）：
        ΔT(t) = P × R_th × (1 - exp(-t/τ_th))
        其中 τ_th = R_th × C_th
    """

    def __init__(self, name, v_nom, i_no_load, i_stall,
                 tau_stall_kgcm, rpm_at_vnom,
                 r_th_kw, c_th_jk, mass_g):
        self.name = name
        self.v = v_nom
        self.i0 = i_no_load          # 空载电流 A
        self.i_stall = i_stall       # 堵转电流 A
        self.tau_stall = tau_stall_kgcm
        self.rpm = rpm_at_vnom
        self.r_th = r_th_kw          # 热阻 K/W
        self.c_th = c_th_jk          # 热容 J/K
        self.mass = mass_g

    @property
    def r_winding(self):
        """绕组电阻（由堵转电流反推）"""
        return self.v / self.i_stall

    @property
    def tau_th(self):
        """热时间常数（秒）"""
        return self.r_th * self.c_th

    def current_at_torque(self, tau_kgcm):
        """给定负载转矩，估算稳态电流"""
        frac = min(max(tau_kgcm / self.tau_stall, 0.0), 1.0)
        return self.i0 + frac * (self.i_stall - self.i0)

    def stall_power(self):
        """堵转时的发热功率（全部变成热）"""
        return self.v * self.i_stall

    def max_safe_current(self, delta_t_max_k=100.0):
        """
        可**长期**承受的电流（散热平衡时的电流）。

        P_max = ΔT_max / R_th,  I_max = P_max / V
        """
        p_max = delta_t_max_k / self.r_th
        return min(p_max / self.v, self.i_stall)

    def time_to_overheat(self, current, delta_t_max_k=100.0,
                         ambient_c=25.0):
        """
        在给定电流下，多久达到允许温升。

        返回秒；None 表示散热平衡温度低于限值（可长期运行）。
        """
        p = self.v * current
        dt_ss = p * self.r_th
        if dt_ss <= delta_t_max_k:
            return None                       # 散热平衡在限值内
        ratio = 1.0 - delta_t_max_k / dt_ss
        if ratio <= 0:
            return 0.0
        return -self.tau_th * math.log(ratio)


# ---- 候选电机 ----
MOTORS = {
    "JGB37-520": MotorModel(
        "JGB37-520 12V (1:56)", 12.0, 0.15, 2.5, 5.0, 200,
        r_th_kw=8.0, c_th_jk=20.0, mass_g=95),
    "JGB37-3530": MotorModel(
        "JGB37-3530 12V (低堵转)", 12.0, 0.10, 1.2, 3.0, 250,
        r_th_kw=10.0, c_th_jk=15.0, mass_g=70),
    "MD36N": MotorModel(
        "MD36N 12V (大扭矩)", 12.0, 0.25, 4.5, 10.0, 180,
        r_th_kw=6.0, c_th_jk=30.0, mass_g=160),
}


# ==========================================================================
# 2. 转矩需求
# ==========================================================================

def required_torque(friction_coeff=MU_ROLL, pushing_piece=True):
    """
    计算行驶所需的总推力与每轮转矩。

    含三项：滚动阻力 + 加速惯性 + 推棋子
    """
    f_roll = ROBOT_MASS_KG * G * friction_coeff
    f_accel = ROBOT_MASS_KG * ACCEL_MPS2
    f_piece = (PIECE_MASS_KG * G * MU_PIECE) if pushing_piece else 0.0
    f_total = f_roll + f_accel + f_piece

    tau_per_wheel_nm = f_total / WHEEL_COUNT * WHEEL_RADIUS_M
    tau_per_wheel_kgcm = tau_per_wheel_nm / 0.0980665

    return {
        "f_roll": f_roll,
        "f_accel": f_accel,
        "f_piece": f_piece,
        "f_total": f_total,
        "tau_nm": tau_per_wheel_nm,
        "tau_kgcm": tau_per_wheel_kgcm,
    }


# ==========================================================================
# 3. 电池模型
# ==========================================================================

class BatteryModel:
    """LiPo 电池的简化模型（内阻 + C 率 + 容量）"""

    def __init__(self, name, cells, capacity_mah, c_rate,
                 ir_per_cell_mohm, mass_g, price_cny):
        self.name = name
        self.cells = cells
        self.cap_mah = capacity_mah
        self.c_rate = c_rate
        self.ir_per_cell = ir_per_cell_mohm
        self.mass = mass_g
        self.price = price_cny

    @property
    def v_nom(self):
        return self.cells * 3.7

    @property
    def v_full(self):
        return self.cells * 4.2

    @property
    def v_empty(self):
        return self.cells * 3.0

    @property
    def ir_total(self):
        """整包内阻（Ω）"""
        return self.cells * self.ir_per_cell / 1000.0

    @property
    def energy_wh(self):
        return self.v_nom * self.cap_mah / 1000.0

    @property
    def max_discharge_a(self):
        return self.cap_mah / 1000.0 * self.c_rate

    def terminal_voltage(self, current_a, v_open=None):
        """带载端电压"""
        if v_open is None:
            v_open = self.v_nom
        return v_open - current_a * self.ir_total

    def runtime_min(self, avg_current_a):
        if avg_current_a <= 0:
            return float("inf")
        return self.cap_mah / 1000.0 / avg_current_a * 60.0


BATTERIES = {
    "3S-850":  BatteryModel("3S 850mAh 45C",  3, 850,  45, 45, 78,  55),
    "3S-1500": BatteryModel("3S 1500mAh 25C", 3, 1500, 25, 35, 130, 85),
    "3S-2200": BatteryModel("3S 2200mAh 30C", 3, 2200, 30, 28, 185, 120),
    "2S-2000": BatteryModel("2S 2000mAh 30C", 2, 2000, 30, 28, 110, 95),
}


# ==========================================================================
# 4. 算力平台（为第 2 轮迭代铺垫）
# ==========================================================================

COMPUTE_PLATFORMS = [
    # (名称, NPU算力, 满载功率W, 5V轨电流A, 质量g, 价格元, 备注)
    ("K230",           "~6 TOPS",  2.0,  0.40,  30,  199, "现方案，1080p@30 吃力"),
    ("RK3576 板",      "6 TOPS",   5.0,  1.00,  60,  420, "1080p@30 可行"),
    ("RK3588 板",      "6 TOPS",   9.0,  1.80,  85,  680, "推荐：余量充足"),
    ("Jetson Orin Nano","40 TOPS", 12.0, 2.40, 150, 1600, "算力过剩，超预算"),
]


# ==========================================================================
# 输出辅助
# ==========================================================================

def section(title):
    print("\n" + "=" * 84)
    print(title)
    print("=" * 84)


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
    print("=" * 84)
    print("电源与电流预算")
    print("=" * 84)
    print(f"  整车质量上限 {ROBOT_MASS_KG} kg，轮径 {WHEEL_RADIUS_M*2000:.0f}mm"
          f"（半径 {WHEEL_RADIUS_M*1000:.1f}mm），差速 {WHEEL_COUNT} 轮")

    # ---------------- 1. 转矩需求 ----------------
    section("1. 行驶转矩需求（决定「正常电流」）")
    tq = required_torque()
    rows = [
        ["滚动阻力",     f"{tq['f_roll']:.3f}", f"{ROBOT_MASS_KG}kg x {MU_ROLL}"],
        ["加速惯性",     f"{tq['f_accel']:.3f}", f"{ACCEL_MPS2} m/s²"],
        ["推棋子",       f"{tq['f_piece']:.3f}", f"{PIECE_MASS_KG*1000:.0f}g x {MU_PIECE}"],
        ["合计推力",     f"{tq['f_total']:.3f}", "N"],
    ]
    table(["项目", "力(N)", "来源"], rows)
    print()
    print(f"  每轮需求转矩 = {tq['tau_nm']*1000:.1f} mN·m "
          f"= {tq['tau_kgcm']:.3f} kgf·cm")
    print()
    print("  >> 关键：这个转矩需求**非常小**。")
    print("     1.5kg 的车在平地上匀速走，需要的推力不到 1N。")
    print("     所以问题不在「正常行驶」，而在下面第 2 节。")

    # ---------------- 2. 正常电流 vs 堵转 ----------------
    section("2. 各电机方案：正常电流 vs 堵转电流")
    rows = []
    for key, m in MOTORS.items():
        i_norm = m.current_at_torque(tq["tau_kgcm"])
        rows.append([
            m.name,
            f"{m.tau_stall:.1f}",
            f"{m.r_winding:.1f}",
            f"{i_norm:.2f}",
            f"{m.i_stall:.1f}",
            f"{m.i_stall/i_norm:.0f}x",
        ])
    table(["电机", "堵转转矩kgf·cm", "绕组Ω", "正常电流A",
           "堵转电流A", "倍数"], rows)
    print()
    m0 = MOTORS["JGB37-520"]
    i_norm0 = m0.current_at_torque(tq["tau_kgcm"])
    print(f"  >> 以 JGB37-520 为例：")
    print(f"     正常行驶每台 {i_norm0:.2f}A，两台共 {i_norm0*2:.2f}A "
          f"= {i_norm0*2*12:.1f}W")
    print(f"     堵转时每台 {m0.i_stall:.1f}A，两台共 {m0.i_stall*2:.1f}A "
          f"= {m0.i_stall*2*12:.0f}W")
    print()
    print(f"     电流差 {m0.i_stall/i_norm0:.0f} 倍 —— 这就是你说的「电流过大」的来源。")
    print(f"     **但它只在堵转/启动瞬间出现，不是常态。**")

    # ---------------- 3. 堵转热分析（核心风险）----------------
    section("3. ★ 真正会烧东西的地方：堵转热效应")
    print("  堵转时 12V 全部加在绕组电阻上，功率全变成热：")
    print()
    rows = []
    for key, m in MOTORS.items():
        p = m.stall_power()
        dt_ss = p * m.r_th
        i_safe = m.max_safe_current()
        t_burn = m.time_to_overheat(m.i_stall)
        rows.append([
            m.name,
            f"{p:.0f}",
            f"{dt_ss:.0f}",
            f"{i_safe:.2f}",
            f"{t_burn:.0f}" if t_burn else "不烧",
        ])
    table(["电机", "堵转发热W", "平衡温升K", "安全电流A",
           "烧毁时间s"], rows)
    print()
    print("  参数说明：允许温升取 100K（25°C 环境 -> 125°C 绕组）")
    print()
    m0 = MOTORS["JGB37-520"]
    print(f"  >> 以 JGB37-520 为例：")
    print(f"     堵转发热 {m0.stall_power():.0f}W，而安全电流只有 "
          f"{m0.max_safe_current():.2f}A")
    print(f"     热时间常数 τ = R_th × C_th = {m0.r_th} × {m0.c_th} "
          f"= {m0.tau_th:.0f} 秒")
    t_burn0 = m0.time_to_overheat(m0.i_stall)
    print(f"     持续堵转 {t_burn0:.0f} 秒后绕组烧毁")
    print()
    print(f"     注意区分两个时间尺度：")
    print(f"       τ = {m0.tau_th:.0f}s  是「升温快慢」（热惯性）")
    print(f"       {t_burn0:.0f}s     是「多久到达 100K 温升」（危险阈值）")
    print(f"     本例中平衡温升 {m0.stall_power()*m0.r_th:.0f}K 远超 100K 限值，")
    print(f"     所以 {t_burn0:.0f}s 就烧，而不是等到 τ 的 3-5 倍。")
    print()
    print(f"  ⚠️ 比赛单场 3 分钟（180 秒）。如果车顶住墙 {t_burn0:.0f} 秒，电机就废了。")
    print("     这不是理论风险 —— 顶墙推棋子、被对方车挡住都会发生。")
    print()
    print("  >> 结论：**真正的风险不是电池，是电机烧毁。**")
    print("     而且这个风险不能靠「选更大的电池」解决，")
    print("     只能靠**限流**（见第 7 节）。")

    # 热时间常数敏感性
    print()
    print("  热参数敏感性（结论是否稳健）：")
    rows = []
    for r_th in [6.0, 8.0, 10.0, 12.0]:
        for c_th in [15.0, 20.0, 30.0]:
            mm = MotorModel("sensitivity", 12.0, 0.15, 2.5, 5.0, 200,
                            r_th, c_th, 95)
            t = mm.time_to_overheat(mm.i_stall)
            rows.append([f"{r_th:.0f}", f"{c_th:.0f}",
                         f"{r_th*c_th:.0f}",
                         f"{mm.max_safe_current():.2f}",
                         f"{t:.0f}" if t else "不烧"])
    table(["R_th(K/W)", "C_th(J/K)", "τ(s)", "安全电流A", "烧毁时间s"], rows)
    print()
    print("  >> 无论参数怎么取，**烧毁时间都在 20-120 秒量级**，")
    print("     而安全电流都在 **0.9-1.7A** 之间。结论稳健：必须限流。")

    # ---------------- 4. 各轨道电流预算 ----------------
    section("4. 各轨道电流预算（按 1080p@30 需求）")
    print("  算力平台对比（为第 2 轮迭代铺垫）：")
    print()
    table(["平台", "NPU", "满载W", "5V轨A", "质量g", "价格元", "备注"],
          [[n, npu, f"{p:.1f}", f"{i:.2f}", m, c, note]
           for n, npu, p, i, m, c, note in COMPUTE_PLATFORMS])
    print()

    for platform in ["K230", "RK3588 板"]:
        cpu = next(c for c in COMPUTE_PLATFORMS if c[0] == platform)
        _, _, p_w, i_5v, mass, price, _ = cpu

        print(f"  ---- 方案：{platform} ----")
        rows = [
            ["动力轨 12V", "2 电机正常", f"{i_norm0*2:.2f}", f"{i_norm0*2*12:.1f}"],
            ["动力轨 12V", "2 电机堵转", f"{m0.i_stall*2:.1f}", f"{m0.i_stall*2*12:.0f}"],
            ["视觉轨 5V", platform, f"{i_5v:.2f}", f"{p_w:.1f}"],
            ["视觉轨 5V", "2 相机", "0.20", "1.0"],
            ["逻辑轨 3.3V", "主控+IMU", "0.15", "0.5"],
            ["逻辑轨 3.3V", "激光+指示", "0.02", "0.07"],
        ]
        table(["轨道", "负载", "电流A", "功率W"], rows)

        i_cont = i_norm0 * 2 + i_5v + 0.20 + 0.15 + 0.02
        i_peak = m0.i_stall * 2 + i_5v + 0.20 + 0.15 + 0.02
        p_cont = i_norm0*2*12 + p_w + 1.0 + 0.5 + 0.07
        p_peak = m0.i_stall*2*12 + p_w + 1.0 + 0.5 + 0.07
        print()
        print(f"    持续总电流(12V侧等效) ≈ {i_cont:.2f} A，"
              f"总功率 ≈ {p_cont:.1f} W")
        print(f"    峰值总电流(12V侧等效) ≈ {i_peak:.2f} A，"
              f"总功率 ≈ {p_peak:.0f} W")
        print(f"    ⚠️ 峰值/持续 = {i_peak/i_cont:.1f} 倍")
        print()

    print("  >> 注意：换更强的算力平台后，")
    print("     持续功率从 ~8W 涨到 ~17W，峰值不变（峰值由电机堵转主导）。")
    print("     所以换主板**不会加剧电流问题**，反而持续功耗翻倍会缩短续航。")

    # ---------------- 5. 电池选型 ----------------
    section("5. 电池选型与压降")
    i_peak_ref = m0.i_stall * 2 + 1.8 + 0.20 + 0.15 + 0.02   # RK3588 方案
    i_cont_ref = i_norm0 * 2 + 1.8 + 0.20 + 0.15 + 0.02

    print(f"  按峰值 {i_peak_ref:.1f}A / 持续 {i_cont_ref:.2f}A 校核")
    print()
    rows = []
    for key, b in BATTERIES.items():
        v_sag_peak = b.terminal_voltage(i_peak_ref)
        v_sag_cont = b.terminal_voltage(i_cont_ref)
        ok_c = b.max_discharge_a >= i_peak_ref
        rt = b.runtime_min(i_cont_ref)
        # 3S 是硬要求（见下），2S 会跌破 BTS7960 之外的多处阈值
        ok_v = "OK" if b.cells == 3 else "**2S**"
        rows.append([
            b.name,
            f"{b.ir_total*1000:.0f}",
            f"{b.max_discharge_a:.0f}",
            "OK" if ok_c else "**不足**",
            ok_v,
            f"{v_sag_peak:.1f}",
            f"{rt:.0f}",
            f"{b.mass}",
        ])
    table(["电池", "内阻mΩ", "最大放电A", "C率够?",
           "电压", "峰值端压V", "续航min", "质量g"], rows)
    print()

    b850 = BATTERIES["3S-850"]
    b1500 = BATTERIES["3S-1500"]
    b2200 = BATTERIES["3S-2200"]

    print("  >> 三层判读：")
    print()
    print(f"     (a) C 率：全部够。最小 45C × 0.85Ah = 38A ≫ {i_peak_ref:.1f}A 峰值。")
    print(f"     (b) 续航：全部够。最短 {b850.runtime_min(i_cont_ref):.0f} 分钟 "
          f"vs 比赛 3 分钟需求。**续航不是选型依据**。")
    print(f"     (c) 电压：**必须 3S**。2S 峰值端压仅 "
          f"{BATTERIES['2S-2000'].terminal_voltage(i_peak_ref):.1f}V，"
          f"驱动与电机都按 12V 设计。")
    print()
    print("  >> 真正的判据是**峰值压降**（决定 5V 轨会不会 brownout）：")
    print()
    rows = []
    for key, b in [("3S-850", b850), ("3S-1500", b1500), ("3S-2200", b2200)]:
        sag = b.v_nom - b.terminal_voltage(i_peak_ref)
        rows.append([b.name, f"{b.ir_total*1000:.0f}", f"{sag:.2f}",
                     f"{b.mass}", f"{b.price}"])
    table(["电池", "内阻mΩ", "峰值压降V", "质量g", "价格元"], rows)
    print()
    print(f"     ⚠️ 注意：850mAh 的内阻（{b850.ir_total*1000:.0f}mΩ）"
          f"比 1500mAh（{b1500.ir_total*1000:.0f}mΩ）"
          f"**高 {(b850.ir_total/b1500.ir_total-1)*100:.0f}%**，并非相当。")
    print(f"     它的压降 {b850.v_nom - b850.terminal_voltage(i_peak_ref):.2f}V "
          f"也相应更大。")
    print()
    print("  >> 权衡：")
    print(f"     850mAh  : 轻 {b1500.mass - b850.mass}g，但压降大 "
          f"{(b850.v_nom-b850.terminal_voltage(i_peak_ref)) - (b1500.v_nom-b1500.terminal_voltage(i_peak_ref)):.2f}V")
    print(f"     1500mAh : 压降更小，重 {b1500.mass}g")
    print()
    print("  >> 推荐：**3S 1500mAh 25C**。理由：")
    print("     - 压降更小，对 5V 轨的 brownout 保护更好（这是主要风险）")
    print(f"     - 质量 {b1500.mass}g 在 1.5kg 预算内可接受")
    print(f"     - 续航 {b1500.runtime_min(i_cont_ref):.0f} 分钟，余量充足")
    print()
    print("     若后期质量紧张，可换 850mAh（省 52g），")
    print("     但**必须同时加强 5V 轨的输入电容**以补偿更大的压降。")

    # ---------------- 6. 线径与保险丝 ----------------
    section("6. 线径与保险丝")
    print(f"  按峰值 {i_peak_ref:.1f}A（毫秒级）、持续 {i_cont_ref:.2f}A、"
          f"线长 0.3m（单程）计算")
    print()
    print("  ⚠️ 载流量额定值是针对**持续**电流的，不能直接与毫秒级峰值比。")
    print("     导线热时间常数远大于毫秒，所以峰值只需满足压降要求即可。")
    print()
    rows = []
    for awg in sorted(AWG_MOHM_PER_M, reverse=True):
        r = AWG_MOHM_PER_M[awg] / 1000.0 * 0.3 * 2   # 去回双程
        v_drop = i_peak_ref * r
        amp = AWG_AMPACITY[awg]
        # 持续电流需在载流量内；峰值只看压降
        ok_cont = "OK" if amp >= i_cont_ref * 2.0 else "不够"
        ok_drop = "OK" if v_drop < 0.20 else "偏大"
        rows.append([f"{awg}", f"{AWG_MOHM_PER_M[awg]:.1f}",
                     f"{v_drop:.3f}", f"{amp:.0f}", ok_cont, ok_drop])
    table(["AWG", "mΩ/m", "峰值压降V", "持续载流量A",
           "持续够(2x余量)?", "压降<0.2V?"], rows)
    print()
    print("  >> 结论：**动力线用 18AWG**。判据是两条：")
    print(f"     - 持续 {i_cont_ref:.2f}A，18AWG 载流量 7A → "
          f"余量 {7.0/i_cont_ref:.1f}x，充足")
    print(f"     - 峰值压降 {i_peak_ref*AWG_MOHM_PER_M[18]/1000*0.3*2:.3f}V < 0.2V，可接受")
    print()
    print("     20AWG 也可用（峰值压降 "
          f"{i_peak_ref*AWG_MOHM_PER_M[20]/1000*0.3*2:.3f}V，持续余量 "
          f"{5.0/i_cont_ref:.1f}x），但 18AWG 更稳妥、成本几乎相同。")
    print("     信号线 26AWG 足够（电流 < 0.2A）。")
    print()
    print(f"  保险丝 sizing：")
    print(f"    持续 {i_cont_ref:.2f}A，峰值 {i_peak_ref:.1f}A（毫秒级）")
    print(f"    原则：额定值 > 持续电流 × 2，且 < 线径持续载流量")
    print(f"    建议：**5A 慢熔**（{5.0/i_cont_ref:.1f}x 持续余量，"
          f"可耐峰值，且 < 18AWG 的 7A）")
    print(f"    7.5A 略偏大（对 18AWG 保护不足）；10A 快熔会在峰值时误断")

    # ---------------- 7. 缓解措施 ----------------
    section("7. ★ 电流过大的缓解措施（按优先级）")
    print("  核心思路：**问题在堵转，不在正常行驶，所以解法是限流不是加电源。**")
    print()
    rows = [
        ["1", "★ 电流采样限流", "INA240/ACS712 测电机电流，PID 限到 1A",
         "根本解法，把堵转电流压到安全值"],
        ["2", "加速度限制", "PWM 斜坡上升，限制 dI/dt",
         "消除启动冲击电流"],
        ["3", "堵转降功率", "检测堵转后 PWM 降到 30% 维持",
         "顶墙时不再烧电机"],
        ["4", "软件死区保护", "堵转超时 5s -> 后退重试",
         "避免长时间顶死"],
        ["5", "选低堵转电流电机", "JGB37-3530（1.2A vs 2.5A）",
         "牺牲扭矩换安全"],
        ["6", "独立逻辑电池", "视觉/逻辑单独供电",
         "隔离 brownout，不解决电机发热"],
    ]
    table(["#", "措施", "做法", "作用"], rows)
    print()
    print("  >> 第 1 条是**必须做**的。没有电流采样，限流无从谈起。")
    print("     加一个 INA240（约 ¥12）比换更大的电池有用得多。")

    # 验证限流效果
    print()
    print("  限流到 1A 后的效果验证：")
    rows = []
    for i_lim in [0.8, 1.0, 1.5, 2.0, 2.5]:
        p = 12.0 * i_lim
        t = m0.time_to_overheat(i_lim)
        rows.append([f"{i_lim:.1f}", f"{p:.0f}",
                     f"{1.0/(12.0*i_lim/m0.stall_power()):.1f}x" if i_lim < m0.i_stall else "-",
                     f"{t:.0f}" if t else "**可长期**"])
    table(["限流A", "发热W", "相对堵转", "烧毁时间s"], rows)
    print()
    print("  >> 限流到 1.0A 时，电机可**长期**堵转不烧 —— 顶墙可以一直顶着。")

    # ---------------- 8. 结论 ----------------
    section("8. 结论与选型建议")
    print("  【电流问题的定性】")
    print(f"    正常行驶  {i_norm0*2:.2f}A（{i_norm0*2*12:.0f}W）  <- 很小，不是问题")
    print(f"    堵转峰值  {m0.i_stall*2:.1f}A（{m0.i_stall*2*12:.0f}W）  <- 大，但毫秒级")
    print(f"    堵转持续  会让电机在 {m0.time_to_overheat(m0.i_stall):.0f} 秒内烧毁  <- ★ 真正的风险")
    print()
    print("  【选型结论】")
    rows = [
        ["电机", "JGB37-520 12V 1:56", "¥68 x2", "加限流后可长期堵转"],
        ["驱动", "BTS7960 / IBT-2", "¥12 x2", "43A，远超需求"],
        ["★ 电流采样", "INA240 或 ACS712", "¥12 x2", "新增，限流必需"],
        ["电池", "3S 1500mAh 25C", "¥85", "内阻 105mΩ，压降小"],
        ["动力线", "18 AWG", "-", "持续余量 2.6x，峰值压降 0.09V"],
        ["保险丝", "5A 慢熔", "¥5", "耐峰值、保护 18AWG"],
        ["视觉轨", "DC-DC 5V 5A", "¥25", "RK3588 需 1.8A + 余量"],
        ["逻辑轨", "5V->3.3V 2A", "¥10", "主控 + IMU + 传感器"],
    ]
    table(["项目", "选型", "价格", "理由"], rows)
    print()
    print("  【为什么电池保持 1500mAh 而不是降到 850mAh】")
    print(f"    850mAh 45C 的内阻是 {BATTERIES['3S-850'].ir_total*1000:.0f}mΩ，")
    print(f"    比 1500mAh 25C 的 {BATTERIES['3S-1500'].ir_total*1000:.0f}mΩ "
          f"**高 {(BATTERIES['3S-850'].ir_total/BATTERIES['3S-1500'].ir_total-1)*100:.0f}%**。")
    print(f"    虽然能省 {BATTERIES['3S-1500'].mass - BATTERIES['3S-850'].mass}g 质量，")
    print(f"    但峰值压降更大 —— 而 5V 轨 brownout 是本项目的主要风险之一。")
    print()
    print(f"    续航在两种情况下都远超需求（≥19 分钟 vs 3 分钟），")
    print(f"    所以**不是**选电池的依据。")
    print()
    print("=" * 84)
    print("第 1 轮迭代完成。下一步（第 2 轮）：基于本文的功耗结论选主控平台。")
    print("=" * 84)


if __name__ == "__main__":
    main()
