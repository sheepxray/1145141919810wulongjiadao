# -*- coding: utf-8 -*-
"""
一致性核对（独立小工具）：MG513X 手册四参数是否自洽
====================================================

直流电机线性模型（含空载电流）：
    T   = Kt · (I − I0)          转矩-电流
    ω   = (V − I·R) / Ke         转速-电压   （Kt = Ke，SI 制）
    ω0  = V / Ke                 空载转速

已知手册给的 4 个点：
    空载 370 rpm、额定 300 rpm / 0.36 A / 1 kgf·cm、堵转 3.2 A / 4.5 kgf·cm

本工具检验：取哪 3 个能自洽、剩下的第 4 个错多少。

运行：python tools/motor_consistency.py
"""

import math
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

KGCM = 0.0980665          # kgf·cm -> N·m
V = 12.0
I0 = 0.030                # 空载电流（经验值，未在商品页给出）

RPM_NO_LOAD = 370.0
RPM_RATED = 300.0
I_RATED = 0.36
T_RATED = 1.0 * KGCM
T_STALL = 4.5 * KGCM
I_STALL_MANUAL = 3.2

def rpm(w_rad):
    return w_rad * 60.0 / (2 * math.pi)

def w_rad(r):
    return r * 2 * math.pi / 60.0

print("=" * 78)
print("  MG513X 手册参数一致性核对")
print("=" * 78)
print(f"  手册：空载 {RPM_NO_LOAD:.0f} rpm | 额定 {RPM_RATED:.0f} rpm / "
      f"{I_RATED:.2f} A / {T_RATED/KGCM:.1f} kgf·cm | "
      f"堵转 {I_STALL_MANUAL:.1f} A / {T_STALL/KGCM:.1f} kgf·cm")
print(f"  假设空载电流 I0 = {I0:.3f} A（商品页未给，经验值）")
print()

# ---- 读法 A：以「额定点 + 空载转速」定 Kt，再推堵转 ----
Kt_A = T_RATED / (I_RATED - I0)
w0_A = V / Kt_A
R_A = V / (I0 + T_STALL / Kt_A)          # 由空载转速+堵转转矩反推绕组
wr_A = (V - I_RATED * R_A) / Kt_A

print("─" * 78)
print("  读法 A：取信【额定 300rpm / 1 kgf·cm / 0.36 A】")
print("─" * 78)
print(f"    Kt = Ke = T_rated/(I_rated−I0) = {Kt_A:.4f} N·m/A")
print(f"    反推空载转速 = V/Kt = {rpm(w0_A):.0f} rpm   （手册 {RPM_NO_LOAD:.0f}）"
      f"   偏差 {abs(rpm(w0_A)-RPM_NO_LOAD)/RPM_NO_LOAD*100:.1f}%  ✅")
print(f"    反推额定转速 = {rpm(wr_A):.0f} rpm   （手册 {RPM_RATED:.0f}）"
      f"   偏差 {abs(rpm(wr_A)-RPM_RATED)/RPM_RATED*100:.1f}%  ✅")
print(f"    反推绕组电阻 = {R_A:.2f} Ω")
i_stall_A = I0 + T_STALL / Kt_A
print(f"    反推堵转电流 = I0 + T_stall/Kt = {i_stall_A:.2f} A   "
      f"（手册 {I_STALL_MANUAL:.1f} A）  ❌ 差 {I_STALL_MANUAL/i_stall_A:.1f} 倍")
print()
print(f"    >> 读法 A 自洽度最高：3 个转速/转矩点吻合，只有「堵转电流」偏小。")
print()

# ---- 读法 B：以「堵转 3.2A / 4.5kgf·cm」定 Kt，再推空载 ----
Kt_B = T_STALL / (I_STALL_MANUAL - I0)
w0_B = V / Kt_B
print("─" * 78)
print("  读法 B：取信【堵转 3.2 A / 4.5 kgf·cm】")
print("─" * 78)
print(f"    Kt = T_stall/(I_stall−I0) = {Kt_B:.4f} N·m/A")
print(f"    反推空载转速 = {rpm(w0_B):.0f} rpm   （手册 {RPM_NO_LOAD:.0f}）"
      f"   ❌ 差 {rpm(w0_B)/RPM_NO_LOAD:.1f} 倍")
print(f"    反推绕组电阻 = {V/I_STALL_MANUAL:.2f} Ω")
wr_B = (V - I_RATED * (V / I_STALL_MANUAL)) / Kt_B
print(f"    反推额定转速 = {rpm(wr_B):.0f} rpm   （手册 {RPM_RATED:.0f}）"
      f"   偏差 {abs(rpm(wr_B)-RPM_RATED)/RPM_RATED*100:.1f}%")
print()
print("    >> 读法 B 与「空载转速」严重冲突（2.2 倍），不可信。")
print()

# ---- 结论 ----
print("=" * 78)
print("  结论")
print("=" * 78)
print(f"  1. 「额定点」三参数自洽：轴功率 = T·ω = "
      f"{T_RATED*w_rad(RPM_RATED):.2f} W ↔ 标称 4 W  ✅")
print(f"  2. 矛盾集中在【堵转电流 3.2 A】与【堵转转矩 4.5 kgf·cm】：")
print(f"       按转矩+额定点推，堵转电流只应是 {i_stall_A:.2f} A（绕组 ≈ {R_A:.1f} Ω）")
print(f"       按堵转电流 3.2 A 推，空载转速应是 {rpm(w0_B):.0f} rpm（手册 370）")
print()
print("  3. 30 秒实测定论：")
print(f"       万用表量绕组电阻 ≈ {R_A:.1f} Ω  -> 堵转 ≈ {i_stall_A:.1f} A（读法 A）")
print(f"       万用表量绕组电阻 ≈ {V/I_STALL_MANUAL:.2f} Ω  -> 堵转 ≈ 3.2 A（读法 B）")
print()
print("  4. ★ 但两种读法的堵转电流都 > TB6612 的 1.2 A 持续能力：")
print(f"       读法 A {i_stall_A:.2f} A -> 超持续 {i_stall_A/1.2:.1f} 倍")
print(f"       读法 B 3.20 A -> 超持续 {3.2/1.2:.1f} 倍")
print("       => 堵转保护无论如何都必须做，只是限流阈值不同。")
print()
