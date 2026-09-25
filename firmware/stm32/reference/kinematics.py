#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
差速运动学 / PID / 航迹推算 参考实现
====================================

用途：**先在 PC 上验证算法正确，再移植到 STM32**。
在 MCU 上调试数学公式代价高得多。

对应文档：docs/03（控制环）、docs/05（定位）

运行自测：
    python firmware/stm32/reference/kinematics.py
"""

import math
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


# ==========================================================================
# 差速运动学
# ==========================================================================

class DiffDrive:
    """
    差速底盘运动学。

    参数单位统一为 SI：米、秒、弧度。
    """

    def __init__(self, wheel_radius_m, wheel_base_m, ticks_per_rev,
                 max_wheel_speed_mps=1.5):
        self.r = wheel_radius_m          # 轮半径
        self.B = wheel_base_m            # 轮距（左右轮中心距）
        self.ticks = ticks_per_rev       # 编码器每转计数
        self.max_v = max_wheel_speed_mps

        # 每计数对应的轮面位移
        self.m_per_tick = (2 * math.pi * self.r) / self.ticks

    # ---- 正运动学：轮速 -> 车体速度 ----
    def forward(self, v_left, v_right):
        """(v_l, v_r) -> (v, omega)"""
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.B
        return v, omega

    # ---- 逆运动学：车体速度 -> 轮速 ----
    def inverse(self, v, omega):
        """(v, omega) -> (v_l, v_r)"""
        v_l = v - omega * self.B / 2.0
        v_r = v + omega * self.B / 2.0
        return v_l, v_r

    # ---- 限幅 ----
    def clamp_wheels(self, v_l, v_r):
        """把轮速限制在电机能力内，保持左右比例（即保持转向半径）"""
        peak = max(abs(v_l), abs(v_r))
        if peak <= self.max_v:
            return v_l, v_r
        k = self.max_v / peak
        return v_l * k, v_r * k

    # ---- 编码器 -> 位移 ----
    def ticks_to_distance(self, delta_ticks):
        return delta_ticks * self.m_per_tick


# ==========================================================================
# PID（位置式 + 抗积分饱和 + 微分低通）
# ==========================================================================

class PID:
    """
    通用 PID。

    anti_windup: 输出饱和时停止积分累积（比简单钳位效果好）
    d_filter:    微分项一阶低通系数 alpha ∈ (0,1]，1.0 表示不滤波
                 （编码器/陀螺噪声大时必须滤波，否则微分项放大噪声）
    """

    def __init__(self, kp, ki, kd, out_min, out_max,
                 anti_windup=True, d_filter=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.anti_windup = anti_windup
        self.d_filter = d_filter

        self.integral = 0.0
        self.prev_err = None
        self.prev_d = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_err = None
        self.prev_d = 0.0

    def update(self, setpoint, measured, dt):
        if dt <= 0:
            return 0.0

        err = setpoint - measured

        # 比例
        p = self.kp * err

        # 积分
        self.integral += err * dt
        i = self.ki * self.integral

        # 微分（带低通）
        if self.prev_err is None:
            d = 0.0
        else:
            raw_d = (err - self.prev_err) / dt
            d = self.d_filter * raw_d + (1.0 - self.d_filter) * self.prev_d
        self.prev_d = d
        dterm = self.kd * d

        out = p + i + dterm

        # 输出饱和 + 抗积分饱和
        if out > self.out_max:
            out = self.out_max
            if self.anti_windup and err > 0:
                self.integral -= err * dt      # 回退这次积分
        elif out < self.out_min:
            out = self.out_min
            if self.anti_windup and err < 0:
                self.integral -= err * dt

        self.prev_err = err
        return out


# ==========================================================================
# 航迹推算
# ==========================================================================

class Odometry:
    """
    基于编码器 + 陀螺的航迹推算。

    ★ 关键：航向优先用陀螺（IMU），而不是左右轮差。
      理由：轮子打滑会让差分航向产生大误差，而陀螺是直接测角速度。
    """

    def __init__(self, diff: DiffDrive, use_gyro=True):
        self.dd = diff
        self.use_gyro = use_gyro

        self.x = 0.0          # m
        self.y = 0.0          # m
        self.theta = 0.0      # rad，0 = 朝 +x，逆时针为正

        self.gyro_bias = 0.0  # rad/s

    def calibrate_gyro(self, samples):
        """静止标定：传入一段静止时的陀螺读数（rad/s）"""
        if not samples:
            return
        self.gyro_bias = sum(samples) / len(samples)

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta

    def update(self, d_ticks_left, d_ticks_right, dt,
               gyro_rate=None):
        """
        推进一个时间步。

        d_ticks_*: 本步的编码器增量（计数）
        gyro_rate: 陀螺角速度 rad/s（若 use_gyro）
        """
        dl = self.dd.ticks_to_distance(d_ticks_left)
        dr = self.dd.ticks_to_distance(d_ticks_right)

        # 航向增量
        if self.use_gyro and gyro_rate is not None:
            dtheta = (gyro_rate - self.gyro_bias) * dt
        else:
            _, omega = self.dd.forward(dl / dt, dr / dt)
            dtheta = omega * dt

        # 位移（取中点朝向，比用起始朝向精度高一个量级）
        ds = (dl + dr) / 2.0
        theta_mid = self.theta + dtheta / 2.0

        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)
        self.theta = wrap_pi(self.theta + dtheta)

    def pose(self):
        return self.x, self.y, self.theta


def wrap_pi(a):
    """把角度归一化到 (-pi, pi]"""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


# ==========================================================================
# 标定辅助
# ==========================================================================

def calibrate_wheel_radius(actual_distance_m, avg_ticks,
                           nominal_radius_m, ticks_per_rev):
    """
    由实测距离反算**有效**轮径。

    ★ 不要用游标卡尺量出来的轮径 —— 轮胎压缩、打滑都会让有效值不同。
    """
    if avg_ticks == 0:
        return nominal_radius_m
    return actual_distance_m * ticks_per_rev / (2 * math.pi * avg_ticks)


def calibrate_wheel_base(actual_rotation_rad, d_ticks_left, d_ticks_right,
                         diff: DiffDrive):
    """由原地旋转实测角度反算有效轮距。"""
    if actual_rotation_rad == 0:
        return diff.B
    dl = diff.ticks_to_distance(d_ticks_left)
    dr = diff.ticks_to_distance(d_ticks_right)
    # dtheta = (dr - dl) / B  =>  B = (dr - dl) / dtheta
    return (dr - dl) / actual_rotation_rad


# ==========================================================================
# 自测
# ==========================================================================

def selftest():
    print("=" * 78)
    print("差速运动学 / PID / 航迹推算 自测")
    print("=" * 78)

    # ---- 1. 正逆运动学互逆 ----
    print("\n[1] 正逆运动学互逆性")
    dd = DiffDrive(wheel_radius_m=0.0325, wheel_base_m=0.15,
                   ticks_per_rev=500 * 4)
    ok = True
    for v, w in [(0.8, 0.0), (0.0, 1.0), (0.5, -0.5), (-0.3, 2.0)]:
        vl, vr = dd.inverse(v, w)
        v2, w2 = dd.forward(vl, vr)
        if abs(v - v2) > 1e-12 or abs(w - w2) > 1e-12:
            ok = False
    print(f"  forward(inverse(v,w)) == (v,w)  {'通过' if ok else '失败'}")
    assert ok

    # ---- 2. 单位换算 ----
    print("\n[2] 编码器换算")
    # 轮周长
    circ = 2 * math.pi * 0.0325
    print(f"  轮周长 = {circ*1000:.2f} mm")
    print(f"  每计数位移 = {dd.m_per_tick*1000:.4f} mm "
          f"({dd.ticks} 计数/转)")
    d = dd.ticks_to_distance(dd.ticks)   # 整转
    print(f"  整转位移 = {d*1000:.2f} mm  "
          f"{'通过' if abs(d - circ) < 1e-9 else '失败'}")
    assert abs(d - circ) < 1e-9

    # ---- 3. 限幅保持转向半径 ----
    print("\n[3] 轮速限幅保持转向半径")
    vl, vr = dd.inverse(2.0, 5.0)          # 超速
    print(f"  原始: L={vl:.3f} R={vr:.3f}  (峰值 {max(abs(vl),abs(vr)):.3f})")
    cl, cr = dd.clamp_wheels(vl, vr)
    print(f"  限幅: L={cl:.3f} R={cr:.3f}  (峰值 {max(abs(cl),abs(cr)):.3f})")
    ratio_before = vl / vr
    ratio_after = cl / cr
    ok = abs(ratio_before - ratio_after) < 1e-12 and \
        abs(max(abs(cl), abs(cr)) - dd.max_v) < 1e-12
    print(f"  左右比例保持 & 峰值=上限  {'通过' if ok else '失败'}")
    assert ok

    # ---- 4. 原地旋转 ----
    print("\n[4] 原地旋转：反向轮速应产生纯转向")
    vl, vr = dd.inverse(0.0, 2.0)
    v, w = dd.forward(vl, vr)
    print(f"  L={vl:.3f} R={vr:.3f} -> v={v:.6f} omega={w:.3f}")
    ok = abs(v) < 1e-12 and abs(w - 2.0) < 1e-12
    print(f"  线速度为0、角速度正确  {'通过' if ok else '失败'}")
    assert ok

    # ---- 5. PID 阶跃响应 ----
    print("\n[5] PID 阶跃响应（速度环典型参数）")
    pid = PID(kp=0.8, ki=0.35, kd=0.0, out_min=-1.0, out_max=1.0)
    dt = 0.001
    x = 0.0
    for _ in range(2000):          # 2 秒
        u = pid.update(1.0, x, dt)
        x += u * dt                # 一阶惯性对象
    print(f"  2 秒后输出 = {x:.4f}（目标 1.0）")
    ok = abs(x - 1.0) < 0.05
    print(f"  跟踪误差 < 5%  {'通过' if ok else '失败'}")
    assert ok

    # ---- 6. 抗积分饱和 ----
    print("\n[6] 抗积分饱和")
    pid = PID(kp=0.1, ki=10.0, kd=0.0, out_min=-1.0, out_max=1.0,
              anti_windup=True)
    for _ in range(500):
        pid.update(100.0, 0.0, 0.001)     # 长时间大误差
    windup = abs(pid.integral)
    print(f"  持续大误差后积分项 = {windup:.4f}")

    pid2 = PID(kp=0.1, ki=10.0, kd=0.0, out_min=-1.0, out_max=1.0,
               anti_windup=False)
    for _ in range(500):
        pid2.update(100.0, 0.0, 0.001)
    windup2 = abs(pid2.integral)
    print(f"  关闭抗饱和后积分项 = {windup2:.4f}")
    ok = windup < windup2 * 0.1
    print(f"  抗饱和有效（积分小一个量级以上）  {'通过' if ok else '失败'}")
    assert ok

    # ---- 7. 航迹推算：走正方形回原点 ----
    print("\n[7] 航迹推算闭环精度（走 1m x 1m 方框）")
    odom = Odometry(dd, use_gyro=False)
    side = 1.0
    dt = 0.01
    # 直行用轮速；转向用反向轮速（原地旋转）
    # 速度取 0.5 m/s，转向 1.0 rad/s，方便按步数换算
    for _ in range(4):
        # --- 直行 side 米 ---
        n_steps = int(side / (0.5 * dt))
        for _ in range(n_steps):
            dist = 0.5 * dt
            ticks = dist / dd.m_per_tick
            odom.update(ticks, ticks, dt, 0.0)

        # --- 原地左转 90 度 ---
        omega = 1.0
        n_rot = int((math.pi / 2) / (omega * dt))
        for _ in range(n_rot):
            _, vr_ = dd.inverse(0.0, omega)
            dtheta = omega * dt
            # 每轮走过的弧长
            dl = (0.0 - omega * dd.B / 2.0) * dt
            dr = (0.0 + omega * dd.B / 2.0) * dt
            odom.update(dl / dd.m_per_tick, dr / dd.m_per_tick, dt, omega)

    x, y, th = odom.pose()
    err_mm = math.hypot(x, y) * 1000
    print(f"  终点 = ({x*1000:.1f}, {y*1000:.1f}) mm, "
          f"朝向 = {math.degrees(th):.2f} 度")
    print(f"  闭合误差 = {err_mm:.1f} mm")
    ok = err_mm < 50.0 and abs(math.degrees(th)) < 2.0
    print(f"  闭合误差 < 50mm 且朝向回归  {'通过' if ok else '失败'}")
    assert ok, f"闭合误差 {err_mm:.1f}mm 过大（理想输入下应接近 0）"

    # ---- 8. 角度环绕 ----
    print("\n[8] 角度归一化")
    cases = [(0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi),
             (3 * math.pi, math.pi), (-3 * math.pi, math.pi),
             (math.pi + 0.1, -math.pi + 0.1)]
    ok = True
    for inp, exp in cases:
        got = wrap_pi(inp)
        if abs(got - exp) > 1e-9:
            ok = False
            print(f"    wrap_pi({inp:.3f}) = {got:.3f}, 期望 {exp:.3f}  [X]")
    print(f"  {'通过' if ok else '失败'}")
    assert ok

    # ---- 9. 轮径标定 ----
    print("\n[9] 有效轮径标定")
    # 2000 计数/转、实测走 3.00m 对应 14700 计数
    #   -> 7.35 转，每转 0.4082m，反算直径 = 0.4082/pi = 130mm ... 直径 130mm
    # 取与 φ65 轮相符的场景：3.00m 走 7.35 转
    nominal = 0.0325                      # 标称半径 32.5mm（φ65 轮）
    revs_for_3m = 3.00 / (2 * math.pi * nominal)
    ticks_measured = int(revs_for_3m * dd.ticks)
    eff = calibrate_wheel_radius(3.00, ticks_measured, nominal, dd.ticks)
    print(f"  标称半径     = {nominal*1000:.2f} mm")
    print(f"  实测 {3.00:.2f} m 对应 {ticks_measured} 计数")
    print(f"  反算有效半径 = {eff*1000:.2f} mm "
          f"（应与标称接近，因为本例用的是标称值反推）")
    # 用有效轮径重算里程，应还原 3.00m
    dd2 = DiffDrive(eff, dd.B, dd.ticks)
    got = dd2.ticks_to_distance(ticks_measured)
    print(f"  用有效轮径反算里程 = {got:.4f} m  "
          f"{'通过' if abs(got - 3.0) < 1e-9 else '失败'}")
    assert abs(got - 3.0) < 1e-9

    # 再验证一个"真实轮胎压缩"场景：标称 32.5mm 但实际有效 32.0mm
    print("\n  模拟轮胎压缩场景（标称 32.5mm，实际有效 32.0mm）:")
    true_r = 0.0320
    ticks_true = int(3.00 / (2 * math.pi * true_r) * dd.ticks)
    eff2 = calibrate_wheel_radius(3.00, ticks_true, nominal, dd.ticks)
    print(f"    实际走 3.00m 时编码器计数 = {ticks_true}")
    print(f"    反算有效半径 = {eff2*1000:.2f} mm（真值 32.00）"
          f"  {'通过' if abs(eff2 - true_r) < 1e-6 else '失败'}")
    dd3 = DiffDrive(eff2, dd.B, dd.ticks)
    dist = dd3.ticks_to_distance(ticks_true)
    print(f"    用有效半径重算里程 = {dist:.4f} m  "
          f"{'通过' if abs(dist - 3.0) < 1e-9 else '失败'}")
    assert abs(dist - 3.0) < 1e-9

    # ---- 10. 运动学参数核算 ----
    print("\n[10] 本车参数核算（轮径 65mm，轮距 150mm）")
    print(f"  轮周长       = {circ*1000:.2f} mm")
    print(f"  0.8 m/s 对应 = {0.8/circ*60:.1f} rpm")
    print(f"  原地转向 omega=3 rad/s 时，轮速差 = "
          f"{3.0*0.15:.3f} m/s（每轮 ±{3.0*0.15/2:.3f}）")

    print("\n" + "=" * 78)
    print("全部自测通过。")
    print("=" * 78)


if __name__ == "__main__":
    selftest()
