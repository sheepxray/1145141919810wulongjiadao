#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机几何校核工具
================

用途：在**采购相机之前**用数值确认几个硬约束，避免买错镜头。

本脚本回答三个问题：

  1. 前视相机要装在什么高度/俯角，才能同时看到远处挡板的**上边缘和下边缘**？
     （看得到才能用挡板做定位基准）

  2. 挡板上下边缘在图像里只差几个像素？由此推出的测距精度是多少？
     （决定"视觉测墙定位"在哪些距离段可用）

  3. 下视相机能不能看清棋子上的汉字？
     （决定是否需要专用的读字相机，以及装在什么俯角）

坐标与符号约定
--------------
  h    : 相机光心离地高度 (m)
  theta: 俯仰角，**正值为向下俯视** (deg)。theta=0 表示光轴水平。
  d    : 目标点到相机光心的**水平**距离 (m)
  z    : 目标点离地高度 (m)；地面 z=0，挡板顶 z=H
  dep  : 俯角(depression)，目标点相对水平线的下倾角 = atan((h-z)/d)

  像素坐标 y 从上往下增大：
      y = cy + f_px * tan(dep - theta)

  由此：
      画幅上边缘 (y=0)     对应 dep = theta - half_vfov
      画幅下边缘 (y=res_v) 对应 dep = theta + half_vfov

  注意：本工具使用**完整投影式**，而不是近似式 d = H*f/dy。
  近似式只在 theta=0 时精确（详见 docs/05）。
"""

import math
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


# --------------------------------------------------------------------------
# 相机模型
# --------------------------------------------------------------------------

class Camera:
    """一个针孔相机 + 镜头的组合。"""

    def __init__(self, name, res_h, res_v, pixel_um, lens_mm):
        self.name = name
        self.res_h = res_h          # 水平分辨率 (px)
        self.res_v = res_v          # 垂直分辨率 (px)
        self.pixel_mm = pixel_um / 1000.0
        self.lens_mm = lens_mm
        # 像素焦距：把物理焦距换算成"多少像素"
        self.f_px = lens_mm / self.pixel_mm
        self.sensor_h = res_h * self.pixel_mm
        self.sensor_v = res_v * self.pixel_mm

    @property
    def half_vfov(self):
        """垂直半视场角 (deg)"""
        return math.degrees(math.atan(self.res_v / 2.0 / self.f_px))

    @property
    def vfov(self):
        return 2.0 * self.half_vfov

    @property
    def hfov(self):
        return 2.0 * math.degrees(math.atan(self.res_h / 2.0 / self.f_px))

    def pixel_y(self, dep_deg, theta_deg):
        """给定俯角和相机俯仰角，返回像素行坐标。"""
        dep = math.radians(dep_deg)
        th = math.radians(theta_deg)
        return self.res_v / 2.0 + self.f_px * math.tan(dep - th)

    def __str__(self):
        return (f"{self.name}: {self.res_h}x{self.res_v}, "
                f"pixel={self.pixel_mm*1000:.1f}um, lens={self.lens_mm}mm, "
                f"f={self.f_px:.0f}px, VFOV={self.vfov:.1f}deg, HFOV={self.hfov:.1f}deg")


# --------------------------------------------------------------------------
# 问题 1 & 2：挡板可见性与测距精度
# --------------------------------------------------------------------------

def depression_to_wall(h, d, wall_h=0.08):
    """返回 (挡板顶俯角, 挡板底俯角)，单位 deg。"""
    dep_top = math.degrees(math.atan((h - wall_h) / d))   # 挡板顶(80mm高)
    dep_bot = math.degrees(math.atan(h / d))              # 挡板底(地面)
    return dep_top, dep_bot


def wall_edge_pixels(cam, h, d, theta, wall_h=0.08):
    """挡板上下边缘在图像中的像素间距。返回 None 表示有边缘不在画幅内。"""
    dep_top, dep_bot = depression_to_wall(h, d, wall_h)
    y_top = cam.pixel_y(dep_top, theta)
    y_bot = cam.pixel_y(dep_bot, theta)
    if y_top < 0 or y_bot > cam.res_v:
        return None
    return y_bot - y_top


def wall_visible_range(cam, h, theta, wall_h=0.08):
    """
    挡板在该安装下可见的水平距离范围。

    几何事实：距离 d 越大，挡板越靠近地平线（俯角趋近 0）；
    d 越小，挡板越落在画幅**下方**之外。

      远端限制 —— 挡板顶必须落在画幅上边缘之下：
          dep_top(inf) >= theta - half_vfov
          即 atan((h-wall_h)/inf) = 0 >= theta - half_vfov
          → theta <= half_vfov + dep_top(inf)
          满足则远至无穷远都可见；不满足则挡板顶被画幅上边缘切掉。

      近端限制 —— 挡板底(地面)必须落在画幅下边缘之上：
          dep_bot(d) <= theta + half_vfov
          → d >= h / tan(theta + half_vfov)

    返回 (d_min, d_max)，d_max 可能为 inf。d_min == inf 表示整个挡板都看不到。
    """
    half = cam.half_vfov
    # 远端：地平线是否在画幅上边缘之下
    dep_top_inf = math.degrees(math.atan((h - wall_h) / 1e6))   # ≈ 0
    if dep_top_inf < theta - half:
        return (float("inf"), 0.0)          # 挡板顶永远在画幅上方，远处挡板不可见

    # 近端
    dep_max = theta + half
    if dep_max >= 90.0:
        d_min = 0.0
    elif dep_max <= 0.0:
        return (float("inf"), 0.0)
    else:
        d_min = h / math.tan(math.radians(dep_max))

    # d_max: 若远端条件满足，理论上无穷远可见；
    # 但挡板顶的像素行会趋近某个正值，只要 >= 0 就一直可见。
    # 这里给出"挡板顶恰好落到画幅上边缘"的距离（若存在）。
    #   y_top(d) = res_v/2 + f*tan(dep_top(d) - theta) = 0
    # 由于 dep_top 随 d 单调递减至 0，y_top 单调递减至 res_v/2 + f*tan(-theta)。
    # 只要该极限 >= 0 就没有有限上限。
    y_top_inf = cam.res_v / 2.0 + cam.f_px * math.tan(math.radians(-theta))
    d_max = float("inf") if y_top_inf >= 0.0 else d_min
    return (d_min, d_max)


def max_wall_range(cam, h, theta, wall_h=0.08):
    """兼容保留：返回能看清挡板的最远距离（inf 表示不受限）。"""
    return wall_visible_range(cam, h, theta, wall_h)[1]


def solve_range_from_pixels(cam, h, theta, dy_px, wall_h=0.08):
    """
    已知挡板上下边缘的像素间距 dy，反解水平距离 d。

    对完整投影式做二分求解（而不是用 d = H*f/dy 的近似）。
    像素间距 dy 随 d 单调递减，因此二分是安全的。
    """
    lo, hi = 0.02, 60.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        dep_top = math.degrees(math.atan((h - wall_h) / mid))
        dep_bot = math.degrees(math.atan(h / mid))
        y_top = cam.pixel_y(dep_top, theta)
        y_bot = cam.pixel_y(dep_bot, theta)
        cur = y_bot - y_top
        if cur > dy_px:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def range_uncertainty(cam, h, d, theta, edge_noise_px=2.0, wall_h=0.08):
    """
    挡板测距的距离不确定度。

    两条边缘各带 edge_noise_px 的定位噪声，总噪声 sigma_dy = sqrt(2)*edge_noise_px。
    再通过 d(dy)/dy 的局部斜率把像素噪声传播到距离：
        sigma_d = |dd/ddy| * sigma_dy
    斜率用数值差分求得（避免解析式的符号错误）。
    """
    dy = wall_edge_pixels(cam, h, d, theta, wall_h)
    if dy is None or dy < 1e-6:
        return None, None

    d_est = solve_range_from_pixels(cam, h, theta, dy, wall_h)

    # 数值差分求斜率，步长取 1px
    d_plus = solve_range_from_pixels(cam, h, theta, dy + 1.0, wall_h)
    d_minus = solve_range_from_pixels(cam, h, theta, max(dy - 1.0, 1e-6), wall_h)
    slope = abs(d_plus - d_minus) / 2.0          # 每像素对应的距离变化 (m/px)

    sigma_dy = math.sqrt(2.0) * edge_noise_px
    return d_est, slope * sigma_dy


# --------------------------------------------------------------------------
# 问题 3：棋子汉字可读性
# --------------------------------------------------------------------------

def visible_ground_band(cam, h, theta):
    """
    给定安装 (h, theta)，返回相机在**地面**上能看到的水平距离区间 (d_min, d_max)。

    可见条件：该点的俯角 dep 落在 [theta - half_vfov, theta + half_vfov] 内，
    即 d = h / tan(dep)。

    注意 d_max 可能为 inf（当 theta - half_vfov <= 0，画幅上边缘达到或越过水平线）。
    """
    half = cam.half_vfov
    dep_min = theta - half          # 对应最远处
    dep_max = theta + half          # 对应最近处

    if dep_max <= 0.0:
        return None                 # 画幅整个朝上，看不到地面
    d_near = h / math.tan(math.radians(dep_max))
    if dep_min <= 0.0:
        d_far = float("inf")        # 上边缘越过水平线，地面延伸到无穷远
    else:
        d_far = h / math.tan(math.radians(dep_min))
    return (max(d_near, 0.0), d_far)


def char_pixels(cam, h, d, theta, char_mm=20.0):
    """
    估计地面上一个 char_mm 大小的方块（棋子汉字）在图像中占多少像素。

    返回 (径向像素, 横向像素)，或 None 表示该点不在视场内。

    实现要点：直接对**近端/远端**分别求像素行坐标后相减，
    而不是对 tan 的差值做运算 —— 后者在 theta 较大时会出现
    "两个大数相减"的灾难性抵消（实测 theta=90 时会给出 266px 的垃圾值）。
    """
    # 该点必须真的在视场内，否则算出来的数字没有意义
    band = visible_ground_band(cam, h, theta)
    if band is None:
        return None
    d_min, d_max = band
    half_s = char_mm / 2000.0       # 半边长，单位 m
    if d < d_min - half_s or d > d_max + half_s:
        return None

    s = char_mm / 1000.0
    d_near = max(d - s / 2.0, 1e-4)
    d_far = d + s / 2.0

    # 径向：近端与远端的像素行坐标之差（近端在下、远端在上）
    dep_n = math.degrees(math.atan(h / d_near))
    dep_f = math.degrees(math.atan(h / d_far))
    y_near = cam.pixel_y(dep_n, theta)
    y_far = cam.pixel_y(dep_f, theta)
    px_radial = abs(y_near - y_far)

    # 横向：以斜距计算角宽
    R = math.sqrt(h * h + d * d)
    px_lateral = cam.f_px * s / R

    return px_radial, px_lateral


def foreshortening(h, d):
    """
    地面目标在当前视线下的透视压缩比。

    光线与地面法线的夹角 = 该点的俯角 atan(h/d)，
    地面特征在这一方向上的投影长度正比于 sin(俯角)。
    = 1.0 表示正对（垂直下视该点），越小表示被压得越扁。
    """
    return math.sin(math.atan2(h, d))


# --------------------------------------------------------------------------
# 问题 4：推板开口的贴墙容错
# --------------------------------------------------------------------------

def wall_hug_window(car_half_width_mm, jaw_half_mm, piece_d_mm, wall_clearance_mm=5.0):
    """
    棋子贴侧墙时，车体横向可用的对准窗口有多宽？

    约束：
      (a) 推板要罩住棋子   : car_y - jaw_half <= piece_center_to_wall
      (b) 车体不能撞墙     : car_y >= car_half_width + wall_clearance

    返回 (窗口宽度 mm, 车体最小中心偏移, 车体最大中心偏移)。
    """
    piece_center = piece_d_mm / 2.0
    car_y_max = piece_center + jaw_half_mm
    car_y_min = car_half_width_mm + wall_clearance_mm
    return car_y_max - car_y_min, car_y_min, car_y_max


def wall_hug_window_overhang(car_half_width_mm, jaw_half_mm, piece_d_mm,
                             wall_clearance_mm=5.0):
    """
    用**悬伸量**表达贴墙容错，可以看出真正的设计杠杆。

    设推板半开口 jaw_half = 车体半宽 car_half + 悬伸量 overhang，
    代入 wall_hug_window 的 (a)(b)：

        window = (piece_r + car_half + overhang) - (car_half + clearance)
               = piece_r + overhang - clearance

    车体半宽 car_half **被完全消掉**。结论：
      - 单纯加宽车体（把 200mm 用满在车体上）= 无效，窗口不变
      - 让推板比车体更宽（悬伸）= 窗口直接增加 overhang 那么多

    受"发车外廓 <= 200mm"约束，最优策略是：
      **车体做窄，推板做满 200mm**，把差额全部变成悬伸量。
    """
    piece_r = piece_d_mm / 2.0
    overhang = jaw_half_mm - car_half_width_mm
    window = piece_r + overhang - wall_clearance_mm
    car_y_min = car_half_width_mm + wall_clearance_mm
    car_y_max = piece_r + jaw_half_mm
    return window, car_y_min, car_y_max


# --------------------------------------------------------------------------
# 主程序：对候选方案做校核
# --------------------------------------------------------------------------

H_CAM = 0.15        # 相机高度 (m)
WALL_H = 0.08       # 挡板高度 (m)

CANDIDATES = [
    Camera("OV5640 + 1.8mm(超广角)", 2592, 1944, 1.4, 1.8),
    Camera("OV5640 + 2.8mm",         2592, 1944, 1.4, 2.8),
    Camera("OV5640 + 3.6mm",         2592, 1944, 1.4, 3.6),
    Camera("OV2640 + 2.8mm",         1600, 1200, 2.2, 2.8),
    Camera("OV9281 全局快门 + 2.8mm", 1280, 800, 3.0, 2.8),
]


def section(title):
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


def main():
    section("候选相机参数")
    for c in CANDIDATES:
        print("  " + str(c))

    # ---------------- 前视相机：能否看到远处挡板 ----------------
    section(f"前视相机：挡板可见性 (安装高度 h={H_CAM*1000:.0f}mm, 挡板 {WALL_H*1000:.0f}mm)")
    print("  几何事实：d 越小挡板越落到画幅**下方**之外；d 越大挡板越靠近地平线。")
    print("  因此约束是**最近可见距离 d_min**，而不是最远距离。")
    print()
    print(f"  {'相机':<30} {'half_vfov':>10} {'theta=25时d_min':>16} {'3m处挡板':>12}")
    print("  " + "-" * 74)

    for c in CANDIDATES:
        theta = 25.0
        d_min, d_max = wall_visible_range(c, H_CAM, theta, WALL_H)
        if d_min == float("inf"):
            band = "完全不可见"
        elif d_min <= 3.0:
            band = "可见"
        else:
            band = f"需>{d_min:.1f}m"
        dmin_str = "inf" if d_min == float("inf") else f"{d_min:.2f}m"
        print(f"  {c.name:<30} {c.half_vfov:>9.1f}d {dmin_str:>16} {band:>12}")

    print()
    print("  >> 解读：俯角 25 度时，垂直半视场小的镜头要退到很远处才能看到挡板底。")
    print("     这正是'俯视看棋子的相机看不到近处挡板'的量化形式。")

    # ---------------- 反推：各相机在 3m 处看挡板所允许的最大俯角 ----------------
    section("反推：要让'3m 处挡板'完整可见，各相机允许的最大俯角")
    _, dep_bot_3m = depression_to_wall(H_CAM, 3.0, WALL_H)
    dep_top_3m, _ = depression_to_wall(H_CAM, 3.0, WALL_H)
    print(f"  3m 处：挡板顶俯角 = {dep_top_3m:.2f}d，挡板底俯角 = {dep_bot_3m:.2f}d")
    print(f"  需求： theta - half_vfov <= {dep_top_3m:.2f}  (挡板顶在画幅内)")
    print(f"         theta + half_vfov >= {dep_bot_3m:.2f}  (挡板底在画幅内)")
    print()
    print(f"  {'相机':<30} {'half_vfov':>10} {'theta_max':>11} {'theta_min':>11} {'可行区间':>10}")
    print("  " + "-" * 78)
    for c in CANDIDATES:
        theta_max = c.half_vfov + dep_top_3m
        theta_min = dep_bot_3m - c.half_vfov
        span = theta_max - theta_min
        verdict = f"{span:.1f}d" if span > 0 else "无解"
        print(f"  {c.name:<30} {c.half_vfov:>9.1f}d {theta_max:>10.1f}d "
              f"{theta_min:>10.1f}d {verdict:>10}")

    print()
    print("  >> theta_min 为负说明挡板底总是可见（画幅上边缘越过水平线）。")
    print("     各相机都有可行的俯角区间，但注意区间越窄，安装公差越紧。")

    # ---------------- 测距精度 ----------------
    section("视觉测墙：像素间距与测距不确定度 (以 OV5640+2.8mm 为例, theta=15deg)")
    cam = CANDIDATES[1]
    theta = 15.0
    print(f"  相机: {cam.name},  安装: h={H_CAM*1000:.0f}mm, theta={theta}deg")
    print(f"  边缘定位噪声假设: 2.0 px")
    print()
    print(f"  {'距离 d(m)':>10} {'像素间距 dy':>12} {'距离估计(m)':>12} {'不确定度(mm)':>13} {'可用性':>8}")
    print("  " + "-" * 62)
    for d in [0.3, 0.5, 0.8, 1.0, 1.5, 2.0, 2.5, 3.0]:
        d_est, sig = range_uncertainty(cam, H_CAM, d, theta, 2.0, WALL_H)
        if d_est is None:
            print(f"  {d:>10.1f} {'不可见':>12} {'-':>12} {'-':>13} {'不可用':>8}")
            continue
        usable = "优" if sig < 0.02 else ("可用" if sig < 0.08 else "差")
        print(f"  {d:>10.1f} {wall_edge_pixels(cam, H_CAM, d, theta, WALL_H):>12.1f} "
              f"{d_est:>12.3f} {sig*1000:>13.1f} {usable:>8}")

    print()
    print("  >> 解读：像素间距随距离迅速塌缩，测距精度按 d^2 恶化。")
    print("     这就是为什么'全场挡板 PnP'不可行，只能做近场测距。")

    # ---------------- 验证 docs/05 中的近似式误差 ----------------
    section("关键校核：近似式 d = H*f/dy 的误差 (即'俯仰角无关'假设是否成立)")
    cam = CANDIDATES[1]
    print(f"  相机: {cam.name}")
    print(f"  对比：(1) 完整投影数值反解   (2) 近似式 d = H*f/dy")
    print(f"  若两者在所有情形下都相等，则'俯仰角无关'成立；否则不成立。")
    print()
    print(f"  {'h(mm)':>7} {'theta':>7} {'真值d(m)':>10} {'完整反解(m)':>12} "
          f"{'近似式(m)':>11} {'近似式误差':>11}")
    print("  " + "-" * 66)
    worst_err = 0.0
    for h in [0.15, 0.25]:
        for theta in [0.0, 10.0, 20.0, 30.0]:
            for d in [0.5, 1.5]:
                dy = wall_edge_pixels(cam, h, d, theta, WALL_H)
                if dy is None:
                    continue
                d_full = solve_range_from_pixels(cam, h, theta, dy, WALL_H)
                d_approx = WALL_H * cam.f_px / dy
                err = (d_approx - d) / d * 100.0
                worst_err = max(worst_err, abs(err))
                flag = "  <--" if abs(err) > 3.0 else ""
                print(f"  {h*1000:>7.0f} {theta:>6.1f}d {d:>10.2f} {d_full:>12.3f} "
                      f"{d_approx:>11.3f} {err:>10.1f}%{flag}")

    print()
    print(f"  >> 最大近似式误差 = {worst_err:.1f}%")
    print(f"     theta=0 时误差为 0（此时 H*f/dy 精确）；theta 越大误差越大，")
    print(f"     且相机越高误差越大。所以该近似式**不能**无条件使用。")
    print(f"     本仓库的定位实现一律使用完整投影反解（见 solve_range_from_pixels）。")

    # ---------------- 下视相机：汉字可读性 ----------------
    section("下视相机：汉字像素高度 vs 安装俯角 (OV5640+2.8mm, h=150mm, 汉字 20mm)")
    cam = CANDIDATES[1]
    print("  可读性经验阈值：汉字 >= 20px 可读，>= 30px 舒适，< 15px 基本不可读")
    print("  注意：只有落在**该安装的可见地面区间**内的距离才有意义（否则算出的是垃圾值）")
    print()
    print(f"  {'俯角':>6} {'可见地面区间(mm)':>18} {'读取距离(mm)':>13} {'径向px':>8} "
          f"{'横向px':>8} {'压缩比':>7} {'可读性':>8}")
    print("  " + "-" * 82)
    for theta in [10.0, 20.0, 30.0, 45.0, 60.0, 75.0, 89.0]:
        band = visible_ground_band(cam, H_CAM, theta)
        if band is None:
            print(f"  {theta:>5.0f}d {'看不到地面':>18}")
            continue
        d_min, d_max = band
        dmax_str = "inf" if d_max == float("inf") else f"{d_max*1000:.0f}"
        band_str = f"{d_min*1000:.0f}~{dmax_str}"

        for d_mm in [200.0, 300.0]:
            res = char_pixels(cam, H_CAM, d_mm / 1000.0, theta, 20.0)
            if res is None:
                print(f"  {theta:>5.0f}d {band_str:>18} {d_mm:>13.0f} {'不在视场':>8}")
                continue
            pr, pl = res
            fs = foreshortening(H_CAM, d_mm / 1000.0)
            worst = min(pr, pl)
            if worst >= 30:
                verdict = "舒适"
            elif worst >= 20:
                verdict = "可读"
            elif worst >= 15:
                verdict = "勉强"
            else:
                verdict = "不可读"
            print(f"  {theta:>5.0f}d {band_str:>18} {d_mm:>13.0f} {pr:>8.1f} "
                  f"{pl:>8.1f} {fs:>7.2f} {verdict:>8}")

    print()
    print("  >> 解读：俯角越小（越平视），地面被压得越扁，径向像素崩塌。")
    print("     读字相机需要较大的俯角（接近垂直下视）以保证汉字不被压扁。")
    print("     而大俯角使可见地面区间急剧收缩 —— 这是读字相机的固有取舍：")
    print("     必须就近读字（d 小），不能指望它看远处。")

    # ---------------- 单相机折中方案评估 ----------------
    section("评估：能否用一台相机同时兼顾'看远墙'和'读汉字'？")
    print("  重要更正：地面透视压缩只取决于 h/d 的比值，**与相机俯角 theta 无关**。")
    print("  theta 只决定该点落在画面的哪一行。因此'俯角大才看得清字'的说法不成立。")
    print()
    print("  真正的冲突在于**视场分配**：")
    print("    (A) 看 3m 处挡板 -> 画幅上边缘必须接近水平：theta <= half_vfov + 1.34d")
    print("    (B) 近处读字 -> 字不能压在画面边缘，且推板前方必须是可见地面")
    print()
    print("  下面加**边缘余量**要求（字心至少离画面上下边缘 15% 画幅高）后重新判定：")
    print()

    print()
    print(f"  {'相机':<30} {'theta上限':>10} {'theta下限':>10} {'可行区间':>10} {'结论':>10}")
    print("  " + "-" * 78)
    for c in CANDIDATES:
        dep_top_3m, _ = depression_to_wall(H_CAM, 3.0, WALL_H)
        theta_hi = c.half_vfov + dep_top_3m          # (A)

        theta_lo = None
        for t in range(3, 90):
            res = char_pixels(c, H_CAM, 0.25, float(t), 20.0)
            if res is None or min(res) < 20.0:
                continue
            # 字心必须离画面上下边缘有 15% 画幅高的余量
            dep_c = math.degrees(math.atan(H_CAM / 0.25))
            y_c = c.pixel_y(dep_c, float(t))
            margin_px = 0.15 * c.res_v
            if margin_px <= y_c <= c.res_v - margin_px:
                theta_lo = float(t)
                break

        if theta_lo is None:
            print(f"  {c.name:<30} {theta_hi:>9.1f}d {'无解':>10} {'-':>10} {'只能看墙':>10}")
        elif theta_lo <= theta_hi:
            span = theta_hi - theta_lo
            verdict = "可行" if span >= 10 else "勉强"
            print(f"  {c.name:<30} {theta_hi:>9.1f}d {theta_lo:>9.1f}d "
                  f"{span:>9.1f}d {verdict:>10}")
        else:
            print(f"  {c.name:<30} {theta_hi:>9.1f}d {theta_lo:>9.1f}d "
                  f"{'无交集':>10} {'需双相机':>10}")

    print()
    print("  >> 判读：若某相机显示'可行'且区间够宽，则单相机在几何上成立，")
    print("     此时可选单相机方案，省一个相机和一路 CSI。")
    print("     但要注意单相机方案的三个工程代价：")
    print("       1. 需要广角镜头，画面边缘畸变最大，而挡板恰好落在边缘")
    print("       2. 远场(挡板, 亮)与近场(地面棋子, 暗)曝光差异大，难以兼顾")
    print("       3. 俯角被夹在窄区间内，安装公差要求高")
    print("     因此本仓库默认推荐双相机，单相机作为预算极紧时的备选。")

    # ---------------- 推板贴墙容错 ----------------
    section("推板：棋子贴侧墙时的对准窗口")
    print("  棋子直径 45mm，离墙间隙 5mm，发车外廓上限 200mm")
    print()
    print("  [方案 A] 推板不超出车体（车体多宽，推板就多宽）")
    print()
    print(f"  {'推板开口(mm)':>14} {'车半宽(mm)':>12} {'对准窗口(mm)':>14} {'评价':>10}")
    print("  " + "-" * 54)
    for jaw in [150, 170, 180, 190, 195, 200]:
        win, ymin, ymax = wall_hug_window(jaw / 2.0, jaw / 2.0, 45.0, 5.0)
        verdict = ("不可能" if win < 0 else
                   "极难" if win < 15 else
                   "困难" if win < 30 else "可行")
        print(f"  {jaw:>14} {jaw/2.0:>12.1f} {win:>14.1f} {verdict:>10}")

    print()
    print("  [方案 B] 车体做窄、推板向两侧悬伸（推板宽 195mm 固定）")
    print()
    print(f"  {'车体宽(mm)':>12} {'悬伸量(mm)':>12} {'对准窗口(mm)':>14} {'评价':>10}")
    print("  " + "-" * 54)
    for car_w in [130, 140, 150, 160, 170, 180]:
        win, ymin, ymax = wall_hug_window_overhang(
            car_w / 2.0, 195.0 / 2.0, 45.0, 5.0)
        verdict = ("不可能" if win < 0 else
                   "极难" if win < 15 else
                   "困难" if win < 30 else "可行")
        print(f"  {car_w:>12} {(195.0-car_w)/2.0:>12.1f} {win:>14.1f} {verdict:>10}")

    print()
    print("  >> 关键结论：对准窗口 = 棋子半径 + 悬伸量 - 离墙间隙，**与车宽无关**。")
    print("     推导：window = (r + car_half + overhang) - (car_half + clearance)")
    print("                  = r + overhang - clearance        <- car_half 被消掉")
    print()
    print("     看方案 A 那一列：无论开口 150 还是 200mm，窗口恒为 17.5mm")
    print("     （= 棋子半径 22.5 - 间隙 5）。这说明**把车体做宽完全无效**。")
    print()
    print("     提高贴墙容错的正确做法是**让推板比车体宽**：")
    print("       车体 150mm + 推板 195mm -> 悬伸 22.5mm -> 窗口 40.0mm（翻一倍多）")
    print("       车体 140mm + 推板 195mm -> 悬伸 27.5mm -> 窗口 45.0mm")
    print()
    print("     受 200mm 外廓限制，最优是：车体做窄(140~160mm)，推板做满 195~200mm。")
    print("     注意：麦克纳姆轮不改变车宽，对这个问题毫无帮助。")

    print("\n" + "=" * 78)
    print("校核完成。把上面的结论回填到 docs/02 (机械) 与 docs/05 (定位)。")
    print("=" * 78)


if __name__ == "__main__":
    main()
