#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
帧率与实时性预算工具
====================

用途：回答一个问题 —— **这套设计能不能保证 30 FPS？瓶颈在哪？**

本工具做四件事：
  1. 列出传感器各模式的**帧率上限**（这是最容易被忽略的第一道卡口）
  2. 计算**所需的最低分辨率**（棋子能不能被检测到、汉字能不能被读出来）
  3. 用可编辑的**帧预算表**算出实际可达帧率与余量
  4. 分析**延迟**（对控制系统而言，延迟比帧率更重要）

所有时间数值都标注了来源：`[实测]` 或 `[预估]`。
**预估值必须用 firmware/k230/profiler.py 在真机上实测替换。**

运行：
    python tools/framerate.py
"""

import math
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


# ==========================================================================
# 设计参数
# ==========================================================================

TARGET_FPS = 30.0
FRAME_PERIOD_MS = 1000.0 / TARGET_FPS      # 33.33 ms

# 工程经验：单帧处理耗时应 <= 帧周期的一半，否则抖动会吃掉余量
SAFE_UTILIZATION = 0.50

PIECE_DIAMETER_MM = 45.0
CHAR_SIZE_MM = 20.0

CAM_HEIGHT_MM = 150.0
TILT_FRONT_DEG = 15.0
TILT_DOWN_DEG = 60.0

LENS_MM = 2.8
PIXEL_UM = 1.4
FULL_W, FULL_H = 2592, 1944

SENSOR_W_MM = FULL_W * PIXEL_UM / 1000.0
F_PX_FULL = LENS_MM / (PIXEL_UM / 1000.0)
HFOV_FULL_DEG = 2 * math.degrees(
    math.atan(SENSOR_W_MM / (2 * LENS_MM)))


# ==========================================================================
# 传感器模式表
#   OV5640 各分辨率下的**硬件帧率上限**（以 datasheet 为准，此处为典型值）
# ==========================================================================

OV5640_MODES = [
    # (宽, 高, 最高帧率)
    (2592, 1944, 15),
    (2048, 1536, 15),
    (1920, 1080, 30),
    (1280, 960, 45),
    (1280, 720, 60),
    (640,  480, 90),
]


# ==========================================================================
# 几何计算
# ==========================================================================

def fov_width_mm(d_mm, hfov_deg):
    """给定距离处，视场的水平宽度（mm）"""
    return 2.0 * d_mm * math.tan(math.radians(hfov_deg / 2.0))


def object_px(size_mm, d_mm, hfov_deg, width_px):
    """目标在图像中的像素宽度（简化：正视）"""
    return size_mm * width_px / fov_width_mm(d_mm, hfov_deg)


def crop_hfov(width_px):
    """
    模式为**裁剪**时的水平视场角。

    裁剪时像素尺寸不变（1.4um），用到的传感器宽度变窄 -> 视场变窄。
    """
    w_mm = width_px * PIXEL_UM / 1000.0
    return 2 * math.degrees(math.atan(w_mm / (2 * LENS_MM)))


def scale_hfov(width_px):
    """
    模式为**缩放/合并**时的水平视场角。

    合并(binning)时等效像素变大，视场**不变**，只是分辨率降低。
    """
    return HFOV_FULL_DEG


def f_px_for(width_px, mode):
    """该模式下的像素焦距"""
    if mode == "crop":
        return F_PX_FULL                       # 像素尺寸不变
    else:                                      # scale / binning
        factor = width_px / FULL_W
        return F_PX_FULL * factor


def char_pixels(d_mm, tilt_deg, width_px, mode, char_mm=CHAR_SIZE_MM,
                h=CAM_HEIGHT_MM):
    """
    汉字在图像中的 (径向px, 横向px)。

    径向：地面上的字被透视压缩最严重的方向（沿视线方向）
    横向：基本不压缩

    用**像素行坐标直接相减**，避免 tan 差值在大角度下的灾难性抵消。
    """
    f = f_px_for(width_px, mode)
    res_h = width_px * 3 // 4                  # 4:3 假设，仅用于估算行坐标

    s = char_mm
    d_near = max(d_mm - s / 2.0, 1.0)
    d_far = d_mm + s / 2.0

    def row(dd):
        dep = math.degrees(math.atan(h / dd))
        return f * math.tan(math.radians(dep - tilt_deg))

    px_radial = abs(row(d_near) - row(d_far))
    R = math.hypot(h, d_mm)
    px_lateral = f * s / R
    return px_radial, px_lateral


# ==========================================================================
# 帧预算
# ==========================================================================

class Budget:
    """
    一帧的处理预算。

    每项 = (阶段名, 耗时ms, 是否在关键路径, 来源标记)
    """

    def __init__(self, name, stages):
        self.name = name
        self.stages = stages

    @property
    def total_ms(self):
        return sum(ms for _, ms, crit, _ in self.stages if crit)

    @property
    def fps(self):
        t = self.total_ms
        return 1000.0 / t if t > 0 else float("inf")

    def scale_worst(self, factor):
        """把所有预估阶段放大 factor 倍（敏感性分析用）"""
        return Budget(self.name, [
            (n, ms * factor if src == "[预估]" else ms, c, src)
            for n, ms, c, src in self.stages
        ])


# ---- 预估值：必须用真机 profiler 替换 ----

SEARCH_BUDGET = Budget("SEARCH 前视搜索（720p采集 -> 640x360处理）", [
    ("取帧（与处理并行，不计入关键路径）",  0.0, False, "[硬件]"),
    ("降采样/合并 到 640x360",              1.2, True,  "[预估]"),
    ("find_blobs 颜色阈值 + 连通域",        6.0, True,  "[预估]"),
    ("坐标变换 + 候选筛选",                 0.8, True,  "[预估]"),
    ("UART 打包发送",                       0.3, True,  "[预估]"),
])

SERVO_BUDGET = Budget("SERVO 下视伺服（720p采集 -> 320x160 ROI）", [
    ("取帧（与处理并行，不计入关键路径）",  0.0, False, "[硬件]"),
    ("ROI 拷贝 320x160",                    0.4, True,  "[预估]"),
    ("find_blobs / 质心 320x160",           2.0, True,  "[预估]"),
    ("UART 打包发送",                       0.2, True,  "[预估]"),
])

READ_BUDGET = Budget("READ 停车读字（无帧率要求）", [
    ("取帧",                                0.0, False, "[硬件]"),
    ("ROI 裁切（推板前方区域）",            0.5, True,  "[预估]"),
    ("圆形筛选 + 红黑颜色判定",             3.0, True,  "[预估]"),
    ("字符裁切 + 归一化 64x64",             2.0, True,  "[预估]"),
    ("CNN 推理（KPU）",                     8.0, True,  "[预估]"),
    ("UART 打包发送",                       0.3, True,  "[预估]"),
])


# ==========================================================================
# 输出辅助
# ==========================================================================

def section(title):
    print("\n" + "=" * 86)
    print(title)
    print("=" * 86)


def table(headers, rows, widths=None):
    if widths is None:
        widths = [max(len(str(h)), *(len(str(r[i])) for r in rows))
                  for i, h in enumerate(headers)]
    line = "  " + " ".join(str(h).rjust(w) for h, w in zip(headers, widths))
    print(line)
    print("  " + "-" * (sum(widths) + len(widths) - 1))
    for r in rows:
        print("  " + " ".join(str(c).rjust(w) for c, w in zip(r, widths)))


# ==========================================================================
# 主程序
# ==========================================================================

def main():
    print("=" * 86)
    print("帧率与实时性预算")
    print(f"目标帧率 = {TARGET_FPS:.0f} FPS  ->  帧周期 = {FRAME_PERIOD_MS:.2f} ms")
    print(f"工程安全线：单帧处理 <= 帧周期 x {SAFE_UTILIZATION:.0%} "
          f"= {FRAME_PERIOD_MS*SAFE_UTILIZATION:.2f} ms")
    print("=" * 86)

    # ---------------- 1. 传感器帧率上限 ----------------
    section("1. 第一道卡口：传感器本身的帧率上限（OV5640）")
    print(f"  镜头 {LENS_MM}mm，像素 {PIXEL_UM}um，全画幅 {FULL_W}x{FULL_H}")
    print(f"  全画幅视场 HFOV = {HFOV_FULL_DEG:.1f} 度")
    print()
    rows = []
    for w, h, fps in OV5640_MODES:
        ok30 = "可以" if fps >= 30 else "**不够**"
        ok60 = "可以" if fps >= 60 else "不够"
        rows.append([f"{w}x{h}", f"{fps}", ok30, ok60,
                     f"{w*h/1e6:.1f} MP"])
    table(["分辨率", "最高fps", "满足30fps?", "满足60fps?", "像素数"],
          rows)
    print()
    print("  >> 关键结论：**全分辨率 2592x1944 只有 15 fps，根本不满足 30 FPS。**")
    print("     这不是算力问题，是传感器读出带宽的硬限制。")
    print("     想上 30 FPS，必须把分辨率降到 **1080p 及以下**；")
    print("     想要余量，用 **720p（60 fps）**。")
    print()
    print("     好消息：本任务**根本不需要 5MP** —— 见第 2、3 节。")

    # ---------------- 2. 检测所需分辨率 ----------------
    section("2. 检测棋子需要多少分辨率？（决定搜索帧率上限）")
    print(f"  棋子直径 {PIECE_DIAMETER_MM:.0f}mm；经验阈值：")
    print("     >= 20 px 可靠检测 / 10-20 px 勉强 / < 10 px 不可靠")
    print()
    print("  crop(裁剪) 与 scale(缩放/合并) 的视场不同，分别计算：")
    print()
    rows = []
    for w, h, _ in [(1280, 720, 0), (1920, 1080, 0), (640, 480, 0)]:
        hf_c = crop_hfov(w)
        hf_s = scale_hfov(w)
        for d in [1000, 2000, 3000]:
            px_c = object_px(PIECE_DIAMETER_MM, d, hf_c, w)
            px_s = object_px(PIECE_DIAMETER_MM, d, hf_s, w)
            rows.append([f"{w}x{h}", f"{d/1000:.0f}m",
                         f"{hf_c:.1f}d", f"{px_c:.1f}",
                         f"{hf_s:.1f}d", f"{px_s:.1f}"])
    table(["分辨率", "距离", "crop视场", "crop棋子px",
           "scale视场", "scale棋子px"], rows)
    print()
    print("  >> 解读：")
    print("     - 720p + scale 模式：2m 处棋子 22px，3m 处 15px —— **够用**")
    print("     - 720p + crop 模式：2m 处 45px，但视场只有 35 度，看不宽")
    print("     - 搜索要的是**看宽** -> 用 **scale/合并** 模式")
    print("     - 5MP 完全没有必要：棋子再大也只是占用更多像素而已")

    # ---------------- 3. 读字所需分辨率 ----------------
    section("3. 读汉字需要多少分辨率？（车静止，不影响 30 FPS）")
    print(f"  汉字 {CHAR_SIZE_MM:.0f}mm，下视相机俯角 {TILT_DOWN_DEG:.0f} 度，"
          f"高度 {CAM_HEIGHT_MM:.0f}mm")
    print("  经验阈值：>= 20px 可读 / >= 30px 舒适")
    print()
    print("  ⚠ 注意：crop(裁剪)模式的像素焦距虽与裁剪宽度无关，")
    print("     但**视场会随之变窄**。窄到一定程度连棋子都装不下，")
    print("     此时'汉字像素数'再高也没有意义。下面同时校核视场。")
    print()
    read_d = 250.0
    rows = []
    for w in [2592, 1920, 1280, 640, 320]:
        for mode in ["crop", "scale"]:
            hf = crop_hfov(w) if mode == "crop" else scale_hfov(w)
            fov_mm = fov_width_mm(read_d, hf)
            # 视场能否覆盖一枚棋子（含 2 倍余量便于左右各留半个）
            covers = fov_mm >= PIECE_DIAMETER_MM * 2
            pr, pl = char_pixels(read_d, TILT_DOWN_DEG, w, mode)
            worst = min(pr, pl)
            verdict = ("舒适" if worst >= 30 else
                       "可读" if worst >= 20 else
                       "勉强" if worst >= 15 else "不可读")
            if not covers:
                verdict = "**视场不足**"
            rows.append([f"{w}px", mode, f"{hf:.1f}d", f"{fov_mm:.0f}",
                         f"{pr:.1f}", f"{pl:.1f}", verdict])
    table(["宽度", "模式", "视场", f"{read_d:.0f}mm处视场宽mm",
           "径向px", "横向px", "可读性"], rows)
    print()
    print("  >> 解读：")
    print("     - 关键不是'汉字多少像素'，而是**视场能不能罩住棋子**。")
    print("     - 320px crop 的汉字像素数最高（92.4），但视场只有 40mm，")
    print("       **连一枚 45mm 的棋子都装不下** —— 这是个陷阱。")
    print("     - 640px scale 仍可读（径向 22.8px），视场 65.9 度、宽 325mm，")
    print("       既装得下棋子又有足够采样 —— 是读字模式的合理下限。")
    print()
    print("     因为读字时**车是静止的**，可以慢慢处理，帧率无要求。")
    print("     所以读字完全不该占用 30 FPS 的预算。")

    # ---------------- 4. 推荐工作点 ----------------
    section("4. 推荐工作点")
    rows = [
        ["SEARCH", "前视", "720p (scale)", "60", "640x360",
         "30+", "检测 2m 内棋子"],
        ["SERVO",  "下视", "720p (scale)", "60", "320x160 ROI",
         "30+", "推入段保持居中"],
        ["READ",   "下视", "720p (crop)",  "30", "ROI 裁切",
         "无要求", "停车读字，可慢慢处理"],
        ["IDLE",   "-",    "关闭",          "-",  "-",
         "-",    "省电降温"],
    ]
    table(["模式", "相机", "采集模式", "采集fps", "处理分辨率",
           "处理fps", "说明"], rows)
    print()
    print("  >> 核心策略：**采集用高帧率，处理用低分辨率。**")
    print("     720p@60 采集给了 2 倍余量，处理端降采样后负担很轻。")

    # ---------------- 5. 帧预算 ----------------
    section("5. 帧预算")
    for b in [SEARCH_BUDGET, SERVO_BUDGET, READ_BUDGET]:
        print(f"\n  【{b.name}】")
        rows = [[n, f"{ms:.1f}", "是" if c else "否", src]
                for n, ms, c, src in b.stages]
        table(["阶段", "耗时ms", "关键路径", "来源"], rows)
        total = b.total_ms
        fps = b.fps
        util = total / FRAME_PERIOD_MS
        if "READ" in b.name:
            print(f"\n    关键路径合计 = {total:.1f} ms  "
                  f"（读字无帧率要求，此处仅供参考）")
        else:
            status = ("达标" if fps >= TARGET_FPS else "**不达标**")
            margin = FRAME_PERIOD_MS - total
            print(f"\n    关键路径合计 = {total:.1f} ms")
            print(f"    可达帧率     = {fps:.1f} FPS   [{status}]")
            print(f"    相对 30 FPS 余量 = {margin:.1f} ms "
                  f"(利用率 {util:.0%})")
            if util > SAFE_UTILIZATION:
                print(f"    ⚠ 利用率超过 {SAFE_UTILIZATION:.0%} 安全线，"
                      f"抖动可能吃掉余量，建议优化")
            else:
                print(f"    ✓ 利用率在 {SAFE_UTILIZATION:.0%} 安全线内")
        print("    注意：以上时间为**预估值**，必须用真机实测替换。")

    # ---------------- 6. 敏感性分析 ----------------
    section("6. 敏感性：如果预估偏乐观会怎样？")
    print("  把标记为 [预估] 的阶段统一放大，看帧率如何变化：")
    print()
    rows = []
    for b in [SEARCH_BUDGET, SERVO_BUDGET]:
        for factor in [1.0, 1.5, 2.0, 3.0]:
            bb = b.scale_worst(factor)
            rows.append([b.name.split()[0], f"x{factor:.1f}",
                         f"{bb.total_ms:.1f}", f"{bb.fps:.1f}",
                         "达标" if bb.fps >= TARGET_FPS else "**不达标**"])
    table(["模式", "放大倍数", "合计ms", "可达fps", "30fps"], rows)
    print()
    print("  >> 判读：SEARCH 即使预估偏差 3 倍仍有余地；")
    print("     若连 3 倍都撑不住，说明 `find_blobs` 的实现方式有问题")
    print("     （见第 9 节的失败模式清单）。")

    # ---------------- 7. 延迟 ----------------
    section("7. 延迟分析（比帧率更重要）")
    print("  对控制系统而言，**传感器到执行的延迟**才是决定性能的量。")
    print("  延迟公式：")
    print("     latency = 流水线深度 x 帧周期 + 处理耗时 + 通信 + 执行")
    print()
    proc = SEARCH_BUDGET.total_ms
    comm = 1.0          # UART + 主控处理
    act = 2.0           # 执行器响应
    rows = []
    for depth in [1, 2, 3, 5]:
        lat = depth * FRAME_PERIOD_MS + proc + comm + act
        verdict = ("好" if lat < 60 else
                   "可接受" if lat < 100 else "**差**")
        rows.append([depth, f"{depth*FRAME_PERIOD_MS:.1f}",
                     f"{lat:.1f}", verdict])
    table(["流水线深度", "排队延迟ms", "总延迟ms", "评价"], rows)
    print()
    print("  >> 关键设计原则：**只处理最新帧，丢弃过期帧。**")
    print("     如果用无界队列缓存每一帧，延迟会随处理变慢而无界增长 ——")
    print("     这是视觉伺服的经典翻车方式。")
    print()
    depth1 = FRAME_PERIOD_MS + proc + comm + act
    print(f"     深度 1 时总延迟约 {depth1:.0f} ms。")
    print()
    print("     换算到推入段的**位置滞后**（决定会不会把棋子推歪）：")
    print()
    rows = []
    push_speed = 250.0      # mm/s，推入段建议速度
    jaw_half = 82.5         # 推板半开口
    for depth_v in [1, 2]:
        lat = depth_v * FRAME_PERIOD_MS + proc + comm + act
        lag = lat / 1000.0 * push_speed
        ratio = lag / jaw_half
        verdict = ("余量充足" if ratio < 0.25 else
                   "可接受" if ratio < 0.5 else "**危险**")
        rows.append([depth_v, f"{lat:.0f}", f"{lag:.0f}", f"{ratio:.0%}",
                     verdict])
    table(["流水线深度", "总延迟ms", "位置滞后mm",
           f"占推板半开口{jaw_half:.0f}mm", "评价"], rows)
    print()
    print(f"  >> 判读：深度 1 时位置滞后约 "
          f"{depth1/1000*push_speed:.0f} mm，")
    print(f"     只占推板半开口 {jaw_half:.0f}mm 的一小部分 —— **余量充足**，")
    print(f"     不需要为此降低推入速度。")
    print()
    print("     但延迟随流水线深度**线性增长且无上界**：")
    print("       深度 1 -> 14%    深度 2 -> 24%    深度 5 -> 54%（危险）")
    print("     所以关键不是'深度等于 1'这个数字，")
    print("     而是**绝不让帧队列积压** —— 处理慢于采集时，必须丢弃旧帧。")

    # ---------------- 8. 降级阶梯 ----------------
    section("8. 达不到 30 FPS 时的降级阶梯")
    rows = [
        ["1", "减少 ROI", "只处理推板前方区域，像素数减半", "无"],
        ["2", "降低处理分辨率", "640x360 -> 480x270", "远距离检测变差"],
        ["3", "缩小搜索范围", "只找 2m 内棋子，靠接近后再确认", "效率下降"],
        ["4", "前视降到 15-20 FPS", "下视伺服保持 30 FPS", "搜索变慢，控制不受影响"],
        ["5", "关掉 CNN 读字", "走降级策略 B", "无法按分值排序"],
    ]
    table(["级别", "手段", "做法", "代价"], rows)
    print()
    print("  >> 关键取舍：**下视伺服必须保住 30 FPS**（它直接决定推入命中率），")
    print("     前视搜索可以降到 15-20 FPS（只是慢一点，不影响得分能力）。")

    # ---------------- 9. 失败模式 ----------------
    section("9. 会毁掉 30 FPS 的六种写法")
    rows = [
        ["1", "Python 逐像素循环", "比 C 实现慢 100-1000 倍",
         "只用 image 模块的 C 方法"],
        ["2", "处理全分辨率", "像素数是 720p 的 5 倍",
         "先降采样再处理"],
        ["3", "两台相机同时跑 30fps", "处理量翻倍", "分时复用（见下）"],
        ["4", "在循环里等 UART 发完", "阻塞关键路径",
         "DMA 发送或异步"],
        ["5", "无界帧队列", "延迟无界增长", "只保留最新帧"],
        ["6", "循环内分配内存", "GC 抖动导致周期性卡顿",
         "预分配缓冲区并复用"],
    ]
    table(["#", "失败模式", "后果", "正确做法"], rows)
    print()
    print("  >> 第 1 条是最常见的坑。CanMV 的 `image` 模块方法（find_blobs、")
    print("     find_circles、find_rects 等）都是 C 实现，必须优先使用；")
    print("     一旦写成 `for x in range(w): for y in range(h):`，帧率必崩。")

    section("10. 双相机：不要同时跑 30 FPS")
    print("  直觉上'两个相机都 30 FPS'意味着 60 帧/秒的处理量，")
    print("  但实际上**两个相机需要的时间是错开的**：")
    print()
    rows = [
        ["SEARCH", "前视 30 FPS", "关闭", "巡场找棋子"],
        ["APPROACH", "前视 30 FPS", "关闭", "开向棋子"],
        ["ALIGN/READ", "关闭(或 5 FPS)", "下视（静止，无帧率要求）",
         "停车读字"],
        ["PUSH", "前视 5-10 FPS（仅避障）", "**下视 30 FPS**",
         "视觉伺服"],
        ["REPOSITION", "前视 30 FPS", "关闭", "退出并重定位"],
    ]
    table(["状态", "前视", "下视", "说明"], rows)
    print()
    print("  >> 任何时刻**只有一台相机需要 30 FPS**。")
    print("     这正好对应 docs/11 里已经定义的 MODE_CMD（IDLE/SEARCH/CLASSIFY/SERVO），")
    print("     用模式切换把处理负载集中到当前真正需要的那一路。")

    print("\n" + "=" * 86)
    print("结论：30 FPS 可达，但有三个前提")
    print("  1. 分辨率降到 720p 采集（5MP 只有 15 fps，硬件上就卡死）")
    print("  2. 处理前降采样，绝不处理全分辨率")
    print("  3. 两台相机分时复用，任何时刻只跑一路 30 FPS")
    print()
    print("下一步：用 firmware/k230/profiler.py 在真机上实测替换本表的预估值。")
    print("=" * 86)


if __name__ == "__main__":
    main()
