#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
1080p@30 视觉管线设计工具
=========================

用途：设计并校核 1080p@30 的完整管线。

**不只是改个分辨率。** 它牵扯三件事：
  1. 传感器能否稳定输出 1080p@30（很多传感器在这一档没有余量）
  2. 内存带宽（1080p 数据量是 720p 的 2.25 倍）
  3. 降采样路径用 CPU 还是硬件加速器（决定 CPU 占用率）

本工具算七件事：
  1. 传感器模式与帧率余量
  2. 数据格式与内存带宽
  3. 管线阶段划分与降采样策略
  4. RK3588 上的处理预算
  5. 端到端延迟
  6. 降级阶梯
  7. 结论

运行：
    python tools/vision_pipeline.py
"""

import math

import sys as _sys

# Windows 控制台默认 GBK，无法显示 emoji / 部分符号
try:
    _sys.stdout.reconfigure(encoding="utf-8")
    _sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass


def _warn_obsolete():
    """运行时醒目警告 —— 本工具基于旧平台，数字不可直接引用。"""
    bar = "=" * 70
    _sys.stderr.write("\n" + bar + "\n")
    _sys.stderr.write("⚠️  警告：本工具假设【RK3588 的 RGA 硬件加速】，K230D 上不存在该硬件！\n")
    _sys.stderr.write("    结论部分见 docs/14（已标注全部失效），实际方案见 docs/16。\n")
    _sys.stderr.write("    帧率需在 K230D 上用 firmware/k230/profiler.py 重新实测。\n")
    _sys.stderr.write(bar + "\n\n")


# ==========================================================================
# 1. 传感器候选
# ==========================================================================

class Sensor:
    def __init__(self, name, mp, modes, mipi_lanes, price_cny, note=""):
        self.name = name
        self.mp = mp
        self.modes = modes          # [(w, h, max_fps)]
        self.lanes = mipi_lanes
        self.price = price_cny
        self.note = note

    def fps_at(self, w, h):
        """指定分辨率下的最高帧率；None 表示不支持"""
        for mw, mh, fps in self.modes:
            if mw == w and mh == h:
                return fps
        return None

    def margin_at(self, w, h, target_fps):
        """帧率余量倍数"""
        f = self.fps_at(w, h)
        if f is None:
            return None
        return f / target_fps


SENSORS = {
    "OV5640": Sensor("OV5640", 5.0, [
        (2592, 1944, 15), (1920, 1080, 30), (1280, 720, 60), (640, 480, 90),
    ], 2, 38, "便宜，但 1080p 恰好卡在 30，无余量"),

    "OV5647": Sensor("OV5647", 5.0, [
        (2592, 1944, 15), (1920, 1080, 30), (1280, 720, 60),
    ], 2, 45, "树莓派 v1 同款"),

    "IMX219": Sensor("IMX219", 8.0, [
        (3280, 2464, 21), (1920, 1080, 60), (1280, 720, 90),
    ], 2, 65, "树莓派 v2 同款，1080p@60 有 2x 余量"),

    "IMX477": Sensor("IMX477", 12.3, [
        (4056, 3040, 10), (1920, 1080, 60), (1280, 720, 120),
    ], 2, 220, "树莓派 HQ 同款，画质最好"),

    "IMX708": Sensor("IMX708", 11.9, [
        (4608, 2592, 14), (2304, 1296, 56), (1920, 1080, 50),
    ], 2, 180, "树莓派 v3 同款，HDR 支持好"),
}


# ==========================================================================
# 2. 像素格式与带宽
# ==========================================================================

class PixelFormat:
    def __init__(self, name, bytes_per_px):
        self.name = name
        self.bpp = bytes_per_px


FORMATS = {
    "RGB565": PixelFormat("RGB565", 2.0),
    "RGB888": PixelFormat("RGB888", 3.0),
    "NV12":   PixelFormat("NV12 (YUV420)", 1.5),
    "YUYV":   PixelFormat("YUYV (YUV422)", 2.0),
    "GRAY8":  PixelFormat("GRAY8", 1.0),
}


def bandwidth_mb_s(w, h, fps, fmt: PixelFormat):
    return w * h * fps * fmt.bpp / 1e6


# ==========================================================================
# 3. 管线阶段
# ==========================================================================

class Stage:
    def __init__(self, name, engine, ms, note="", critical=True):
        self.name = name
        self.engine = engine        # CPU / RGA / NPU / DMA / ISP
        self.ms = ms
        self.note = note
        self.critical = critical


# RK3588 上的 1080p@30 管线
# 关键：降采样用 RGA 硬件，不用 CPU
PIPELINE_1080P = [
    Stage("ISP 采集 1080p", "ISP", 0.0,
          "与处理并行，不计入关键路径", critical=False),
    Stage("RGA 降采样 1080p -> 540p", "RGA", 1.8,
          "硬件加速，CPU 几乎不参与"),
    Stage("色彩空间转换 (RGA)", "RGA", 1.2,
          "RGB -> HSV 或 YUV 阈值友好格式"),
    Stage("阈值分割 540p", "CPU", 4.5,
          "960x540 像素扫描"),
    Stage("连通域标记 540p", "CPU", 7.0,
          "两遍扫描 + 并查集"),
    Stage("圆形/面积筛选", "CPU", 1.5, "候选筛选"),
    Stage("坐标变换 + 打包", "CPU", 0.8, "图像->车体坐标"),
    Stage("UART 发送", "DMA", 0.3, "非阻塞"),
]

# 伺服模式（推入段）：小 ROI，更高频率
PIPELINE_SERVO = [
    Stage("ISP 采集 1080p", "ISP", 0.0, "并行", critical=False),
    Stage("RGA 裁 ROI + 降采样", "RGA", 0.9,
          "540x270，只处理推板前方"),
    Stage("阈值 + 质心 540x270", "CPU", 3.0, "小 ROI 很快"),
    Stage("UART 发送", "DMA", 0.2, "非阻塞"),
]

# 读字模式（静止）
PIPELINE_READ = [
    Stage("ISP 采集 1080p", "ISP", 0.0, "并行", critical=False),
    Stage("RGA 裁 ROI", "RGA", 0.8, "推板前方区域"),
    Stage("圆形筛选 + 红黑判定", "CPU", 3.5, "静止，可慢慢算"),
    Stage("字符裁切 + 归一化", "RGA+CPU", 2.0, "缩放到 64x64"),
    Stage("NPU 推理", "NPU", 6.0, "7 类兵种分类"),
    Stage("后处理 + 发送", "CPU", 0.5, ""),
]


def total_ms(pipeline):
    return sum(s.ms for s in pipeline if s.critical)


def by_engine(pipeline):
    out = {}
    for s in pipeline:
        if not s.critical:
            continue
        out[s.engine] = out.get(s.engine, 0.0) + s.ms
    return out


# ==========================================================================
# 输出辅助
# ==========================================================================

def section(title):
    print("\n" + "=" * 86)
    print(title)
    print("=" * 86)


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
    TARGET_W, TARGET_H, TARGET_FPS = 1920, 1080, 30.0
    FRAME_MS = 1000.0 / TARGET_FPS

    print("=" * 86)
    print("1080p@30 视觉管线设计")
    print("=" * 86)
    print(f"  目标：{TARGET_W}x{TARGET_H} @ {TARGET_FPS:.0f} FPS "
          f"（帧周期 {FRAME_MS:.2f} ms）")
    print(f"  平台：RK3588（CPU 20000 MOPS / NPU 6 TOPS）")

    # ---------------- 1. 传感器 ----------------
    section("1. 传感器：谁能稳定输出 1080p@30")
    print("  ⚠️ 关键判据不是「支持 1080p@30」，而是**有没有余量**。")
    print("     恰好卡在 30 fps 的传感器，实际会因为曝光/读出开销掉到 25-28。")
    print()
    rows = []
    for k, s in SENSORS.items():
        fps = s.fps_at(1920, 1080)
        margin = s.margin_at(1920, 1080, TARGET_FPS)
        if fps is None:
            rows.append([s.name, s.mp, "不支持", "-", "-", f"¥{s.price}", s.note])
            continue
        verdict = ("**无余量**" if margin < 1.2 else
                   "勉强" if margin < 1.5 else
                   "充足")
        rows.append([s.name, f"{s.mp:.1f}", f"{fps}", f"{margin:.1f}x",
                     verdict, f"¥{s.price}", s.note])
    table(["传感器", "MP", "1080p最高fps", "余量", "评价", "价格", "备注"], rows)
    print()
    print("  >> 结论：")
    print("     - **OV5640 的 1080p 恰好是 30 fps，零余量** —— 有风险")
    print("     - **IMX219（1080p@60）有 2.0x 余量**，价格仅贵 ¥27")
    print("     - IMX477/IMX708 余量更大但价格高")
    print()
    print("     **推荐 IMX219**：2x 余量让帧率稳定，且与 RK3588 的")
    print("     MIPI CSI 接口兼容性好（树莓派生态成熟）。")
    print()
    print("     代价：单价 ¥38 -> ¥65，两台 +¥54。")
    print("     这 ¥54 买的是「帧率不会掉到 30 以下」的确定性。")

    # ---------------- 2. 内存带宽 ----------------
    section("2. 内存带宽（1080p 的数据量是 720p 的 2.25 倍）")
    print(f"  按 {TARGET_W}x{TARGET_H}@{TARGET_FPS:.0f} 计算各格式的带宽：")
    print()
    rows = []
    for k, fmt in FORMATS.items():
        bw = bandwidth_mb_s(TARGET_W, TARGET_H, TARGET_FPS, fmt)
        # 假设管线中数据被读写 4 次（采集写入、降采样读+写、算法读）
        total = bw * 4
        rows.append([fmt.name, f"{fmt.bpp:.1f}", f"{bw:.0f}", f"{total:.0f}"])
    table(["格式", "字节/像素", "单次带宽 MB/s", "管线4次 MB/s"], rows)
    print()
    print("  >> RK3588 的内存带宽约 10-20 GB/s，所以**带宽不是瓶颈**。")
    print("     即使 RGB888 + 4 次读写也只有 1.5 GB/s，占用 < 15%。")
    print()
    print("  >> 但格式选择仍然重要，因为它影响 **CPU 的处理速度**：")
    print("     - RGB565 (2B): 阈值比较直接，但要拆位")
    print("     - NV12 (1.5B): ISP 原生输出，CPU 读 Y 分量即可做灰度阈值")
    print("     - **推荐：ISP 输出 NV12，降采样后转灰度做阈值**")
    print("       灰度单通道让 CPU 的阈值扫描快 2-3 倍")

    # ---------------- 3. 管线阶段 ----------------
    section("3. 管线阶段划分与降采样策略")
    print("  ★ 核心设计：**降采样用 RGA 硬件，不用 CPU**。")
    print("     RK3588 的 RGA 是 2D 硬件加速器，做缩放/色彩转换几乎不占 CPU。")
    print()
    for pname, pipeline, freq in [
        ("SEARCH（巡场搜索）", PIPELINE_1080P, 30),
        ("SERVO（推入伺服）", PIPELINE_SERVO, 30),
        ("READ（停车读字）", PIPELINE_READ, None),
    ]:
        print(f"  【{pname}】")
        rows = []
        for s in pipeline:
            rows.append([
                s.name, s.engine,
                f"{s.ms:.1f}" if s.critical else "并行",
                "是" if s.critical else "否",
                s.note,
            ])
        table(["阶段", "引擎", "耗时ms", "关键路径", "说明"], rows)
        tot = total_ms(pipeline)
        eng = by_engine(pipeline)
        print()
        if freq:
            fps = 1000.0 / tot if tot > 0 else float("inf")
            print(f"    关键路径 {tot:.1f} ms -> {fps:.0f} FPS  "
                  f"{'达标' if fps >= TARGET_FPS else '**不达标**'}"
                  f"  （余量 {FRAME_MS - tot:.1f} ms）")
        else:
            print(f"    关键路径 {tot:.1f} ms（读字无帧率要求）")
        print(f"    各引擎占用: " +
              " ".join(f"{k}={v:.1f}ms" for k, v in eng.items()))
        print()

    # ---------------- 4. 对比：CPU 降采样 vs RGA 降采样 ----------------
    section("4. ★ 为什么必须用 RGA 而不是 CPU 降采样")
    print("  对比两种降采样方式（1080p -> 540p）：")
    print()
    compare = [
        ["CPU 逐像素降采样", "CPU", "22.0",
         "1080p 全像素遍历，Python/C 都慢"],
        ["CPU 用 SIMD 优化", "CPU", "8.0", "需要手写 NEON，开发成本高"],
        ["**RGA 硬件**", "**RGA**", "**1.8**", "★ 硬件加速，CPU 几乎不参与"],
    ]
    table(["方式", "引擎", "耗时ms", "说明"], compare)
    print()
    print("  >> CPU 降采样要 22ms，单这一项就吃掉 66% 的帧预算。")
    print("     RGA 只要 1.8ms，**快 12 倍**。")
    print()
    print("     这印证了第 2 轮的结论：瓶颈在 CPU 侧。")
    print("     **把降采样卸载到 RGA，是达成 1080p@30 的关键手段。**")
    print()
    print("  若改用 CPU 降采样的后果：")
    cpu_total = 22.0 + 4.5 + 7.0 + 1.5 + 0.8 + 0.3
    print(f"    关键路径变成 {cpu_total:.1f} ms -> {1000/cpu_total:.0f} FPS")
    print(f"    **{'达标' if 1000/cpu_total >= 30 else '不达标，无法满足 30 FPS'}**")

    # ---------------- 5. 端到端延迟 ----------------
    section("5. 端到端延迟")
    search_ms = total_ms(PIPELINE_1080P)
    servo_ms = total_ms(PIPELINE_SERVO)
    print("  延迟组成：流水线深度 x 帧周期 + 处理 + 通信 + 执行")
    print()
    print("  ⚠️ 必须分两种工况算 —— 它们用不同的管线：")
    print(f"     SEARCH 模式处理 {search_ms:.1f}ms（巡场时）")
    print(f"     SERVO  模式处理 {servo_ms:.1f}ms（推入时）★ 这个才决定命中率")
    print()
    print(f"  【推入段 SERVO 模式】车速 250 mm/s，推板半开口 82.5mm")
    print()
    rows = []
    for depth in [1, 2, 3]:
        lat = depth * FRAME_MS + servo_ms + 1.0 + 2.0
        lag = lat / 1000.0 * 250.0
        ratio = lag / 82.5
        verdict = ("余量充足" if ratio < 0.25 else
                   "可接受" if ratio < 0.5 else "危险")
        rows.append([depth, f"{depth*FRAME_MS:.1f}", f"{lat:.1f}",
                     f"{lag:.0f}", f"{ratio:.0%}", verdict])
    table(["深度", "排队ms", "总延迟ms", "位置滞后mm",
           "占推板半开口", "评价"], rows)
    print()
    print(f"  【巡场段 SEARCH 模式】同样算法（此段不推棋子，滞后不关键）")
    print()
    rows = []
    for depth in [1, 2]:
        lat = depth * FRAME_MS + search_ms + 1.0 + 2.0
        rows.append([depth, f"{depth*FRAME_MS:.1f}", f"{lat:.1f}"])
    table(["深度", "排队ms", "总延迟ms"], rows)
    print()
    print(f"  >> 关键：推入段总延迟仅 "
          f"{FRAME_MS + servo_ms + 3.0:.0f} ms，位置滞后 "
          f"{(FRAME_MS+servo_ms+3.0)/1000*250:.0f} mm，")
    print(f"     只占推板半开口的 "
          f"{(FRAME_MS+servo_ms+3.0)/1000*250/82.5*100:.0f}% —— **余量充足**。")
    print()
    print(f"     虽然 1080p 的帧周期（{FRAME_MS:.1f}ms）比 720p 大，")
    print(f"     但 SERVO 模式只处理 540x270 的小 ROI，处理耗时压到 "
          f"{servo_ms:.1f}ms，")
    print(f"     所以**延迟反而优于用大管线**。")

    # ---------------- 6. 降级阶梯 ----------------
    section("6. 达不到 1080p@30 时的降级阶梯")
    rows = [
        ["1", "降采样用 RGA", "别用 CPU 做缩放", "无（必须做）"],
        ["2", "SEARCH 降到 20 FPS", "SERVO 保持 30 FPS", "搜索变慢"],
        ["3", "处理分辨率降 540p->360p", "阈值/连通域像素减半", "远距离检测变差"],
        ["4", "降采样两级 1080p->540p->270p", "只在 270p 上做检测", "小棋子可能漏检"],
        ["5", "采集降到 720p", "放弃 1080p 需求", "分辨率回退"],
    ]
    table(["级别", "手段", "做法", "代价"], rows)
    print()
    print("  >> 注意第 5 条：如果 RGA 用不了或驱动异常，")
    print("     退回 720p 是最后的保险 —— 但那就没有满足 1080p 需求。")

    # ---------------- 7. 结论 ----------------
    section("7. 结论")
    print("  【管线设计】")
    print(f"    采集    {TARGET_W}x{TARGET_H} @ {TARGET_FPS:.0f} FPS (ISP)")
    print(f"    降采样  RGA -> 960x540          (硬件加速，1.8ms)")
    print(f"    检测    阈值 + 连通域 @ 540p     (CPU, 11.5ms)")
    print(f"    读字    NPU 7 类分类            (6.0ms, 仅停车时)")
    print()
    print("  【关键决策】")
    rows = [
        ["传感器", "IMX219 (1080p@60)", "¥65 x2",
         "2x 帧率余量，OV5640 零余量有风险"],
        ["降采样", "RGA 硬件加速", "无",
         "★ 关键：CPU 降采样要 22ms，RGA 只要 1.8ms"],
        ["像素格式", "NV12", "无", "ISP 原生，灰度阈值快 2-3x"],
        ["SEARCH 处理分辨率", "960x540", "无", "1080p 的 1/4 像素"],
        ["SERVO 处理分辨率", "540x270 ROI", "无", "只处理推板前方"],
    ]
    table(["项目", "选型", "价格", "理由"], rows)
    print()
    print("  【性能】")
    print(f"    SEARCH  关键路径 {search_ms:.1f} ms -> "
          f"{1000/search_ms:.0f} FPS（余量 {FRAME_MS-search_ms:.1f}ms）")
    print(f"    SERVO   关键路径 {total_ms(PIPELINE_SERVO):.1f} ms -> "
          f"{1000/total_ms(PIPELINE_SERVO):.0f} FPS")
    print(f"    READ    关键路径 {total_ms(PIPELINE_READ):.1f} ms（静止，无要求）")
    print()
    print("=" * 86)
    print("第 3 轮迭代完成。下一步（第 4 轮）：字符识别与优先级算法。")
    print("=" * 86)


if __name__ == "__main__":
    _warn_obsolete()
    main()
