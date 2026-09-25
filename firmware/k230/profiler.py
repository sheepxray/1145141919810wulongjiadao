#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
K230 真机帧率测量工具
=====================

在 **K230 (CanMV)** 上运行，输出真实的各阶段耗时与可达帧率。

为什么必须有这个工具
--------------------
`tools/framerate.py` 里的时间全是 **[预估]**。估算可以指导设计，
但**只有实测才能保证 30 FPS**。本脚本把预估值替换成真实数据。

用法
----
1. 把本文件拷到 K230，重命名为 `main.py`（或从别的脚本 import）
2. 接好相机与串口
3. 运行，读串口输出的表格
4. 把实测值回填到 `tools/framerate.py` 的 Budget 表里，重新核算

测量项
------
  - 取帧耗时（sensor.snapshot）
  - 各处理阶段的耗时
  - 端到端帧率
  - 掉帧情况
  - 内存分配抖动

注意
----
本文件顶部导入 `sensor` / `image` / `time`，这些是 CanMV 专有模块，
**不能在 PC 上运行**。已做 try/except 保护，PC 上 import 不会报错，
但 `run()` 会提示需要真机。
"""

import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

# ---- CanMV 专有模块（PC 上不可用）----
try:
    import sensor
    import image
    import time
    from machine import UART
    ON_DEVICE = True
except ImportError:
    ON_DEVICE = False


# ==========================================================================
# 配置 —— 与 tools/framerate.py 的工作点保持一致
# ==========================================================================

CONFIG = {
    # 采集：720p@60 给处理端留 2 倍余量
    "sensor_w": 1280,
    "sensor_h": 720,
    "sensor_fps": 60,

    # 处理分辨率（降采样后）
    "proc_w": 640,
    "proc_h": 360,

    # 伺服模式的处理分辨率（更小的 ROI）
    "servo_roi_w": 320,
    "servo_roi_h": 160,

    # 颜色阈值（占位，必须现场标定）
    "red_threshold": (0, 100, 20, 127, 20, 127),
    "black_threshold": (0, 60, -20, 20, -20, 20),

    # 采样帧数（越多越准，但耗时越长）
    "sample_frames": 300,

    # 串口（若接了 STM32）
    "uart_id": 2,
    "uart_baud": 921600,
}


# ==========================================================================
# 计时辅助
# ==========================================================================

class Ticker:
    """
    简单的分段计时器。

    用 ticks_ms/ticks_us，避免 time.time() 的浮点开销。
    """

    def __init__(self):
        self.t0 = time.ticks_us()
        self.marks = []

    def mark(self, name):
        now = time.ticks_us()
        self.marks.append((name, time.ticks_diff(now, self.t0) / 1000.0))
        self.t0 = now

    def report(self):
        return list(self.marks)


class Accum:
    """累积多帧的分段耗时，输出均值/最大/最小。"""

    def __init__(self):
        self.data = {}

    def add(self, name, ms):
        if name not in self.data:
            self.data[name] = []
        self.data[name].append(ms)

    def stats(self):
        out = []
        for name, samples in self.data.items():
            if not samples:
                continue
            n = len(samples)
            mean = sum(samples) / n
            out.append((name, mean, min(samples), max(samples), n))
        return out


# ==========================================================================
# 传感器初始化
# ==========================================================================

def init_sensor(cfg):
    """按推荐工作点配置 OV5640。"""
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.HD)          # 1280x720
    sensor.set_hmirror(False)
    sensor.set_vflip(False)

    # 尝试设置帧率（不同 CanMV 版本 API 不同）
    try:
        sensor.set_framerate(cfg["sensor_fps"])
    except Exception:
        try:
            sensor.set_framerate(cfg["sensor_fps"], cfg["sensor_fps"])
        except Exception:
            pass

    sensor.skip_frames(time=1000)
    return sensor


# ==========================================================================
# 各项测量
# ==========================================================================

def measure_snapshot(acc, cfg):
    """只取帧，不处理 —— 测出**传感器真实上限帧率**"""
    n = cfg["sample_frames"]
    t_start = time.ticks_us()
    for _ in range(n):
        img = sensor.snapshot()
    t_end = time.ticks_us()
    total_ms = time.ticks_diff(t_end, t_start) / 1000.0
    per_frame = total_ms / n
    acc.add("取帧（snapshot）", per_frame)
    return per_frame


def measure_search(acc, cfg):
    """SEARCH：取帧 -> 降采样 -> find_blobs -> 打包"""
    n = cfg["sample_frames"]
    proc_w, proc_h = cfg["proc_w"], cfg["proc_h"]

    # 预分配缓冲区，避免循环内分配（GC 抖动）
    for _ in range(n):
        t = Ticker()
        img = sensor.snapshot()
        t.mark("取帧")

        # 降采样（ROI 直接取 + 缩小）
        half = img.mean_pooled(2, 2)
        t.mark("降采样")

        blobs = half.find_blobs(
            [cfg["red_threshold"], cfg["black_threshold"]],
            pixels_threshold=50,
            area_threshold=50,
            merge=True,
        )
        t.mark("find_blobs")

        # 候选筛选（模拟坐标变换）
        cnt = 0
        for b in blobs:
            if b.w() > 2 and b.h() > 2:
                cnt += 1
        t.mark("坐标变换与筛选")

        for name, ms in t.report():
            acc.add(name, ms)


def measure_servo(acc, cfg):
    """SERVO：小 ROI 的 find_blobs（推入段用）"""
    n = cfg["sample_frames"]
    roi_w, roi_h = cfg["servo_roi_w"], cfg["servo_roi_h"]

    for _ in range(n):
        t = Ticker()
        img = sensor.snapshot()
        t.mark("取帧")

        # 只在推板前方取 ROI
        cx = (img.width() - roi_w) // 2
        roi = img.copy(cx, img.height() - roi_h - 160, roi_w, roi_h)
        t.mark("ROI 拷贝")

        blobs = roi.find_blobs(
            [cfg["red_threshold"]],
            pixels_threshold=30,
            area_threshold=30,
            merge=True,
        )
        t.mark("find_blobs(ROI)")

        for name, ms in t.report():
            acc.add(name, ms)


def measure_char_crop(acc, cfg):
    """READ：读字预处理（车静止，无帧率要求，但要看绝对耗时）"""
    n = min(cfg["sample_frames"], 100)

    for _ in range(n):
        t = Ticker()
        img = sensor.snapshot()
        t.mark("取帧")

        cx = img.width() // 2
        roi = img.copy(cx - 160, img.height() - 480, 320, 320)
        t.mark("ROI 裁切")

        blobs = roi.find_blobs([cfg["red_threshold"]], merge=True)
        t.mark("圆形筛选")

        if blobs:
            b = blobs[0]
            crop = roi.copy(b.x(), b.y(), b.w(), b.h())
            t.mark("字符裁切")

            small = crop.resize(64, 64)
            t.mark("归一化 64x64")

        for name, ms in t.report():
            acc.add(name, ms)


def measure_nn(acc, cfg):
    """
    测 CNN 推理耗时。

    需要真实 kmodel。若未部署，跳过并提示。
    """
    try:
        from libs.PipeLine import ScopedTiming
        # 真机上若已加载 kmodel，这里应替换为实际推理调用
        # 目前仅做占位，避免误报
        raise ImportError("kmodel 未部署")
    except ImportError:
        acc.add("CNN 推理（KPU）", float("nan"))
        return False


# ==========================================================================
# 报告
# ==========================================================================

def report(acc, cfg):
    print("\n" + "=" * 78)
    print("K230 实测帧率报告")
    print("=" * 78)
    print(f"  采集模式: {cfg['sensor_w']}x{cfg['sensor_h']} "
          f"@ {cfg['sensor_fps']} fps 目标")
    print(f"  处理分辨率: {cfg['proc_w']}x{cfg['proc_h']}")
    print(f"  采样帧数: {cfg['sample_frames']}")
    print()

    # ---- 分段耗时 ----
    print("  各阶段耗时（ms）")
    print(f"  {'阶段':<28} {'均值':>8} {'最小':>8} {'最大':>8} {'样本':>6}")
    print("  " + "-" * 62)

    groups = {
        "取帧": "取帧（snapshot）",
    }

    total_search = 0.0
    total_servo = 0.0

    for name, mean, lo, hi, n in acc.stats():
        if mean != mean:      # NaN
            print(f"  {name:<28} {'未测量':>8} {'-':>8} {'-':>8} {n:>6}")
            continue
        print(f"  {name:<28} {mean:>8.2f} {lo:>8.2f} {hi:>8.2f} {n:>6}")

    # ---- 帧率 ----
    print()
    print("=" * 78)
    print("可达帧率")
    print("=" * 78)

    stats = {name: mean for name, mean, _, _, _ in acc.stats()
             if mean == mean}

    snap = stats.get("取帧（snapshot）", None)
    if snap:
        print(f"  传感器上限（仅取帧）    : {1000.0/snap:>7.1f} FPS "
              f"({snap:.2f} ms/帧)")

    # SEARCH 关键路径
    search_stages = ["降采样", "find_blobs", "坐标变换与筛选"]
    if all(s in stats for s in search_stages):
        total = snap + sum(stats[s] for s in search_stages)
        print(f"  SEARCH 关键路径         : {total:>7.2f} ms  "
              f"-> {1000.0/total:>6.1f} FPS  "
              f"{'达标' if 1000.0/total >= 30 else '**不达标**'}")

    # SERVO 关键路径
    servo_stages = ["ROI 拷贝", "find_blobs(ROI)"]
    if all(s in stats for s in servo_stages):
        total = snap + sum(stats[s] for s in servo_stages)
        print(f"  SERVO 关键路径          : {total:>7.2f} ms  "
              f"-> {1000.0/total:>6.1f} FPS  "
              f"{'达标' if 1000.0/total >= 30 else '**不达标**'}")

    # READ
    read_stages = ["ROI 裁切", "圆形筛选", "字符裁切", "归一化 64x64"]
    if all(s in stats for s in read_stages):
        total = snap + sum(stats[s] for s in read_stages)
        print(f"  READ 关键路径           : {total:>7.2f} ms  "
              f"(读字无帧率要求)")

    # ---- 结论 ----
    print()
    print("=" * 78)
    print("回填指引")
    print("=" * 78)
    print("  把上面的实测值填进 tools/framerate.py 的 Budget 表，")
    print("  并把来源标记从 [预估] 改成 [实测]。")
    print("  然后重跑 `python tools/framerate.py` 复核 30 FPS 是否仍达标。")
    print()
    print("  若 SEARCH 或 SERVO 不达标，按 tools/framerate.py 第 8 节的")
    print("  降级阶梯处理，优先保住 **SERVO**（它决定推入命中率）。")
    print("=" * 78)


# ==========================================================================
# 主程序
# ==========================================================================

def run():
    if not ON_DEVICE:
        print("=" * 78)
        print("本脚本需要在 K230 (CanMV) 上运行。")
        print("=" * 78)
        print()
        print("在 PC 上无法执行的原因：依赖 sensor / image 模块。")
        print()
        print("在 PC 上请改用：")
        print("    python tools/framerate.py       # 预算分析（预估）")
        print()
        print("在 K230 上部署步骤：")
        print("    1. 拷贝本文件到 K230 文件系统")
        print("    2. 接好相机（CSICamera）")
        print("    3. 运行，读串口输出")
        print("    4. 把实测值回填 tools/framerate.py")
        print("=" * 78)
        return

    cfg = CONFIG
    print("K230 帧率测量开始...")
    print(f"  采集 {cfg['sensor_w']}x{cfg['sensor_h']} "
          f"@ {cfg['sensor_fps']} fps")

    init_sensor(cfg)
    print("  传感器初始化完成")

    acc = Accum()

    print("\n[1/4] 测传感器上限（仅取帧）...")
    measure_snapshot(acc, cfg)

    print("[2/4] 测 SEARCH 路径...")
    measure_search(acc, cfg)

    print("[3/4] 测 SERVO 路径...")
    measure_servo(acc, cfg)

    print("[4/4] 测 READ 预处理...")
    measure_char_crop(acc, cfg)

    if not measure_nn(acc, cfg):
        print("      CNN 推理未测量（kmodel 未部署）")

    report(acc, cfg)


if __name__ == "__main__":
    run()
