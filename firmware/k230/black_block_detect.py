# CanMV K230D —— 黑色长方体小块识别
#
# 在摄像头画面里找出「黑色、形状接近长方形」的小块，输出它的位置 / 尺寸 /
# 长轴角度，并在 IDE 预览画面里框出来。
#
# 思路：两级筛选 —— 先颜色，后形状
#   1) 颜色：黑色 = LAB 空间的亮度 L 很低。用 find_blobs 把画面里所有暗色
#      连通域找出来（这一步免费、快、稳，不依赖 CNN）。
#   2) 形状：真正的「长方体小块」是一个实心长条形，用两个旋转无关的特征区分：
#        solidity = 像素数 / 凸包面积   （实心块 ≈ 1.0，噪声/镂空会明显偏低）
#        aspect   = 长轴长 / 短轴长      （长方体明显 > 1，圆点/正方形 ≈ 1）
#      注意：这里刻意不用「像素数 / 外接矩形面积」那个 extent —— 它只在方块
#      摆正时接近 1，斜着摆放会掉到 0.5，会把斜放的目标误杀。
#
# 只依赖主程序 main.py 已经在用的 media.* 接口；本文件既可单独运行调试，
# 也可以 import 到 main.py 里复用（见文件末尾的 detect_black_block）。
#
# 调参顺序建议：先看 IDE 预览里的框，再依次调 BLACK_THRESHOLD（黑白分界）
# → PIXELS_THRESHOLD（多小的算噪声）→ MIN_SOLIDITY / MIN_ASPECT（形状宽容度）
# → EXPECTED_ASPECT（你的黑块长宽比，用于在多个候选里挑最像的那个）。
#
# 真机提示（K230D v1.8-0 实测）：
#   * 若黑色目标时有时无，多半是自动曝光/白平衡在漂 —— 可调用
#     sensor.auto_exposure(False)/set_auto_gain(False)/set_auto_whitebal(False)
#     并手动定住 sensor.exposure()，比反复调阈值有效。
#   * 需要更高帧率时把 FRAME_W/H 降到 320x240（仓库视觉文档的红线是全分辨率
#     别跑满帧），再把输出的像素坐标按比例放大即可。

import time, os, gc, math

from media.sensor import *
from media.display import *
from media.media import *

__all__ = [
    "detect_black_block", "measure_blob", "is_black_block",
    "BLACK_THRESHOLD",
]

# ============================ 可调参数 ============================

FRAME_W = 640
FRAME_H = 480

# 黑色阈值（LAB 空间）。find_blobs 在 RGB565 图上按 LAB 阈值工作。
#   格式：(L_min, L_max, A_min, A_max, B_min, B_max)
#   L 越小越黑。注意：像素接近黑时 A/B 本来就会收敛到中性，所以收窄 A/B
#   并不能真的把「深蓝/深绿」排除掉；真正决定成败的是 L 上限。现场光照偏色
#   时优先调 L_max，或放宽 A/B（例如 ±64）以免把被暖光染色的黑块漏掉。
BLACK_THRESHOLD = (0, 40, -30, 30, -30, 30)

# find_blobs 的面积门限：像素数少于 PIXELS_THRESHOLD、或外接矩形小于
# AREA_THRESHOLD 的色块直接丢弃（用来滤掉噪点）。
PIXELS_THRESHOLD = 200
AREA_THRESHOLD   = 100

# 是否合并相邻色块。默认关：一个实心黑块不该碎裂，开启反而会把附近的暗斑 /
# 阴影并进来、把尺寸和形状带偏（已实测：merge 会把干净场景并成一个
# solidity 很低的 blob 从而丢失真目标）。只有当黑块被高光切开时才开。
MERGE        = False
MERGE_MARGIN = 4

# 只在这块区域内搜索；None = 全画面。例如只看中间：(100, 80, 440, 320)
ROI = None

# 形状宽容度
MIN_SOLIDITY = 0.85   # 实心度下限：噪声/镂空/不规则形状会被刷掉
MIN_ASPECT   = 1.20   # 长轴/短轴下限：用来和圆形棋子区分开
MAX_ASPECT   = 6.0    # 上限：太细长的更像线/边缘，不是「小块」
EXPECTED_ASPECT = 2.5 # 你的黑块「长边/短边」大概是多少；多个候选时挑最接近的

# 尺寸合理性上限：真正的「小块」不会占满画面。暗场景 / 镜头被挡时整帧会变成
# 一个大暗块（aspect ≈ 画面比，能骗过形状筛选），必须用尺寸和贴边把它们排除。
MAX_AREA_FRAC = 0.35  # 外接矩形面积 > 该比例的整帧 → 不是目标
MAX_DIM_FRAC  = 0.90  # 外接矩形任一边 > 该比例的整边 → 不是目标
REJECT_BORDER = True  # 贴着画面边缘的 blob 通常是背景/阴影 → 丢弃

MAX_PRINT = 5         # 调试时每帧最多打印几行候选

COLOR_BOX   = (255, 255, 0)   # 外接矩形：黄
COLOR_ORI   = (0, 255, 0)     # 长轴方向框：绿
COLOR_CROSS = (255, 0, 0)     # 中心十字：红


# ============================ 识别核心 ============================

def _axis_len(line):
    """major_axis_line()/minor_axis_line() 返回 (x0,y0,x1,y1)，求其长度。"""
    x0, y0, x1, y1 = line
    return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)


def measure_blob(b):
    """把一个 blob 量化成形状特征，返回 dict（不做通过/否决判断）。

    坐标均为图像像素，方向以图像为准（若相机装了 hmirror/vflip，调用方需知悉）。
    """
    solidity = b.solidity()
    major = _axis_len(b.major_axis_line())
    minor = _axis_len(b.minor_axis_line())
    aspect = (major / minor) if minor > 0 else 0.0
    return {
        "x": b.x(), "y": b.y(), "w": b.w(), "h": b.h(),
        "cx": b.cx(), "cy": b.cy(),
        "pixels": b.pixels(),
        "area": b.area(),
        "solidity": solidity,
        "aspect": aspect,
        # 接近正方形或被边缘裁切时角度不可靠，用 angle_ok 标记，角度本身仍给出。
        "angle": b.rotation_deg(),
        "angle_ok": aspect >= 1.6,
    }


def is_black_block(m, frame_w=FRAME_W, frame_h=FRAME_H):
    """按形状特征判断是否像「黑色长方体小块」。m 为 measure_blob() 的结果。

    含三重把关：形状（实心度 + 长宽比）、尺寸上限、贴边排除。
    """
    if not (m["solidity"] >= MIN_SOLIDITY and
            MIN_ASPECT <= m["aspect"] <= MAX_ASPECT):
        return False
    # 尺寸上限：排除「整帧暗块」这类伪装成方块的背景。
    if m["w"] * m["h"] > MAX_AREA_FRAC * frame_w * frame_h:
        return False
    if m["w"] > MAX_DIM_FRAC * frame_w or m["h"] > MAX_DIM_FRAC * frame_h:
        return False
    # 贴边排除：真目标在画面内部，贴边的多半是背景/阴影/桌面边。
    if REJECT_BORDER and (m["x"] <= 0 or m["y"] <= 0 or
                          m["x"] + m["w"] >= frame_w or
                          m["y"] + m["h"] >= frame_h):
        return False
    return True


def _block_score(m):
    """候选排序分：形状优先，面积只作次要项。

    单独用像素数排序会选中「更大但更不像方块」的区域；这里以实心度为主，
    再用长宽比与 EXPECTED_ASPECT 的接近程度加权。
    """
    asp_err = abs(m["aspect"] - EXPECTED_ASPECT) / EXPECTED_ASPECT
    return m["solidity"] * (1.0 - min(1.0, asp_err))


def detect_black_block(img, draw=True, verbose=False):
    """在 img 上检测黑色长方体小块。

    返回：最可信的一个目标的 dict（字段见 measure_blob），没找到返回 None。
    传入 draw=True 会在 img 上画框，调用方自行 Display.show_image(img)。
    """
    kwargs = {
        "pixels_threshold": PIXELS_THRESHOLD,
        "area_threshold": AREA_THRESHOLD,
        "merge": MERGE,
        "margin": MERGE_MARGIN,
    }
    if ROI is not None:          # 不传 roi=None，避免个别固件不接受 None
        kwargs["roi"] = ROI
    blobs = img.find_blobs([BLACK_THRESHOLD], **kwargs)

    fw = img.width()
    fh = img.height()

    candidates = []
    for i, b in enumerate(blobs):
        m = measure_blob(b)
        ok = is_black_block(m, fw, fh)
        if verbose and i < MAX_PRINT:       # 打印预算按「检查过几个」算，与通过数无关
            print("blob px=%d solid=%.2f aspect=%.2f %s" % (
                m["pixels"], m["solidity"], m["aspect"], "OK" if ok else "--"))
        if ok:
            m["_blob"] = b
            candidates.append(m)

    if not candidates:
        return None

    # 按形状分挑最像方块的；分数相同再看像素数。
    best = max(candidates, key=lambda m: (_block_score(m), m["pixels"]))
    b = best.pop("_blob")

    if draw:
        img.draw_rectangle(best["x"], best["y"], best["w"], best["h"],
                           color=COLOR_BOX, thickness=2)
        img.draw_cross(best["cx"], best["cy"], color=COLOR_CROSS, size=8)
        # 用最小外接矩形的四个角画一个「贴合物体的斜框」，能直观看出长轴方向。
        try:
            corners = b.min_corners()
            if len(corners) == 4:
                for i in range(4):
                    p0 = corners[i]
                    p1 = corners[(i + 1) % 4]
                    img.draw_line(p0[0], p0[1], p1[0], p1[1],
                                  color=COLOR_ORI, thickness=2)
        except Exception:
            pass
        # 用 draw_string_advanced（旧 draw_string 在本固件上每帧打印弃用警告）。
        img.draw_string_advanced(best["x"], max(0, best["y"] - 16), 16,
                                 "black-block", color=COLOR_BOX)

    return best


# ============================ 独立运行 ============================

def main():
    sensor = None
    try:
        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=FRAME_W, height=FRAME_H)
        sensor.set_pixformat(Sensor.RGB565)

        Display.init(Display.VIRT, width=FRAME_W, height=FRAME_H, to_ide=True)
        MediaManager.init()
        sensor.run()

        clock = time.clock()
        # 丢掉开头的暗帧（sensor 刚启动那几帧 AE 未收敛，容易整帧偏暗）。
        for _ in range(10):
            os.exitpoint()
            sensor.snapshot()

        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot()

            target = detect_black_block(img, draw=True)

            if target:
                print("target cx=%d cy=%d w=%d h=%d ang=%d(ok=%s) solid=%.2f aspect=%.2f fps=%.1f"
                      % (target["cx"], target["cy"], target["w"], target["h"],
                         target["angle"], target["angle_ok"],
                         target["solidity"], target["aspect"], clock.fps()))
            else:
                print("no target  fps=%.1f" % clock.fps())

            Display.show_image(img)
            img = None
            gc.collect()

    except KeyboardInterrupt as e:
        print("用户停止:", e)
    except BaseException as e:
        import sys
        sys.print_exception(e)
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
