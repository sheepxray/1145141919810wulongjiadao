#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SketchUp 模型生成器（参数化）
=============================

用途：生成 SketchUp 可执行的 Ruby 建模脚本，并校验尺寸自洽性。

**为什么用生成器而不是手写脚本**：
五轮迭代中尺寸改过多次（车宽、推板、主控、相机）。参数化后
改一个数字就能重新生成整套模型，而不是维护一份会过时的死图。

用法：
    python tools/sketchup_model.py                 # 校验 + 生成
    python tools/sketchup_model.py --layout        # 只看三视图

输出：
    hardware/mechanical/sketchup/build_robot.rb    # SketchUp Ruby 脚本

在 SketchUp 中运行：
    窗口 -> Ruby 控制台 -> 粘贴脚本内容 -> 回车
    或：扩展程序 -> Ruby 控制台 -> load '路径/build_robot.rb'
"""

import argparse
import math
import os
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


# ==========================================================================
# ★ 全部设计参数集中在这里 —— 改这里，重新生成即可
# ==========================================================================

P = {
    # ---- 赛题约束 ----
    "limit_L": 250.0,          # 发车尺寸上限 mm
    "limit_W": 200.0,
    "limit_H": 180.0,
    "limit_mass": 1500.0,      # g

    # ---- 车体（第 1 轮确定的"车体做窄"方案）----
    "body_L": 190.0,           # 车体长
    "body_W": 150.0,           # ★ 车体宽（刻意做窄，见 docs/02 §3.2）
    "plate_t": 3.0,            # 板厚
    "base_t": 4.0,             # 底板厚（承力）

    # ---- 推板（盒式：宽开口 + 侧裙 + 柔性贴地）----
    "jaw_W": 195.0,            # ★ 推板宽（接近 200 上限）
    "jaw_H": 40.0,             # 推板高
    "jaw_t": 3.0,              # 推板厚
    "skirt_h": 30.0,           # 侧裙高
    "skirt_t": 3.0,
    "jaw_gap": 1.5,            # 推板离地间隙
    "jaw_opening": 165.0,      # 开口宽度

    # ---- 轮系 ----
    "wheel_d": 65.0,
    "wheel_w": 26.0,
    "wheel_x": 40.0,           # 轮中心距车体中心的前后偏移

    # ---- 板层间距 ----
    "standoff_1": 20.0,        # 底板->中层板
    "standoff_2": 25.0,        # 中层板->上层板

    # ---- 相机 ----
    "cam_tilt_front": 15.0,    # 前视俯角（deg）
    "cam_tilt_down": 60.0,     # ★ 下视俯角（不是 90，见 docs/02 §5）
    "cam_h": 150.0,            # 相机安装高度（光心离地）

    # ---- 器件尺寸（用于占位方块）----
    "battery": (70.0, 35.0, 20.0),      # 3S 1500mAh
    "vision_board": (90.0, 65.0, 15.0),  # RK3588 板
    "mcu_board": (60.0, 45.0, 12.0),     # STM32H723 核心板
    "driver": (50.0, 25.0, 12.0),        # BTS7960
    "k230_style_cam": (25.0, 25.0, 20.0),  # IMX219 模块
}


# ==========================================================================
# 派生尺寸
# ==========================================================================

class Layout:
    def __init__(self, p):
        self.p = p
        self.compute()

    def compute(self):
        p = self.p

        # 坐标约定：x 向前（推板方向），y 向右，z 向上，原点在车体中心地面投影
        # ---- 轮 ----
        self.wheel_r = p["wheel_d"] / 2.0
        self.wheel_z = self.wheel_r                       # 轮心高度
        self.wheel_x = p["wheel_x"]
        self.wheel_y = p["body_W"] / 2.0 + p["wheel_w"] / 2.0

        # ---- 板层高度 ----
        self.base_z0 = self.wheel_z + 12.0                # 底板底面（轮心上方）
        self.base_z1 = self.base_z0 + p["base_t"]
        self.mid_z0 = self.base_z1 + p["standoff_1"]
        self.mid_z1 = self.mid_z0 + p["plate_t"]
        self.top_z0 = self.mid_z1 + p["standoff_2"]
        self.top_z1 = self.top_z0 + p["plate_t"]

        # ---- 推板 ----
        self.jaw_x0 = p["body_L"] / 2.0                   # 推板后缘（贴车体前）
        self.jaw_x1 = self.jaw_x0 + p["jaw_t"] + 30.0     # 推板前缘（含弧形段）
        self.jaw_y = p["jaw_W"] / 2.0
        self.jaw_z0 = p["jaw_gap"]
        self.jaw_z1 = self.jaw_z0 + p["jaw_H"]

        # 侧裙（推板两侧向后延伸）
        self.skirt_x0 = self.jaw_x0
        self.skirt_x1 = self.jaw_x1 - p["jaw_t"]
        self.skirt_y = self.jaw_y - p["skirt_t"] / 2.0

        # ---- 相机位置 ----
        # ⚠️ 相机光心高度 cam_h 是全部几何计算的基准（见 docs/02 §5、docs/14）：
        #    挡板可见性、汉字像素数、可见地面区间都基于它。
        #    三层板顶面在 top_z1，若 cam_h 高于它，需要一根**立柱**把相机举高。
        self.cam_mast_h = max(p["cam_h"] - self.top_z1, 0.0)

        self.cam_front_x = p["body_L"] / 2.0 - 15.0   # 车头
        self.cam_front_z = p["cam_h"]
        self.cam_down_x = self.jaw_x0 + 5.0           # 推板正上方
        self.cam_down_z = p["cam_h"]

        # ---- 外廓（含推板与相机立柱）----
        self.total_L = self.jaw_x1 - (-p["body_L"] / 2.0)
        self.total_W = max(p["body_W"], p["jaw_W"])
        # 总高必须含相机顶面，否则尺寸校验会漏掉立柱
        cam_top = p["cam_h"] + p["k230_style_cam"][2] / 2.0
        self.total_H = max(cam_top, self.jaw_z1 + 5.0)

    def report(self):
        p = self.p
        L, W, H = self.total_L, self.total_W, self.total_H
        rows = [
            ("长", L, p["limit_L"]),
            ("宽", W, p["limit_W"]),
            ("高", H, p["limit_H"]),
        ]
        print(f"  {'方向':<6} {'设计mm':>10} {'上限mm':>10} {'余量mm':>10} {'状态':>8}")
        print("  " + "-" * 48)
        ok = True
        for name, v, lim in rows:
            m = lim - v
            st = "OK" if m > 0 else "**超限**"
            if m <= 0:
                ok = False
            print(f"  {name:<6} {v:>10.1f} {lim:>10.1f} {m:>10.1f} {st:>8}")
        return ok


# ==========================================================================
# Ruby 生成
# ==========================================================================

RUBY_HEADER = '''# -*- coding: utf-8 -*-
# =============================================================================
# 光电赛智能车 — SketchUp 建模脚本（自动生成，请勿手改）
# =============================================================================
#
# 生成器: tools/sketchup_model.py
# 重新生成: python tools/sketchup_model.py
#
# 在 SketchUp 2026 中运行:
#   方式1: 窗口 -> Ruby 控制台 -> 粘贴全文 -> 回车
#   方式2: 扩展程序 -> Ruby 控制台 -> load '<绝对路径>/build_robot.rb'
#
# 坐标约定: x 向前(推板方向), y 向右, z 向上, 原点在车体中心的地面投影
#
# 生成时间参数见文件末尾的 PARAMS 哈希
# =============================================================================

model = Sketchup.active_model
model.start_operation("Build Robot", true)

# 清空旧模型（若重复运行）
# ⚠️ 注意：这会删除当前模型里的所有内容。请在一个**新建的空模型**里运行。
ents = model.active_entities
if ents.count > 0
  puts "提示: 当前模型非空（#{ents.count} 个实体），将清空后重建。"
  puts "      若不想清空，请先新建一个空白模型。"
  ents.clear!
end

# ---- 工具函数 ----------------------------------------------------------

# 创建一个长方体（以最小角 + 尺寸定义）
def box(ents, name, x0, y0, z0, dx, dy, dz, color = nil)
  grp = ents.add_group
  g = grp.entities
  pts = [
    [x0,      y0,      z0],
    [x0 + dx, y0,      z0],
    [x0 + dx, y0 + dy, z0],
    [x0,      y0 + dy, z0],
  ].map { |p| p.map { |v| v.mm } }
  face = g.add_face(pts)
  face.pushpull(dz.mm)
  grp.name = name
  if color
    grp.material = color
  end
  grp
end

# 以中心 + 尺寸创建
def box_c(ents, name, cx, cy, cz, dx, dy, dz, color = nil)
  box(ents, name, cx - dx/2.0, cy - dy/2.0, cz - dz/2.0, dx, dy, dz, color)
end

# 圆柱（用于电机/轮子/立柱）
# 实现要点：
#   1. 圆面画在 XY 平面（法线 +Z），pushpull 出高度
#   2. 再按 axis 旋转 + 平移到目标位置
# 兼容性注意：不同 SketchUp 版本的 add_circle 行为不同 ——
#   有的会自动创建面，有的只创建边。这里两种情况都处理。
def cyl(ents, name, cx, cy, cz, r, h, axis = :z, color = nil)
  grp = ents.add_group
  g = grp.entities

  circle = g.add_circle([0, 0, 0], [0, 0, 1], r.mm, 24)

  # add_circle 可能已自动建面；若没有则手动建
  face = nil
  g.each { |e| face = e if e.is_a?(Sketchup::Face) }
  if face.nil?
    face = g.add_face(circle)
  end
  if face.nil?
    puts "警告: 无法创建圆柱面 (#{name})"
    return grp
  end
  face.pushpull(h.mm)

  # 按轴旋转并平移
  tr = Geom::Transformation.new
  case axis
  when :z
    tr = Geom::Transformation.translation([cx.mm, cy.mm, cz.mm])
  when :y
    # 绕 X 轴转 -90 度，使柱轴指向 +Y
    rot = Geom::Transformation.rotation([0, 0, 0], [1, 0, 0], -90.degrees)
    tr = Geom::Transformation.translation([cx.mm, cy.mm, cz.mm]) * rot
  when :x
    # 绕 Y 轴转 +90 度，使柱轴指向 +X
    rot = Geom::Transformation.rotation([0, 0, 0], [0, 1, 0], 90.degrees)
    tr = Geom::Transformation.translation([cx.mm, cy.mm, cz.mm]) * rot
  end
  grp.transform!(tr)

  grp.name = name
  grp.material = color if color
  grp
end

# ---- 材质 --------------------------------------------------------------
mat_body   = model.materials.add("Carbon")
mat_body.color = Sketchup::Color.new(60, 60, 65)
mat_alu    = model.materials.add("ALU")
mat_alu.color = Sketchup::Color.new(180, 185, 195)
mat_pcb    = model.materials.add("PCB")
mat_pcb.color = Sketchup::Color.new(20, 90, 50)
mat_bat    = model.materials.add("Battery")
mat_bat.color = Sketchup::Color.new(40, 60, 140)
mat_rubber = model.materials.add("Rubber")
mat_rubber.color = Sketchup::Color.new(35, 35, 35)
mat_cam    = model.materials.add("Cam")
mat_cam.color = Sketchup::Color.new(150, 40, 40)
mat_jaw    = model.materials.add("Jaw")
mat_jaw.color = Sketchup::Color.new(200, 150, 60)

'''

RUBY_FOOTER = '''
model.commit_operation
Sketchup.status_text = "Robot built. Check Outliner for parts."

# ---- 参数记录（便于追溯）----------------------------------------------
PARAMS = {params}
puts "=== 建模完成 ==="
puts "外廓: {L:.1f} x {W:.1f} x {H:.1f} mm"
puts "上限: {limL:.1f} x {limW:.1f} x {limH:.1f} mm"
PARAMS.each {{ |k, v| puts "  #{{k}} = #{{v}}" }}
'''


def gen_ruby(L: Layout):
    p = L.p
    lines = [RUBY_HEADER]

    def w(s=""):
        lines.append(s)

    w("# ---- 轮系 ----")
    w(f"# 轮径 {p['wheel_d']}mm，轮心高 {L.wheel_z:.1f}mm，轮距中心 {L.wheel_y:.1f}mm")
    # 轮子沿 +Y 方向挤出，左侧轮心需减去轮宽，使两轮关于 y=0 对称
    for side, y_start in (("L", -L.wheel_y - p["wheel_w"] / 2.0),
                          ("R",  L.wheel_y - p["wheel_w"] / 2.0)):
        w(f'cyl(ents, "Wheel_{side}", '
          f'{L.wheel_x:.1f}, {y_start:.1f}, {L.wheel_z:.1f}, '
          f'{L.wheel_r:.1f}, {p["wheel_w"]:.1f}, :y, mat_rubber)')
    w()
    w("# ---- 电机（JGB37-520）----")
    for side, y in (("L", -L.wheel_y - 5.0), ("R", L.wheel_y + 5.0)):
        w(f'cyl(ents, "Motor_{side}", '
          f'{L.wheel_x:.1f}, {y:.1f}, {L.wheel_z:.1f}, 18.0, 30.0, :y, mat_alu)')
    w()
    w("# ---- 万向球（前后各一）----")
    for tag, x in (("F", p["body_L"]/2 - 10), ("B", -p["body_L"]/2 + 10)):
        w(f'cyl(ents, "Caster_{tag}", {x:.1f}, 0, 12.0, 8.0, 24.0, :z, mat_alu)')
    w()

    w("# ---- 三层板 ----")
    w(f'box_c(ents, "BasePlate", 0, 0, {(L.base_z0 + L.base_z1)/2:.1f}, '
      f'{p["body_L"]:.1f}, {p["body_W"]:.1f}, {p["base_t"]:.1f}, mat_body)')
    w(f'box_c(ents, "MidPlate", 0, 0, {(L.mid_z0 + L.mid_z1)/2:.1f}, '
      f'{p["body_L"]-20:.1f}, {p["body_W"]-10:.1f}, {p["plate_t"]:.1f}, mat_body)')
    w(f'box_c(ents, "TopPlate", 0, 0, {(L.top_z0 + L.top_z1)/2:.1f}, '
      f'{p["body_L"]-30:.1f}, {p["body_W"]-20:.1f}, {p["plate_t"]:.1f}, mat_body)')
    w()

    w("# ---- 铜柱 ----")
    for i, (x, y) in enumerate([
        (p["body_L"]/2-15, -p["body_W"]/2+12),
        (p["body_L"]/2-15,  p["body_W"]/2-12),
        (-p["body_L"]/2+15, -p["body_W"]/2+12),
        (-p["body_L"]/2+15,  p["body_W"]/2-12),
    ]):
        w(f'cyl(ents, "Standoff1_{i}", {x:.1f}, {y:.1f}, '
          f'{L.base_z1:.1f}, 3.0, {p["standoff_1"]:.1f}, :z, mat_alu)')
    w()

    w("# ---- 推板（盒式：宽开口 + 侧裙）----")
    w(f"# 推板宽 {p['jaw_W']}mm，比车体 {p['body_W']}mm 宽 "
      f"{(p['jaw_W']-p['body_W'])/2:.1f}mm/侧（悬伸，提升贴墙容错）")
    w(f'box(ents, "Jaw_Back", {L.jaw_x0:.1f}, {-L.jaw_y:.1f}, {L.jaw_z0:.1f}, '
      f'{p["jaw_t"]:.1f}, {p["jaw_W"]:.1f}, {p["jaw_H"]:.1f}, mat_jaw)')
    # 两侧侧裙
    for side, sy in (("L", -L.skirt_y - p["skirt_t"]/2), ("R", L.skirt_y - p["skirt_t"]/2)):
        w(f'box(ents, "Skirt_{side}", {L.skirt_x0:.1f}, {sy:.1f}, {L.jaw_z0:.1f}, '
          f'{L.skirt_x1-L.skirt_x0:.1f}, {p["skirt_t"]:.1f}, {p["skirt_h"]:.1f}, mat_jaw)')
    w()

    w("# ---- 相机 ----")
    w(f"# 相机光心高度 {p['cam_h']:.0f}mm（几何计算基准）")
    w(f"# 前视俯角 {p['cam_tilt_front']}deg；下视俯角 {p['cam_tilt_down']}deg"
      f"（不是 90，见 docs/02 §5）")
    if L.cam_mast_h > 1.0:
        w(f"# ⚠️ 需立柱 {L.cam_mast_h:.1f}mm：三层板顶面 z={L.top_z1:.0f}，"
          f"相机在 z={p['cam_h']:.0f}")
        w(f'cyl(ents, "CamMast_F", {L.cam_front_x:.1f}, 0, '
          f'{L.top_z1:.1f}, 5.0, {L.cam_mast_h:.1f}, :z, mat_alu)')
        w(f'cyl(ents, "CamMast_D", {L.cam_down_x:.1f}, 0, '
          f'{L.top_z1:.1f}, 5.0, {L.cam_mast_h:.1f}, :z, mat_alu)')
    else:
        w("# 相机直接装在三层板上，无需立柱")
    w(f'box_c(ents, "Cam_Front", {L.cam_front_x:.1f}, 0, {L.cam_front_z:.1f}, '
      f'{p["k230_style_cam"][0]:.1f}, {p["k230_style_cam"][1]:.1f}, '
      f'{p["k230_style_cam"][2]:.1f}, mat_cam)')
    w(f'box_c(ents, "Cam_Down", {L.cam_down_x:.1f}, 0, {L.cam_down_z:.1f}, '
      f'{p["k230_style_cam"][0]:.1f}, {p["k230_style_cam"][1]:.1f}, '
      f'{p["k230_style_cam"][2]:.1f}, mat_cam)')
    w()

    w("# ---- 电子件占位 ----")
    bw, bd, bh = p["battery"]
    w(f'box_c(ents, "Battery", {p["body_L"]/2 - 45:.1f}, 0, '
      f'{L.mid_z1 + bh/2:.1f}, {bw:.1f}, {bd:.1f}, {bh:.1f}, mat_bat)')

    vw, vd, vh = p["vision_board"]
    w(f'box_c(ents, "VisionBoard_RK3588", 0, 0, {L.top_z1 + vh/2:.1f}, '
      f'{vw:.1f}, {vd:.1f}, {vh:.1f}, mat_pcb)')
    w(f'box_c(ents, "MCU_H723", {-20:.1f}, 0, {L.top_z1 + p["mcu_board"][2]/2 + 2:.1f}, '
      f'{p["mcu_board"][0]:.1f}, {p["mcu_board"][1]:.1f}, '
      f'{p["mcu_board"][2]:.1f}, mat_pcb)')

    dw, dd, dh = p["driver"]
    for side, y in (("L", -30.0), ("R", 30.0)):
        w(f'box_c(ents, "Driver_{side}", {-60:.1f}, {y:.1f}, '
          f'{L.mid_z1 + dh/2:.1f}, {dw:.1f}, {dd:.1f}, {dh:.1f}, mat_pcb)')
    w()

    w("# ---- 激光模组（合规装备，低位向前）----")
    w(f'box_c(ents, "Laser", {p["body_L"]/2 - 5:.1f}, 0, 15.0, 12.0, 12.0, 10.0, mat_cam)')
    w()

    lines.append(RUBY_FOOTER.format(
        params=repr({k: v for k, v in p.items()}).replace("'", '"'),
        L=L.total_L, W=L.total_W, H=L.total_H,
        limL=p["limit_L"], limW=p["limit_W"], limH=p["limit_H"],
    ))
    return "\n".join(lines)


# ==========================================================================
# ASCII 三视图（终端校验用）
# ==========================================================================

def draw_layout(L: Layout):
    p = L.p
    print("\n" + "=" * 78)
    print("俯视图（上=前）")
    print("=" * 78)
    W = 66
    scale = W / max(L.total_L, L.total_W * 2.2)

    def sx(x):
        return int((x + p["body_L"]/2) * scale) + 2

    print()
    # 推板
    jaw_row = int(L.jaw_y * 2 * scale)
    print(" " * sx(-p["body_L"]/2) + "┌" + "─" * jaw_row + "┐")
    lbl = f"推板 {p['jaw_W']:.0f}mm (悬伸 {(p['jaw_W']-p['body_W'])/2:.1f}/侧)"
    print(" " * sx(-p["body_L"]/2) + "│" + lbl.center(jaw_row) + "│")
    print(" " * sx(-p["body_L"]/2) + "└" + "─" * jaw_row + "┘")
    print()
    # 车体
    body_row = int(p["body_W"] * scale)
    print(" " * sx(-p["body_L"]/2) + "┌" + "─" * body_row + "┐")
    for txt in [
        f"车体 {p['body_L']:.0f} x {p['body_W']:.0f}mm",
        "○              ○   <- 轮",
        "  [RK3588] [H723]",
        "  [BTS7960]x2  [电池]",
    ]:
        print(" " * sx(-p["body_L"]/2) + "│" + txt.center(body_row) + "│")
    print(" " * sx(-p["body_L"]/2) + "└" + "─" * body_row + "┘")
    print()

    print("=" * 78)
    print("侧视图（右=前）")
    print("=" * 78)
    print()
    mast = (f"  ← 立柱 {L.cam_mast_h:.0f}mm" if L.cam_mast_h > 1.0 else "  ← 直装")
    print(f"   相机   z={p['cam_h']:.0f} ──────────────────────  [相机]{mast}")
    print(f"   上层板 z={L.top_z1:.0f} ──────────────────────  [RK3588]")
    print(f"   中层板 z={L.mid_z0:.0f} ────────────────  [BTS7960] [电池]")
    print(f"   底板   z={L.base_z0:.0f} ──────────────────────────")
    print(f"   轮心   z={L.wheel_z:.0f}    ○                       ○")
    print(f"   地面   z=0   ──────────────────────────────────")
    print(f"   推板 z={L.jaw_z0:.1f}~{L.jaw_z1:.0f}  ┃▓▓▓┃  ← 离地 {p['jaw_gap']}mm")
    print()
    print(f"   总高 {L.total_H:.1f}mm（上限 {p['limit_H']}，余量 "
          f"{p['limit_H']-L.total_H:.1f}）")
    if L.cam_mast_h > 1.0:
        print(f"   ⚠️ 相机需 {L.cam_mast_h:.0f}mm 立柱 —— 这是外廓的主要来源，")
        print(f"      而相机高度 {p['cam_h']:.0f}mm 是几何计算的基准，不能降低。")


# ==========================================================================
# 主程序
# ==========================================================================

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", action="store_true", help="只显示三视图")
    ap.add_argument("--out", default=None, help="输出路径")
    args = ap.parse_args()

    L = Layout(P)

    print("=" * 78)
    print("SketchUp 模型生成器")
    print("=" * 78)

    print("\n[1] 尺寸自洽性校验")
    print()
    ok = L.report()

    print("\n[2] 关键尺寸")
    print()
    rows = [
        ("车体", f"{P['body_L']:.0f} x {P['body_W']:.0f} x {P['base_t']:.0f}"),
        ("推板", f"{P['jaw_W']:.0f} x {P['jaw_H']:.0f} x {P['jaw_t']:.0f}"),
        ("推板悬伸", f"{(P['jaw_W']-P['body_W'])/2:.1f} / 侧"),
        ("贴墙容错窗口", f"{(45.0/2)+(P['jaw_W']-P['body_W'])/2-5:.1f}"),
        ("轮径", f"{P['wheel_d']:.0f}"),
        ("相机高度", f"{P['cam_h']:.0f}"),
        ("前视俯角", f"{P['cam_tilt_front']:.0f} deg"),
        ("下视俯角", f"{P['cam_tilt_down']:.0f} deg"),
        ("三层板高度", f"{L.base_z0:.0f} / {L.mid_z0:.0f} / {L.top_z0:.0f}"),
    ]
    for k, v in rows:
        print(f"   {k:<16} {v}")

    if args.layout:
        draw_layout(L)
        return

    # 生成 Ruby
    ruby = gen_ruby(L)
    out = args.out or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "hardware", "mechanical", "sketchup", "build_robot.rb")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write(ruby)

    print(f"\n[3] 已生成 SketchUp 脚本")
    print(f"   {out}")
    print(f"   {len(ruby.splitlines())} 行")

    print("\n[4] 使用方法")
    print("   方式1: SketchUp -> 窗口 -> Ruby 控制台 -> 粘贴全文 -> 回车")
    print(f"   方式2: SketchUp -> 扩展程序 -> Ruby 控制台 -> load '{out}'")

    print("\n[5] 三视图")
    draw_layout(L)

    if not ok:
        print("\n⚠️ 外廓超限！请调整参数")
        sys.exit(1)

    print("\n" + "=" * 78)
    print("尺寸自洽，脚本已生成。")
    print("=" * 78)


if __name__ == "__main__":
    main()
