# -*- coding: utf-8 -*-
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
ents = model.active_entities
ents.clear!

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
# 实现要点：SketchUp 中圆柱 = 圆面 + pushpull(高度) + 平移到起点
def cyl(ents, name, cx, cy, cz, r, h, axis = :z, color = nil)
  grp = ents.add_group
  g = grp.entities
  # 圆面始终画在 XY 平面（法线 +Z），再按需旋转移到位
  circle = g.add_circle([0, 0, 0], [0, 0, 1], r.mm, 24)
  face = g.add_face(circle)
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


# ---- 轮系 ----
# 轮径 65.0mm，轮心高 32.5mm，轮距中心 88.0mm
cyl(ents, "Wheel_L", 40.0, -101.0, 32.5, 32.5, 26.0, :y, mat_rubber)
cyl(ents, "Wheel_R", 40.0, 75.0, 32.5, 32.5, 26.0, :y, mat_rubber)

# ---- 电机（JGB37-520）----
cyl(ents, "Motor_L", 40.0, -93.0, 32.5, 18.0, 30.0, :y, mat_alu)
cyl(ents, "Motor_R", 40.0, 93.0, 32.5, 18.0, 30.0, :y, mat_alu)

# ---- 万向球（前后各一）----
cyl(ents, "Caster_F", 85.0, 0, 12.0, 8.0, 24.0, :z, mat_alu)
cyl(ents, "Caster_B", -85.0, 0, 12.0, 8.0, 24.0, :z, mat_alu)

# ---- 三层板 ----
box_c(ents, "BasePlate", 0, 0, 46.5, 190.0, 150.0, 4.0, mat_body)
box_c(ents, "MidPlate", 0, 0, 70.0, 170.0, 140.0, 3.0, mat_body)
box_c(ents, "TopPlate", 0, 0, 98.0, 160.0, 130.0, 3.0, mat_body)

# ---- 铜柱 ----
cyl(ents, "Standoff1_0", 80.0, -63.0, 48.5, 3.0, 20.0, :z, mat_alu)
cyl(ents, "Standoff1_1", 80.0, 63.0, 48.5, 3.0, 20.0, :z, mat_alu)
cyl(ents, "Standoff1_2", -80.0, -63.0, 48.5, 3.0, 20.0, :z, mat_alu)
cyl(ents, "Standoff1_3", -80.0, 63.0, 48.5, 3.0, 20.0, :z, mat_alu)

# ---- 推板（盒式：宽开口 + 侧裙）----
# 推板宽 195.0mm，比车体 150.0mm 宽 22.5mm/侧（悬伸，提升贴墙容错）
box(ents, "Jaw_Back", 95.0, -97.5, 1.5, 3.0, 195.0, 40.0, mat_jaw)
box(ents, "Skirt_L", 95.0, -97.5, 1.5, 30.0, 3.0, 30.0, mat_jaw)
box(ents, "Skirt_R", 95.0, 94.5, 1.5, 30.0, 3.0, 30.0, mat_jaw)

# ---- 相机 ----
# 相机光心高度 150mm（几何计算基准）
# 前视俯角 15.0deg；下视俯角 60.0deg（不是 90，见 docs/02 §5）
# ⚠️ 需立柱 50.5mm：三层板顶面 z=100，相机在 z=150
cyl(ents, "CamMast_F", 80.0, 0, 99.5, 5.0, 50.5, :z, mat_alu)
cyl(ents, "CamMast_D", 100.0, 0, 99.5, 5.0, 50.5, :z, mat_alu)
box_c(ents, "Cam_Front", 80.0, 0, 150.0, 25.0, 25.0, 20.0, mat_cam)
box_c(ents, "Cam_Down", 100.0, 0, 150.0, 25.0, 25.0, 20.0, mat_cam)

# ---- 电子件占位 ----
box_c(ents, "Battery", 50.0, 0, 81.5, 70.0, 35.0, 20.0, mat_bat)
box_c(ents, "VisionBoard_RK3588", 0, 0, 107.0, 90.0, 65.0, 15.0, mat_pcb)
box_c(ents, "MCU_H723", -20.0, 0, 107.5, 60.0, 45.0, 12.0, mat_pcb)
box_c(ents, "Driver_L", -60.0, -30.0, 77.5, 50.0, 25.0, 12.0, mat_pcb)
box_c(ents, "Driver_R", -60.0, 30.0, 77.5, 50.0, 25.0, 12.0, mat_pcb)

# ---- 激光模组（合规装备，低位向前）----
box_c(ents, "Laser", 90.0, 0, 15.0, 12.0, 12.0, 10.0, mat_cam)


model.commit_operation
Sketchup.status_text = "Robot built. Check Outliner for parts."

# ---- 参数记录（便于追溯）----------------------------------------------
PARAMS = {"limit_L": 250.0, "limit_W": 200.0, "limit_H": 180.0, "limit_mass": 1500.0, "body_L": 190.0, "body_W": 150.0, "plate_t": 3.0, "base_t": 4.0, "jaw_W": 195.0, "jaw_H": 40.0, "jaw_t": 3.0, "skirt_h": 30.0, "skirt_t": 3.0, "jaw_gap": 1.5, "jaw_opening": 165.0, "wheel_d": 65.0, "wheel_w": 26.0, "wheel_x": 40.0, "standoff_1": 20.0, "standoff_2": 25.0, "cam_tilt_front": 15.0, "cam_tilt_down": 60.0, "cam_h": 150.0, "battery": (70.0, 35.0, 20.0), "vision_board": (90.0, 65.0, 15.0), "mcu_board": (60.0, 45.0, 12.0), "driver": (50.0, 25.0, 12.0), "k230_style_cam": (25.0, 25.0, 20.0)}
puts "=== 建模完成 ==="
puts "外廓: 223.0 x 195.0 x 160.0 mm"
puts "上限: 250.0 x 200.0 x 180.0 mm"
PARAMS.each { |k, v| puts "  #{k} = #{v}" }
