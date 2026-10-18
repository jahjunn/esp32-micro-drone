"""生成轻量无人机飞控「板即机架」异形板框 DXF (R2000)。
坐标单位: mm。可调参数见 PARAMS。
"""
import math

# ===================== PARAMS (mm) =====================
R_MOTOR = 50.0     # 电机中心半径 (轴距100 => 对角100 => 半50)
R_TIP   = 56.0     # 臂端外沿半径 (电机中心 + 余量)
A_ARM   = 6.0      # 臂半宽 => 臂宽12mm
C_HALF  = 25.0     # 中心岛半边长 => 50x50 (再叠加高度)
IX_W, IX_H = 50.0, 55.0   # 中心飞控岛参考尺寸 (mm)
MOTOR_HOLE = 6.0   # 1104 安装孔 ±6 (12x12 M2 方阵)
HOLE_R  = 1.0      # M2 钻孔半径 (直径2.0mm)
DAMP    = 20.0     # 中心减震柱孔 ±20 (M2)
# ======================================================

def attach(cth, sth, offset_sign):
    """求臂长边(偏移 offset_sign*a)与中心方板的边界交点(取最靠臂端者)。"""
    cands = []
    # x = +C
    if abs(cth) > 1e-9:
        t = (C_HALF + offset_sign * A_ARM * sth) / cth
        y = t * sth + offset_sign * A_ARM * cth
        if -C_HALF - 1e-6 <= y <= C_HALF + 1e-6 and t > 0:
            cands.append((t, (C_HALF, y)))
    # x = -C
    if abs(cth) > 1e-9:
        t = (-C_HALF + offset_sign * A_ARM * sth) / cth
        y = t * sth + offset_sign * A_ARM * cth
        if -C_HALF - 1e-6 <= y <= C_HALF + 1e-6 and t > 0:
            cands.append((t, (-C_HALF, y)))
    # y = +C
    if abs(sth) > 1e-9:
        t = (C_HALF - offset_sign * A_ARM * cth) / sth
        x = t * cth - offset_sign * A_ARM * sth
        if -C_HALF - 1e-6 <= x <= C_HALF + 1e-6 and t > 0:
            cands.append((t, (x, C_HALF)))
    # y = -C
    if abs(sth) > 1e-9:
        t = (-C_HALF - offset_sign * A_ARM * cth) / sth
        x = t * cth - offset_sign * A_ARM * sth
        if -C_HALF - 1e-6 <= x <= C_HALF + 1e-6 and t > 0:
            cands.append((t, (x, -C_HALF)))
    cands.sort(key=lambda q: q[0])
    return cands[-1][1]

# ---- 板框 16 顶点 (CCW) ----
outline = []
for k in range(4):
    th = math.radians(45 + 90 * k)
    cth, sth = math.cos(th), math.sin(th)
    px, py = -sth, cth                      # 垂直(CCW)单位向量
    tip = (R_TIP * cth, R_TIP * sth)
    tip_low  = (tip[0] - A_ARM * px, tip[1] - A_ARM * py)
    tip_high = (tip[0] + A_ARM * px, tip[1] + A_ARM * py)
    al = attach(cth, sth, -1.0)
    ah = attach(cth, sth, +1.0)
    outline.append(al)
    outline.append(tip_low)
    outline.append(tip_high)
    outline.append(ah)

# ---- 电机安装孔 (4 电机 x 4 孔) ----
motor_holes = []
for k in range(4):
    th = math.radians(45 + 90 * k)
    mc = (R_MOTOR * math.cos(th), R_MOTOR * math.sin(th))
    for dx in (-MOTOR_HOLE, MOTOR_HOLE):
        for dy in (-MOTOR_HOLE, MOTOR_HOLE):
            motor_holes.append((mc[0] + dx, mc[1] + dy))

# ---- 中心减震柱孔 (4 孔) ----
damp_holes = [(dx, dy) for dx in (-DAMP, DAMP) for dy in (-DAMP, DAMP)]

# ---- 参考层: 中心岛矩形 + F411 + GY-521 ----
ix = [(-IX_W/2, -IX_H/2), (IX_W/2, -IX_H/2),
      (IX_W/2, IX_H/2), (-IX_W/2, IX_H/2)]
f411 = [(-10.5, -25.5), (10.5, -25.5), (10.5, 25.5), (-10.5, 25.5)]
gy521 = [(-12, -8), (8, -8), (8, 8), (-12, 8)]

# ===================== 写 DXF (R2000) =====================
def w(out, s):
    out.append(s)

lines = []
h = 0xA
def handle():
    global h
    v = "%X" % h
    h += 1
    return v

w(lines, "0"); w(lines, "SECTION")
w(lines, "2"); w(lines, "HEADER")
w(lines, "9"); w(lines, "$ACADVER")
w(lines, "1"); w(lines, "AC1015")
w(lines, "0"); w(lines, "ENDSEC")
w(lines, "0"); w(lines, "SECTION")
w(lines, "2"); w(lines, "ENTITIES")

# 板框 LWPOLYLINE (闭合) -> layer BOARD
w(lines, "0"); w(lines, "LWPOLYLINE")
w(lines, "5"); w(lines, handle())
w(lines, "8"); w(lines, "BOARD")
w(lines, "90"); w(lines, str(len(outline)))
w(lines, "70"); w(lines, "1")     # 闭合
for (x, y) in outline:
    w(lines, "10"); w(lines, "%.4f" % x)
    w(lines, "20"); w(lines, "%.4f" % y)

# 电机孔 + 减震孔 -> CIRCLE layer HOLES
for (x, y) in motor_holes + damp_holes:
    w(lines, "0"); w(lines, "CIRCLE")
    w(lines, "5"); w(lines, handle())
    w(lines, "8"); w(lines, "HOLES")
    w(lines, "10"); w(lines, "%.4f" % x)
    w(lines, "20"); w(lines, "%.4f" % y)
    w(lines, "30"); w(lines, "0.0")
    w(lines, "40"); w(lines, "%.4f" % HOLE_R)

# 参考矩形 -> LINE layer REF
for rect in (ix, f411, gy521):
    pts = rect + [rect[0]]
    for i in range(len(pts) - 1):
        w(lines, "0"); w(lines, "LINE")
        w(lines, "5"); w(lines, handle())
        w(lines, "8"); w(lines, "REF")
        w(lines, "10"); w(lines, "%.4f" % pts[i][0])
        w(lines, "20"); w(lines, "%.4f" % pts[i][1])
        w(lines, "30"); w(lines, "0.0")
        w(lines, "11"); w(lines, "%.4f" % pts[i+1][0])
        w(lines, "21"); w(lines, "%.4f" % pts[i+1][1])
        w(lines, "31"); w(lines, "0.0")

w(lines, "0"); w(lines, "ENDSEC")
w(lines, "0"); w(lines, "EOF")

with open("board_outline_v1.dxf", "w") as f:
    f.write("\n".join(lines) + "\n")

print("OK vertices=%d motor_holes=%d damp=%d" %
      (len(outline), len(motor_holes), len(damp_holes)))
print("first outline pt:", outline[0], "last:", outline[-1])
