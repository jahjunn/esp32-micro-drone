"""板即机架 v7 DXF:
  中心: 只放得下飞控的圆角长条(胶囊) 30 x 76, 非实心大板
  电机: 单个圆 φ14.2 (按厂家图纸电机外径), 无多余参考圆
  孔  : 4x M2, φ9 螺栓圆(PCD), 钻孔 φ2.2
  对角轴距 100mm, 臂宽 10mm
  REF 层仅保留飞控(21x51) / IMU(20x16) 投影框, 便于 Layout 摆件。
单位 mm。依赖 shapely。
"""
import math
from shapely.geometry import box, LineString, Point, Polygon
from shapely.ops import unary_union

# ===================== PARAMS (mm) =====================
WHEELBASE   = 100.0   # 对角轴距
PAD_R       = 7.1     # 电机盘 φ14.2 (图纸电机外径) —— 唯一一个圆
MOTOR_PCD_R = 4.5     # 4xM2 螺栓圆半径 (φ9 PCD)
HOLE_R      = 1.1     # M2 通孔 φ2.2
CTR_W       = 30.0    # 中心长条宽
CTR_L       = 76.0    # 中心长条长
ARM_W       = 10.0    # 臂宽
BUF_RES     = 64
# ======================================================

D   = (WHEELBASE / 2.0) / math.sqrt(2.0)   # 35.355 => 盘心 (±D,±D)
OFF = MOTOR_PCD_R / math.sqrt(2.0)         # 3.182   => 孔心偏移
CORNERS = [(1,1),(-1,1),(-1,-1),(1,-1)]

# 中心: 胶囊 = 矩形(减去两端半径) + 半圆端
half_w, half_l = CTR_W/2.0, CTR_L/2.0
ctr = Polygon([(half_w, half_l-PAD_R*0+-(half_l-half_w)),
               (half_w, (half_l-half_w)),
               (-half_w, (half_l-half_w)),
               (-half_w, -(half_l-half_w))]) \
        .buffer(half_w, resolution=BUF_RES)

parts, holes, centers = [ctr], [], []
for (sx, sy) in CORNERS:
    cx, cy = D*sx, D*sy
    centers.append((cx, cy))
    parts.append(LineString([(0.0, 0.0), (cx, cy)]).buffer(ARM_W/2, resolution=BUF_RES))
    parts.append(Point(cx, cy).buffer(PAD_R, resolution=BUF_RES))
    for hx in (-1, 1):
        for hy in (-1, 1):
            holes.append((cx + hx*OFF, cy + hy*OFF))

board = unary_union(parts)
if hasattr(board, "geoms"):
    board = max(board.geoms, key=lambda g: g.area)
ext = list(board.exterior.coords)
if ext[0] == ext[-1]:
    ext = ext[:-1]

xs = [p[0] for p in ext]; ys = [p[1] for p in ext]
print("bbox x[%.2f,%.2f] y[%.2f,%.2f] => overall %.1f x %.1f mm"
      % (min(xs), max(xs), min(ys), max(ys), max(xs)-min(xs), max(ys)-min(ys)))
print("center strip %.0f x %.0f (capsule) | motor pad=phi%.1f x4 (single circle)"
      % (CTR_W, CTR_L, 2*PAD_R))
print("wheelbase=%.1f adjacent=%.1f" % (2*D*math.sqrt(2), 2*D))
print("hole pitch: adjacent=%.2f diagonal=%.2f PCD=phi%.1f hole=phi%.1f"
      % (2*OFF, 2*MOTOR_PCD_R, 2*MOTOR_PCD_R, 2*HOLE_R))
print("pad edge beyond hole = %.2f mm" % (PAD_R - (OFF*math.sqrt(2) + HOLE_R)))

# ===================== DXF (R2000) =====================
lines, h = [], 0xA
def handle():
    global h; v = "%X" % h; h += 1; return v

lines += ["0","SECTION","2","HEADER","9","$ACADVER","1","AC1015","0","ENDSEC"]
lines += ["0","SECTION","2","ENTITIES"]
lines += ["0","LWPOLYLINE","5",handle(),"8","BOARD","90",str(len(ext)),"70","1"]
for (x, y) in ext:
    lines += ["10","%.4f" % x,"20","%.4f" % y]
for (x, y) in holes:
    lines += ["0","CIRCLE","5",handle(),"8","HOLES",
              "10","%.4f" % x,"20","%.4f" % y,"30","0.0","40","%.4f" % HOLE_R]

# REF: 仅飞控与 IMU 投影框(方框, 非圆)
f411_cy, imu_cy = 10.0, -27.5
ref_polys = {
    "REF_F411":  list(box(-10.5, f411_cy-25.5, 10.5, f411_cy+25.5).exterior.coords),
    "REF_GY521": list(box(-10.0, imu_cy-8.0, 10.0, imu_cy+8.0).exterior.coords),
}
for name, pts in ref_polys.items():
    p = pts[:-1] if pts[0] == pts[-1] else pts
    lines += ["0","LWPOLYLINE","5",handle(),"8",name,"90",str(len(p)),"70","1"]
    for (x, y) in p:
        lines += ["10","%.4f" % x,"20","%.4f" % y]

lines += ["0","ENDSEC","0","EOF"]
with open("board_outline_v7.dxf", "w") as f:
    f.write("\n".join(lines) + "\n")
print("written board_outline_v7.dxf")
