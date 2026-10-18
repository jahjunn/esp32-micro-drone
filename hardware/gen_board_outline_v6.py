"""板即机架 v6 DXF (依据厂家 1104 图纸):
  中央 50x50 大圆角方岛 + 四角电机盘 φ18 + 臂 10mm。
  对角轴距 100mm。
  电机孔: 4x M2，分布在 φ9 螺栓圆(PCD)上  ->  孔心 (±3.182, ±3.182)
          (相邻孔距 6.36mm, 对角孔距 9.0mm)  钻孔 φ2.2
  REF 层: 电机外径 φ14.2 / 螺栓圆 φ9(虚线参考) / F411 / GY521。
  单位 mm。依赖 shapely。
"""
import math
from shapely.geometry import box, LineString, Point
from shapely.ops import unary_union

# ===================== PARAMS (mm) =====================
WHEELBASE   = 100.0   # 对角轴距
DISC_R      = 9.0     # 电机盘半径 φ18 (留电机 φ14.2 外 ~2mm 余量)
CTR_HALF    = 25.0    # 中央方岛半边长 => 50x50
CTR_RAD     = 10.0    # 方岛圆角
ARM_W       = 10.0    # 臂宽
MOTOR_PCD_R = 4.5     # 4xM2 螺栓圆半径 (φ9 PCD)
HOLE_R      = 1.1     # M2 通孔 φ2.2
BELL_R      = 7.1     # 电机外径 φ14.2 (参考)
BUF_RES     = 64
# ======================================================

D = (WHEELBASE / 2.0) / math.sqrt(2.0)   # 35.355 => 盘心 (±D,±D)
OFF = MOTOR_PCD_R / math.sqrt(2.0)       # 3.182   => 孔心 (±OFF,±OFF) 绕盘心
CORNERS = [(1,1),(-1,1),(-1,-1),(1,-1)]

ctr = box(-(CTR_HALF-CTR_RAD), -(CTR_HALF-CTR_RAD),
          (CTR_HALF-CTR_RAD), (CTR_HALF-CTR_RAD)).buffer(CTR_RAD, resolution=BUF_RES)

parts, holes, centers = [ctr], [], []
for (sx, sy) in CORNERS:
    cx, cy = D*sx, D*sy
    centers.append((cx, cy))
    parts.append(LineString([(0.0, 0.0), (cx, cy)]).buffer(ARM_W/2, resolution=BUF_RES))
    parts.append(Point(cx, cy).buffer(DISC_R, resolution=BUF_RES))
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
print("wheelbase=%.1f adjacent=%.1f disc=phi%.0f" % (2*D*math.sqrt(2), 2*D, 2*DISC_R))
print("hole pitch: adjacent=%.2f diagonal=%.2f  hole=phi%.1f  PCD=phi%.1f"
      % (2*OFF, 2*MOTOR_PCD_R, 2*HOLE_R, 2*MOTOR_PCD_R))
print("margin: disc->bell=%.2f  disc->hole_edge=%.2f  (bell phi%.1f)"
      % (DISC_R-BELL_R, DISC_R-(OFF*math.sqrt(2)+HOLE_R), 2*BELL_R))

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

# REF: 电机外径 + 螺栓圆
for (x, y) in centers:
    for r, layer in ((BELL_R, "REF_MOTOR"), (MOTOR_PCD_R, "REF_PCD")):
        lines += ["0","CIRCLE","5",handle(),"8",layer,
                  "10","%.4f" % x,"20","%.4f" % y,"30","0.0","40","%.4f" % r]

ref_polys = {
    "REF_CTR":   list(box(-CTR_HALF, -CTR_HALF, CTR_HALF, CTR_HALF).exterior.coords),
    "REF_F411":  list(box(-10.5, 4-25.5, 10.5, 4+25.5).exterior.coords),
    "REF_GY521": list(box(-10.0, -14-8, 10.0, -14+8).exterior.coords),
}
for name, pts in ref_polys.items():
    p = pts[:-1] if pts[0] == pts[-1] else pts
    lines += ["0","LWPOLYLINE","5",handle(),"8",name,"90",str(len(p)),"70","1"]
    for (x, y) in p:
        lines += ["10","%.4f" % x,"20","%.4f" % y]

lines += ["0","ENDSEC","0","EOF"]
with open("board_outline_v6.dxf", "w") as f:
    f.write("\n".join(lines) + "\n")
print("written board_outline_v6.dxf")
