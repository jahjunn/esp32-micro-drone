"""板即机架 v5 DXF: 中央 50x50 大圆角方岛 + 四角电机盘(φ20, 收小留余量) + 臂10mm。
对角轴距 100mm；电机孔 1104: 4x M2, 9x9mm 方阵, 钻孔 φ2.2。
REF 层附: 中央岛边界 / F411 / GY521 / 电机底座 φ14 轮廓(目视余量用)。
单位 mm。依赖 shapely。
"""
import math
from shapely.geometry import box, LineString, Point
from shapely.ops import unary_union

# ===================== PARAMS (mm) =====================
WHEELBASE  = 100.0   # 对角轴距(对角电机中心距)
DISC_R     = 10.0    # 电机盘半径 φ20  <-- v5 收小(原 φ22)
CTR_HALF   = 25.0    # 中央方岛半边长 => 50x50
CTR_RAD    = 10.0    # 方岛圆角半径
ARM_W      = 10.0    # 臂身宽
MOTOR_HOLE = 4.5     # 1104: 9x9mm 方阵 -> 半距 4.5
HOLE_R     = 1.1     # M2 通孔 φ2.2 半径
MOTOR_OD_R = 7.0     # 电机底座轮廓半径(参考) φ14
BUF_RES    = 64
# ======================================================

D = (WHEELBASE / 2.0) / math.sqrt(2.0)   # 35.355 => 电机盘心 (±D,±D)
CORNERS = [(1,1),(-1,1),(-1,-1),(1,-1)]

ctr = box(-(CTR_HALF-CTR_RAD), -(CTR_HALF-CTR_RAD),
          (CTR_HALF-CTR_RAD), (CTR_HALF-CTR_RAD)).buffer(CTR_RAD, resolution=BUF_RES)

parts = [ctr]
holes = []
motor_centers = []
for (sx, sy) in CORNERS:
    cx, cy = D * sx, D * sy
    motor_centers.append((cx, cy))
    arm  = LineString([(0.0, 0.0), (cx, cy)]).buffer(ARM_W/2, resolution=BUF_RES)
    disc = Point(cx, cy).buffer(DISC_R, resolution=BUF_RES)
    parts.append(arm); parts.append(disc)
    for hx in (-1, 1):
        for hy in (-1, 1):
            holes.append((cx + hx*MOTOR_HOLE, cy + hy*MOTOR_HOLE))

board = unary_union(parts)
if hasattr(board, "geoms"):
    board = max(board.geoms, key=lambda g: g.area)

ext = list(board.exterior.coords)
if ext[0] == ext[-1]:
    ext = ext[:-1]

xs = [p[0] for p in ext]; ys = [p[1] for p in ext]
edge_margin = DISC_R - (MOTOR_HOLE*math.sqrt(2) + HOLE_R)
print("bbox x[%.2f,%.2f] y[%.2f,%.2f] => overall %.1f x %.1f mm"
      % (min(xs), max(xs), min(ys), max(ys),
         max(xs)-min(xs), max(ys)-min(ys)))
print("wheelbase=%.1f adjacent=%.1f disc=phi%.0f hole_pitch=%.1f hole=phi%.1f"
      % (2*D*math.sqrt(2), 2*D, 2*DISC_R, 2*MOTOR_HOLE, 2*HOLE_R))
print("disc edge margin beyond hole = %.2f mm ; motor_base(phi14) margin = %.2f mm"
      % (edge_margin, DISC_R - MOTOR_OD_R))

# ===================== DXF (R2000) =====================
lines = []
h = 0xA
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

for (x, y) in motor_centers:
    lines += ["0","CIRCLE","5",handle(),"8","REF_MOTOR",
              "10","%.4f" % x,"20","%.4f" % y,"30","0.0","40","%.4f" % MOTOR_OD_R]

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
with open("board_outline_v5.dxf", "w") as f:
    f.write("\n".join(lines) + "\n")
print("written board_outline_v5.dxf")
