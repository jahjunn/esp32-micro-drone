"""板即机架 v3 DXF: 无外框 X 异形 + 四角 φ22 电机盘 + 中央 50x50 大圆角方岛 + 臂10mm。
对角轴距 = 100mm。包络 ~93x93 (≤100x100 嘉立创免费档)。单位 mm。依赖 shapely。
"""
import math
from shapely.geometry import box, LineString, Point
from shapely.ops import unary_union

# ===================== PARAMS (mm) =====================
WHEELBASE = 100.0    # 对角轴距(对角电机中心距)
DISC_R    = 11.0     # 电机安装盘半径 (φ22)
CTR_HALF  = 25.0     # 中央方岛半边长 => 50x50
CTR_RAD   = 10.0     # 方岛圆角半径
ARM_W     = 10.0     # 臂身宽
MOTOR_HOLE= 6.0      # 1104 12x12 M2 方阵半距
HOLE_R    = 1.0      # M2 钻孔半径(直径2.0)
BUF_RES   = 64
# ======================================================

D = (WHEELBASE / 2.0) / math.sqrt(2.0)   # 35.355 => 电机盘心 (±D,±D)
CORNERS = [(1,1),(-1,1),(-1,-1),(1,-1)]

# 中央大圆角方岛: box(half=CTR_HALF-CTR_RAD).buffer(CTR_RAD)
ctr = box(-(CTR_HALF-CTR_RAD), -(CTR_HALF-CTR_RAD),
          (CTR_HALF-CTR_RAD), (CTR_HALF-CTR_RAD)).buffer(CTR_RAD, resolution=BUF_RES)

parts = [ctr]
holes = []
for (sx, sy) in CORNERS:
    cx, cy = D * sx, D * sy
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
print("bbox x[%.2f,%.2f] y[%.2f,%.2f] => overall %.1f x %.1f mm"
      % (min(xs), max(xs), min(ys), max(ys),
         max(xs)-min(xs), max(ys)-min(ys)))
print("wheelbase=%.1f  board vertices=%d  motor_holes=%d"
      % (2*D*math.sqrt(2), len(ext), len(holes)))

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

# REF: 中央岛边界 + F411(竖21x51 偏上) + GY521(20x16 偏下)
ref_polys = {
    "CTR":  list(box(-CTR_HALF, -CTR_HALF, CTR_HALF, CTR_HALF).exterior.coords),
    "F411": list(box(-10.5, 4-25.5, 10.5, 4+25.5).exterior.coords),
    "GY521":list(box(-10.0, -14-8, 10.0, -14+8).exterior.coords),
}
for name, pts in ref_polys.items():
    p = pts[:-1] if pts[0] == pts[-1] else pts
    lines += ["0","LWPOLYLINE","5",handle(),"8","REF","90",str(len(p)),"70","1"]
    for (x, y) in p:
        lines += ["10","%.4f" % x,"20","%.4f" % y]

lines += ["0","ENDSEC","0","EOF"]
with open("board_outline_v3.dxf", "w") as f:
    f.write("\n".join(lines) + "\n")
print("written board_outline_v3.dxf")
