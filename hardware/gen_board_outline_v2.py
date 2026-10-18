"""板即机架 v2 DXF: 无外框 X 异形 + 四角 φ22 电机盘 + 中央胶囊岛 36x70 + 臂10mm。
包络 100x100 (嘉立创免费档)。单位 mm。依赖 shapely。
"""
import math
from shapely.geometry import box, LineString, Point
from shapely.ops import unary_union

# ===================== PARAMS (mm) =====================
BOUND     = 50.0     # 包络半边 => 100x100
CAP_W     = 36.0     # 胶囊宽
CAP_LEN   = 70.0     # 胶囊总长
ARM_W     = 10.0     # 臂身宽
DISC_R    = 11.0     # 电机安装盘半径 (φ22)
DISC_C    = 39.0     # 盘心 ±DISC_C (±39,±39) => 盘压在100x100边内
MOTOR_HOLE= 6.0      # 1104 12x12 M2 方阵半距
HOLE_R    = 1.0      # M2 钻孔半径(直径2.0)
BUF_RES   = 64       # 圆弧细分
# ======================================================

# 中央胶囊 (box 缓冲 => 圆角端胶囊)
straight = (CAP_LEN - CAP_W) / 2.0      # 17.0
cap = box(-CAP_W/2, -straight, CAP_W/2, straight).buffer(CAP_W/2, resolution=BUF_RES)

parts = [cap]
for (sx, sy) in [(1,1),(-1,1),(-1,-1),(1,-1)]:
    cx, cy = DISC_C * sx, DISC_C * sy          # 盘心坐标(±39,±39) => 压在100x100边内
    arm  = LineString([(0.0, 0.0), (cx, cy)]).buffer(ARM_W/2, resolution=BUF_RES)
    disc = Point(cx, cy).buffer(DISC_R, resolution=BUF_RES)
    parts.append(arm); parts.append(disc)

board = unary_union(parts)
if hasattr(board, "geoms"):
    board = max(board.geoms, key=lambda g: g.area)   # 取最大连通块

ext = list(board.exterior.coords)
if ext[0] == ext[-1]:
    ext = ext[:-1]

# 电机安装孔 (4 电机 x 4)
holes = []
for (sx, sy) in [(1,1),(-1,1),(-1,-1),(1,-1)]:
    cx, cy = DISC_C * sx, DISC_C * sy
    for hx in (-1, 1):
        for hy in (-1, 1):
            holes.append((cx + hx * MOTOR_HOLE, cy + hy * MOTOR_HOLE))

# 参考层: 胶囊 / F411(竖21x51, 居中偏上) / GY521(20x16, 下方)
ref_polys = {
    "CAP":  list(box(-CAP_W/2, -CAP_LEN/2, CAP_W/2, CAP_LEN/2).exterior.coords),
    "F411": list(box(-10.5, 7-25.5, 10.5, 7+25.5).exterior.coords),
    "GY521":list(box(-10.0, -22-8, 10.0, -22+8).exterior.coords),
}

xs = [p[0] for p in ext]; ys = [p[1] for p in ext]
print("bbox x[%.2f,%.2f] y[%.2f,%.2f]" % (min(xs), max(xs), min(ys), max(ys)))
print("board vertices=%d  motor_holes=%d" % (len(ext), len(holes)))

# ===================== DXF (R2000) =====================
lines = []
h = 0xA
def handle():
    global h; v = "%X" % h; h += 1; return v

lines += ["0","SECTION","2","HEADER","9","$ACADVER","1","AC1015","0","ENDSEC"]
lines += ["0","SECTION","2","ENTITIES"]

# BOARD 板框 (闭合 LWPOLYLINE)
lines += ["0","LWPOLYLINE","5",handle(),"8","BOARD","90",str(len(ext)),"70","1"]
for (x, y) in ext:
    lines += ["10","%.4f" % x,"20","%.4f" % y]

# HOLES 电机孔 (CIRCLE)
for (x, y) in holes:
    lines += ["0","CIRCLE","5",handle(),"8","HOLES",
              "10","%.4f" % x,"20","%.4f" % y,"30","0.0","40","%.4f" % HOLE_R]

# REF 参考框 (闭合 LWPOLYLINE)
for name, pts in ref_polys.items():
    p = pts[:-1] if pts[0] == pts[-1] else pts
    lines += ["0","LWPOLYLINE","5",handle(),"8","REF","90",str(len(p)),"70","1"]
    for (x, y) in p:
        lines += ["10","%.4f" % x,"20","%.4f" % y]

lines += ["0","ENDSEC","0","EOF"]
with open("board_outline_v2.dxf", "w") as f:
    f.write("\n".join(lines) + "\n")
print("written board_outline_v2.dxf")
