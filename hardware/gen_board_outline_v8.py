"""板即机架 v8 DXF:
  中心: 一块素板(50x50 大圆角方岛), 内部不画任何参考线
  电机: 只有一个圆 φ14.2 (按图纸电机外径); 4xM2 孔在 φ9 螺栓圆上, 钻孔 φ2.2
  对角轴距 100mm, 臂宽 10mm
  输出仅 2 个图层: BOARD(板框) + HOLES(钻孔)。无 REF。
单位 mm。依赖 shapely。
"""
import math
from shapely.geometry import box, LineString, Point
from shapely.ops import unary_union

# ===================== PARAMS (mm) =====================
WHEELBASE   = 100.0   # 对角轴距
PAD_R       = 7.1     # 电机圆 φ14.2 (图纸电机外径) —— 唯一一个圆
MOTOR_PCD_R = 4.5     # 4xM2 螺栓圆半径 (φ9 PCD)
HOLE_R      = 1.1     # M2 通孔 φ2.2
CTR_HALF    = 25.0    # 中心素板半边长 => 50x50
CTR_RAD     = 10.0    # 素板圆角半径
ARM_W       = 10.0    # 臂宽
BUF_RES     = 64
# ======================================================

D   = (WHEELBASE / 2.0) / math.sqrt(2.0)   # 35.355 => 电机圆心 (±D,±D)
OFF = MOTOR_PCD_R / math.sqrt(2.0)         # 3.182   => 孔心偏移
CORNERS = [(1,1),(-1,1),(-1,-1),(1,-1)]

ctr = box(-(CTR_HALF-CTR_RAD), -(CTR_HALF-CTR_RAD),
          (CTR_HALF-CTR_RAD), (CTR_HALF-CTR_RAD)).buffer(CTR_RAD, resolution=BUF_RES)

parts, holes = [ctr], []
for (sx, sy) in CORNERS:
    cx, cy = D*sx, D*sy
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
print("center solid pad %.0f x %.0f (no inner lines) | motor circle phi%.1f x4"
      % (2*CTR_HALF, 2*CTR_HALF, 2*PAD_R))
print("wheelbase=%.1f adjacent=%.1f" % (2*D*math.sqrt(2), 2*D))
print("hole pitch: adjacent=%.2f diagonal=%.2f PCD=phi%.1f hole=phi%.1f"
      % (2*OFF, 2*MOTOR_PCD_R, 2*MOTOR_PCD_R, 2*HOLE_R))
print("pad edge beyond hole = %.2f mm  (pad=phi%.1f, screw head phi3.8 needs %.2f)"
      % (PAD_R-(OFF*math.sqrt(2)+HOLE_R), 2*PAD_R, OFF*math.sqrt(2)+1.9))

# ===================== DXF (R2000): BOARD + HOLES only =====================
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
lines += ["0","ENDSEC","0","EOF"]
with open("board_outline_v8.dxf", "w") as f:
    f.write("\n".join(lines) + "\n")
print("written board_outline_v8.dxf  (layers: BOARD, HOLES)")
