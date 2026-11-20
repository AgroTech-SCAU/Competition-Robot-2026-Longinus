from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Arc, Circle, FancyArrowPatch, FancyBboxPatch, Rectangle


OUT = Path(__file__).with_name("current_laser_base_link_topview.png")


def choose_cjk_font():
    preferred = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Source Han Sans SC"]
    installed = {font.name for font in font_manager.fontManager.ttflist}
    for name in preferred:
        if name in installed:
            return name
    return "DejaVu Sans"


plt.rcParams["font.family"] = choose_cjk_font()
plt.rcParams["axes.unicode_minus"] = False

fig, ax = plt.subplots(figsize=(15, 12), dpi=160)
fig.patch.set_facecolor("#f5f7fb")
ax.set_facecolor("#ffffff")
ax.set_aspect("equal")
ax.set_xlim(-285, 285)
ax.set_ylim(-275, 255)
ax.axis("off")

ax.text(0, 238, "当前底盘、四路激光与 IMU 坐标定义", ha="center", va="center",
        fontsize=24, fontweight="bold", color="#172033")
ax.text(0, 216, "俯视 · 遥控器／运动学右手系 · 非比例图 · IMU 位置仅为示意，不代表实测安装点",
        ha="center", va="center", fontsize=12.5, color="#667085")

# 车身外缘与突出物仅表达已有机械记录，不作为精确包络测量。
body = FancyBboxPatch((-132, -132), 264, 264,
                      boxstyle="round,pad=0,rounding_size=18",
                      linewidth=2, edgecolor="#7d8aa5", facecolor="#edf1f7",
                      linestyle=(0, (6, 4)), zorder=1)
ax.add_patch(body)
ax.text(-119, 119, "车身外缘（示意）", ha="left", va="top", fontsize=10, color="#697586")
ax.add_patch(FancyBboxPatch((-54, 130), 108, 39, boxstyle="round,pad=0,rounding_size=7",
                            facecolor="#fff1d6", edgecolor="#d98b18", linewidth=1.5, zorder=2))
ax.text(0, 150, "电池突出 40 mm", ha="center", va="center", fontsize=10, color="#9a5b00", zorder=4)
ax.add_patch(FancyBboxPatch((130, -42), 22, 84, boxstyle="round,pad=0,rounding_size=5",
                            facecolor="#f2e9ff", edgecolor="#865ac2", linewidth=1.5, zorder=2))
ax.text(143, 0, "气泵", ha="center", va="center", rotation=90, fontsize=10, color="#6941a5", zorder=4)

wheel_positions = {
    "FL": (-96, 96),
    "FR": (96, 96),
    "RR": (96, -96),
    "RL": (-96, -96),
}
for name, (x, y) in wheel_positions.items():
    ax.add_patch(Rectangle((x - 25, y - 15), 50, 30,
                           facecolor="#344054", edgecolor="#101828", linewidth=1.3, zorder=5))
    ax.text(x, y, name, ha="center", va="center", fontsize=12,
            fontweight="bold", color="white", zorder=6)

ax.annotate("", xy=(-96, 67), xytext=(96, 67),
            arrowprops=dict(arrowstyle="<->", color="#526077", lw=1.2))
ax.text(-4, 56, "192 mm 左右舵向轴间距", ha="center", va="top", fontsize=10, color="#526077")
ax.annotate("", xy=(-67, -96), xytext=(-67, 96),
            arrowprops=dict(arrowstyle="<->", color="#526077", lw=1.2))
ax.text(-56, -6, "192 mm\n前后舵向轴间距", ha="left", va="center", fontsize=10, color="#526077")

# base_link 与运动学坐标轴。
ax.add_patch(Circle((0, 0), 8, facecolor="#1570ef", edgecolor="white", linewidth=2, zorder=8))
ax.text(-13, 15, "base_link\n四轮舵向轴几何中心", ha="right", va="bottom",
        fontsize=11, color="#175cd3", fontweight="bold", zorder=8)
ax.add_patch(FancyArrowPatch((0, 0), (0, 92), arrowstyle="-|>", mutation_scale=19,
                             linewidth=2.7, color="#1570ef", zorder=7))
ax.text(12, 87, "+vx（车体 x）\n实际车头", ha="left", va="center", fontsize=12,
        color="#175cd3", fontweight="bold")
ax.add_patch(FancyArrowPatch((0, 0), (-92, 0), arrowstyle="-|>", mutation_scale=19,
                             linewidth=2.7, color="#12b76a", zorder=7))
ax.text(-84, -17, "+vy（车体 y）\n运动学左侧", ha="center", va="top", fontsize=12,
        color="#027a48", fontweight="bold")
ax.add_patch(Arc((0, 0), 62, 62, theta1=25, theta2=305, color="#f04438", lw=2.0, zorder=7))
ax.add_patch(FancyArrowPatch((26, -16), (31, -2), arrowstyle="-|>", mutation_scale=13,
                             color="#f04438", lw=2, zorder=7))
ax.text(38, -34, "+wz 逆时针\n+Z 垂直纸面向外", ha="left", va="top",
        fontsize=10.5, color="#b42318")

# IMU 模块和轴向仅表示当前软件假设；真实安装点与方向尚未测量。
imu_xy = (69, 39)
ax.add_patch(FancyBboxPatch((45, 24), 48, 34, boxstyle="round,pad=0.2,rounding_size=4",
                            facecolor="#e7f6ff", edgecolor="#0077b6", linewidth=2, zorder=8))
ax.text(*imu_xy, "BMI088\nIMU", ha="center", va="center", fontsize=10,
        color="#005f8f", fontweight="bold", zorder=9)
ax.add_patch(FancyArrowPatch((88, 50), (88, 72), arrowstyle="-|>", mutation_scale=11,
                             linewidth=1.7, color="#1570ef", zorder=9))
ax.add_patch(FancyArrowPatch((52, 50), (30, 50), arrowstyle="-|>", mutation_scale=11,
                             linewidth=1.7, color="#12b76a", zorder=9))
ax.text(94, 70, "xI", fontsize=9, color="#175cd3", fontweight="bold")
ax.text(27, 56, "yI", fontsize=9, color="#027a48", fontweight="bold")
ax.add_patch(FancyArrowPatch((8, 8), (43, 27), arrowstyle="-|>", mutation_scale=13,
                             linewidth=1.7, linestyle=(0, (4, 3)), color="#7a5af8", zorder=8))
ax.text(112, 88, "IMU（示意位置）\n代码假设：R_B_I = I\nxI∥+vx，yI∥+vy，zI∥+Z\n平移 r_B_I 未标定，代码按 0",
        ha="left", va="center", fontsize=10.2, color="#344054", linespacing=1.35,
        bbox=dict(boxstyle="round,pad=0.45", facecolor="#f0f9ff", edgecolor="#38bdf8"), zorder=10)

# 激光标记表示方向；圆点坐标不是测得的发射点外参。
def laser(name, xy, direction, color, detail, text_xy, align="center"):
    x, y = xy
    ax.add_patch(Circle(xy, 10, facecolor=color, edgecolor="white", linewidth=2, zorder=9))
    ax.text(x, y, name, ha="center", va="center", fontsize=10, color="white",
            fontweight="bold", zorder=10)
    dx, dy = direction
    ax.add_patch(FancyArrowPatch((x + dx * 12, y + dy * 12), (x + dx * 70, y + dy * 70),
                                 arrowstyle="-|>", mutation_scale=18, linewidth=2.5,
                                 color=color, zorder=8))
    ax.text(text_xy[0], text_xy[1], detail, ha=align, va="center", fontsize=10.6,
            color=color, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor=color, alpha=0.96),
            zorder=11)


laser("B", (0, 112), (0, 1), "#d92d20",
      "LASER_REAR = +vx\n实际车头 · 外缘预留记录 56 mm", (156, 166), "center")
laser("F", (0, -112), (0, -1), "#7a5af8",
      "LASER_FRONT = -vx\n实际车尾 · 外缘预留记录 16 mm", (133, -157), "center")
laser("R", (-112, 0), (-1, 0), "#039855",
      "LASER_RIGHT = +vy\n运动学左侧 · 外缘预留记录 16 mm", (-207, 78), "center")
laser("L", (112, 0), (1, 0), "#dc6803",
      "LASER_LEFT = -vy\n运动学右侧 · 外缘预留记录 26 mm", (205, -79), "center")

ax.add_patch(FancyArrowPatch((0, -176), (0, -221), arrowstyle="-|>", mutation_scale=21,
                             linewidth=3, color="#475467", zorder=7))
ax.text(13, -211, "摆车与入场：以 -vx 倒车", ha="left", va="center", fontsize=11.5,
        color="#344054", fontweight="bold")

note = (
    "场地路线坐标：+X = +vy，+Y = -vx（正常右手旋转）\n"
    "当前 Auto：轮速积分位置与路线进度；IMU 独立航向保持；激光负责横移、就位与安全逃离。\n"
    "轮速位姿不被 IMU／激光回写，当前不运行轮速＋IMU Kalman 位置融合。"
)
ax.text(-270, -262, note, ha="left", va="bottom", fontsize=10.4, color="#344054",
        linespacing=1.5,
        bbox=dict(boxstyle="round,pad=0.6", facecolor="#f8fafc", edgecolor="#cbd5e1"))
ax.text(270, -262,
        "外参现状\n• 四路发射点相对 base_link 的坐标尚未实测\n• 上图预留量是机械记录，不是坐标变换\n• 当前控制直接比较原始距离，未扣除 setback",
        ha="right", va="bottom", fontsize=9.8, color="#6941c6", linespacing=1.4,
        bbox=dict(boxstyle="round,pad=0.55", facecolor="#f9f5ff", edgecolor="#d6bbfb"))

plt.savefig(OUT, bbox_inches="tight", facecolor=fig.get_facecolor())
print(OUT.resolve())
