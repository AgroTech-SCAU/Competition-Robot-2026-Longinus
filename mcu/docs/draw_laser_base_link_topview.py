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

fig, ax = plt.subplots(figsize=(14, 11), dpi=160)
fig.patch.set_facecolor("#f5f7fb")
ax.set_facecolor("#ffffff")
ax.set_aspect("equal")
ax.set_xlim(-265, 265)
ax.set_ylim(-255, 245)
ax.axis("off")

# Title and explicit drawing status.
ax.text(0, 228, "四舵轮底盘与四路激光俯视示意图", ha="center", va="center",
        fontsize=24, fontweight="bold", color="#172033")
ax.text(0, 207, "以遥控器／运动学右手系为准 · 非比例图 · 激光 base_link 坐标尚未实测",
        ha="center", va="center", fontsize=12.5, color="#667085")

# Nominal outer body edges are schematic; only steering-axis spacing is known.
body = FancyBboxPatch((-132, -132), 264, 264,
                      boxstyle="round,pad=0,rounding_size=18",
                      linewidth=2, edgecolor="#7d8aa5", facecolor="#edf1f7",
                      linestyle=(0, (6, 4)), zorder=1)
ax.add_patch(body)
ax.text(108, 112, "车身外缘（示意）", ha="right", va="top", fontsize=10, color="#697586")

# Battery and air-pump protrusions, matching the comments rather than measured envelopes.
ax.add_patch(FancyBboxPatch((-54, 130), 108, 39, boxstyle="round,pad=0,rounding_size=7",
                            facecolor="#fff1d6", edgecolor="#d98b18", linewidth=1.5, zorder=2))
ax.text(0, 150, "电池突出 40 mm", ha="center", va="center", fontsize=10, color="#9a5b00", zorder=4)
ax.add_patch(FancyBboxPatch((130, -42), 22, 84, boxstyle="round,pad=0,rounding_size=5",
                            facecolor="#f2e9ff", edgecolor="#865ac2", linewidth=1.5, zorder=2))
ax.text(143, 0, "气泵", ha="center", va="center", rotation=90, fontsize=10, color="#6941a5", zorder=4)

# Steering module positions are the only known geometric coordinates: +/-96 mm.
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

# Known steering-axis spacing dimensions.
ax.annotate("", xy=(-96, 67), xytext=(96, 67),
            arrowprops=dict(arrowstyle="<->", color="#526077", lw=1.2))
ax.text(0, 55, "192 mm 左右舵向轴间距", ha="center", va="top", fontsize=10, color="#526077")
ax.annotate("", xy=(-67, -96), xytext=(-67, 96),
            arrowprops=dict(arrowstyle="<->", color="#526077", lw=1.2))
ax.text(-56, 0, "192 mm\n前后舵向轴间距", ha="left", va="center", fontsize=10, color="#526077")

# Base link and the standard right-handed body frame.
ax.add_patch(Circle((0, 0), 8, facecolor="#1570ef", edgecolor="white", linewidth=2, zorder=8))
ax.text(13, 13, "base_link\n四轮舵向轴几何中心", ha="left", va="bottom",
        fontsize=11, color="#175cd3", fontweight="bold", zorder=8)
ax.add_patch(FancyArrowPatch((0, 0), (0, 92), arrowstyle="-|>", mutation_scale=19,
                             linewidth=2.7, color="#1570ef", zorder=7))
ax.text(12, 86, "+vx（车体 x）\n实际车头", ha="left", va="center", fontsize=12,
        color="#175cd3", fontweight="bold")
ax.add_patch(FancyArrowPatch((0, 0), (-92, 0), arrowstyle="-|>", mutation_scale=19,
                             linewidth=2.7, color="#12b76a", zorder=7))
ax.text(-84, -17, "+vy（车体 y）\n运动学左侧", ha="center", va="top", fontsize=12,
        color="#027a48", fontweight="bold")

# CCW yaw indicator and +Z out of the page.
ax.add_patch(Arc((0, 0), 62, 62, theta1=25, theta2=305, color="#f04438", lw=2.0, zorder=7))
ax.add_patch(FancyArrowPatch((26, -16), (31, -2), arrowstyle="-|>", mutation_scale=13,
                             color="#f04438", lw=2, zorder=7))
ax.text(38, -32, "+wz 逆时针\n+Z 垂直纸面向外", ha="left", va="top",
        fontsize=10.5, color="#b42318")

# Sensor markers are deliberately labelled schematic; no measured base_link coordinates exist.
def laser(name, xy, direction, color, detail, text_xy, align="center"):
    x, y = xy
    ax.add_patch(Circle(xy, 10, facecolor=color, edgecolor="white", linewidth=2, zorder=9))
    ax.text(x, y, name, ha="center", va="center", fontsize=10, color="white",
            fontweight="bold", zorder=10)
    dx, dy = direction
    ax.add_patch(FancyArrowPatch((x + dx * 12, y + dy * 12), (x + dx * 70, y + dy * 70),
                                 arrowstyle="-|>", mutation_scale=18, linewidth=2.5,
                                 color=color, zorder=8))
    ax.text(text_xy[0], text_xy[1], detail, ha=align, va="center", fontsize=11,
            color=color, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor=color, alpha=0.96),
            zorder=11)


laser("B", (0, 112), (0, 1), "#d92d20",
      "LASER_REAR = +vx\n实际车头 · 外缘预留 56 mm", (150, 155), "center")
laser("F", (0, -112), (0, -1), "#7a5af8",
      "LASER_FRONT = -vx\n实际车尾 · 外缘预留 16 mm", (125, -165), "center")
laser("R", (-112, 0), (-1, 0), "#039855",
      "LASER_RIGHT = +vy\n运动学左侧 · 外缘预留 16 mm", (-198, 69), "center")
laser("L", (112, 0), (1, 0), "#dc6803",
      "LASER_LEFT = -vy\n运动学右侧 · 外缘预留 26 mm", (202, -69), "center")

# Reverse entry direction and corrected proper-rotation field relationship.
ax.add_patch(FancyArrowPatch((0, -160), (0, -229), arrowstyle="-|>", mutation_scale=21,
                             linewidth=3, color="#475467", zorder=7))
ax.text(13, -220, "倒车进入地图：-vx", ha="left", va="center", fontsize=12,
        color="#344054", fontweight="bold")

note = (
    "统一后的方向关系\n"
    "场地 +Y = -vx（倒车入场）\n"
    "场地 +X = +vy（与车体构成正常右手旋转）\n\n"
    "当前 Auto：纯轮速导航，激光只记录、不参与控制。\n"
    "外缘净间隙 = 原始测距 - 对应 setback；传感器中心坐标需实测。"
)
ax.text(-248, -238, note, ha="left", va="bottom", fontsize=10.5, color="#344054",
        linespacing=1.45,
        bbox=dict(boxstyle="round,pad=0.65", facecolor="#f8fafc", edgecolor="#cbd5e1"))

plt.savefig(OUT, bbox_inches="tight", facecolor=fig.get_facecolor())
print(OUT.resolve())
