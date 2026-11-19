#!/usr/bin/env python3
"""Draw the current map_route as a wheel-only open-field validation route."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, Rectangle

from test_fast_route import parse_route


OUT = Path(__file__).resolve().parents[2] / "docs/wheel_only_route.png"


def choose_cjk_font():
    installed = {font.name for font in font_manager.fontManager.ttflist}
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Source Han Sans SC"):
        if name in installed:
            return name
    return "DejaVu Sans"


plt.rcParams["font.family"] = choose_cjk_font()
plt.rcParams["axes.unicode_minus"] = False

legs = parse_route()
colors = {"-vx": "#1570ef", "+vx": "#dc6803", "+vy": "#039855", "-vy": "#d92d20"}

fig = plt.figure(figsize=(14, 11), dpi=160, facecolor="#f5f7fb")
ax = fig.add_axes([0.065, 0.09, 0.60, 0.79])
ax.set_facecolor("white")
ax.set_aspect("equal")
ax.set_xlim(-0.65, 3.45)
ax.set_ylim(-1.05, 4.85)
ax.set_xlabel("场地 X / m（+X = +vy）", fontsize=12)
ax.set_ylabel("场地 Y / m（+Y = -vx）", fontsize=12)
ax.grid(True, color="#e4e7ec", linewidth=0.8)
ax.set_axisbelow(True)

# Nominal field and row centres; the route itself is the subject of this test.
ax.add_patch(Rectangle((0, 0), 3.0, 4.55, facecolor="#f8fafc",
                       edgecolor="#98a2b3", linewidth=1.4, linestyle="--", zorder=0))
for row in range(1, 8):
    y = 0.2 + 0.7 * (row - 1)
    ax.plot([0.0, 3.0], [y, y], color="#eaecf0", linewidth=1.0, zorder=0)
    ax.text(3.04, y, f"沟 {row}", va="center", fontsize=9, color="#98a2b3")

previous = (0.2, -0.5)
segments = []
for index, leg in enumerate(legs, 1):
    start = previous
    end = (leg.x, leg.y)
    dx, dy = end[0] - start[0], end[1] - start[1]
    command = ("+vy" if dx > 0 else "-vy") if dx else ("-vx" if dy > 0 else "+vx")
    distance = abs(dx) + abs(dy)
    color = colors[command]

    # Slight display offsets keep opposite-direction segments readable.
    offset = (0.022 if index % 2 else -0.022)
    if dx:
        display_start = (start[0], start[1] + offset)
        display_end = (end[0], end[1] + offset)
        label_xy = ((start[0] + end[0]) / 2, start[1] + (0.10 if index % 2 else -0.10))
    else:
        display_start = (start[0] + offset, start[1])
        display_end = (end[0] + offset, end[1])
        label_xy = (start[0] + (0.13 if index % 2 else -0.13), (start[1] + end[1]) / 2)
    ax.add_patch(FancyArrowPatch(display_start, display_end, arrowstyle="-|>",
                                 mutation_scale=14, linewidth=2.0, color=color,
                                 shrinkA=3, shrinkB=3, zorder=3))
    ax.text(*label_xy, f"{index:02d}  {command}", ha="center", va="center",
            fontsize=8.5, color=color, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.15", facecolor="white", edgecolor="none", alpha=0.85),
            zorder=4)
    segments.append((index, command, distance, leg.speed, start, end))
    previous = end

ax.scatter([0.2], [-0.5], s=90, marker="o", color="#175cd3", edgecolor="white", zorder=6)
ax.text(0.08, -0.58, "起点/终点\n(0.20, -0.50)", ha="right", va="top", fontsize=10,
        color="#175cd3", fontweight="bold")

# Launch orientation: actual front points out of the field; entry is reverse.
ax.add_patch(FancyArrowPatch((0.2, -0.5), (0.2, -0.94), arrowstyle="-|>",
                             mutation_scale=16, linewidth=2.2, color="#dc6803", zorder=5))
ax.text(0.30, -0.86, "+vx 实际车头朝场外", ha="left", va="center", fontsize=9.5, color="#b54708")
ax.add_patch(FancyArrowPatch((0.2, -0.5), (0.2, -0.08), arrowstyle="-|>",
                             mutation_scale=16, linewidth=2.2, color="#1570ef", zorder=5))
ax.text(0.30, -0.15, "-vx 倒车入场", ha="left", va="center", fontsize=9.5, color="#175cd3")
ax.add_patch(FancyArrowPatch((0.2, -0.5), (0.62, -0.5), arrowstyle="-|>",
                             mutation_scale=16, linewidth=2.2, color="#039855", zorder=5))
ax.text(0.60, -0.63, "+vy", ha="center", va="top", fontsize=9.5, color="#027a48")

ax.legend(handles=[Line2D([0], [0], color=color, lw=2.5, label=command)
                   for command, color in colors.items()],
          loc="upper left", frameon=True, fontsize=10, title="车体速度方向")

fig.suptitle("纯轮速里程计空旷场地验证路线", x=0.37, y=0.955,
             fontsize=24, fontweight="bold", color="#172033")
fig.text(0.37, 0.915, "16 段完整往返 · 箭头旁为段号和车体命令 · 激光仅记录、不参与控制",
         ha="center", fontsize=11.5, color="#667085")

fig.text(0.70, 0.875, "测试配置", fontsize=17, fontweight="bold", color="#172033")
config_text = (
    "理论路线总长：30.600 m\n"
    "理想运动时间：约 103 s（不含实车误差）\n"
    "起点：X=0.20 m，Y=-0.50 m\n"
    "摆车：实际车头 +vx 朝场外\n"
    "入场：车尾 LASER_FRONT 朝场内，-vx 倒车\n\n"
    "速度：首段 0.35，横跨 0.30 m/s\n"
    "换沟 0.25，返回 0.30 m/s\n"
    "段间衔接速度 0.20 m/s\n\n"
    "路线理论范围：X 0.20～2.80 m\n"
    "Y -0.50～4.40 m；实车四周另留安全余量"
)
fig.text(0.70, 0.845, config_text, va="top", fontsize=11, color="#344054", linespacing=1.55)

fig.text(0.70, 0.535, "逐段路线", fontsize=17, fontweight="bold", color="#172033")
for column in range(2):
    for row in range(8):
        number, command, distance, speed, start, end = segments[column * 8 + row]
        del start, end
        text = f"{number:02d} {command}\n{distance:.2f} m · {speed:.2f} m/s"
        fig.text(0.70 + column * 0.145, 0.505 - row * 0.050, text,
                 va="top", fontsize=9.2, color=colors[command], linespacing=1.3)

fig.text(0.70, 0.105,
         "注意：仅用于方向和路线验证。\n"
         "纯轮速无法消除打滑与累计漂移；不能据此\n"
         "确认实际贴边距离或最终比赛精度。",
         va="top", fontsize=9.5, color="#b42318", linespacing=1.5,
         bbox=dict(boxstyle="round,pad=0.5", facecolor="#fff6ed", edgecolor="#f4b183"))

fig.savefig(OUT, facecolor=fig.get_facecolor())
print(OUT)
