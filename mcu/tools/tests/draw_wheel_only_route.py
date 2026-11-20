#!/usr/bin/env python3
"""Draw the firmware route and its operator-facing control overview."""

from pathlib import Path
import math
import re

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, Rectangle
from matplotlib.ticker import MultipleLocator

from test_fast_route import CONTROL_SOURCE, numeric_define, parse_route


MCU = Path(__file__).resolve().parents[2]
OUTPUT = MCU / "docs/wheel_only_route.png"
COLORS = {"-vx": "#1075B6", "+vx": "#C66617", "+vy": "#078264", "-vy": "#8343A5"}
INK = "#122738"
MUTED = "#526575"


def configure_fonts():
    installed = {font.name for font in font_manager.fontManager.ttflist}
    for name in ("Microsoft YaHei", "Noto Sans CJK SC", "Source Han Sans SC", "SimHei"):
        if name in installed:
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False


def read_route():
    header = (MCU / "src/app/map_route.h").read_text(encoding="utf-8")
    start_y = re.search(r"^#define\s+MAP_START_Y_M\s+\((-?\d+(?:\.\d+)?)f\)", header, re.M)
    assert start_y
    start = (numeric_define(header, "MAP_START_X_M"), float(start_y.group(1)))

    segments = []
    furrows = {}
    trim_segments = []
    previous = start
    for number, leg in enumerate(parse_route(), 1):
        end = (leg.x, leg.y)
        dx, dy = end[0] - previous[0], end[1] - previous[1]
        if dx:
            command = "+vy" if dx > 0 else "-vy"
        else:
            command = "-vx" if dy > 0 else "+vx"
        distance = math.hypot(dx, dy)
        segments.append((number, previous, end, distance, command))
        if dx:
            furrows.setdefault(leg.y, []).append((number, distance, leg.work_stop))
        if leg.endpoint_side_trim:
            trim_segments.append((number, command))
        previous = end
    assert len(segments) == 16 and len(furrows) == 7, "route schematic expects 16 legs and 7 furrows"
    return start, segments, furrows, trim_segments


def draw_field(ax, start, segments, furrows):
    ax.set_facecolor("#FFFFFF")
    ax.add_patch(Rectangle((0, 0), 3.0, 4.6, facecolor="#FFFDF7", edgecolor="#263746",
                           linewidth=2, zorder=0))
    ax.add_patch(Rectangle((0, -1.0), 0.4, 1.0, facecolor="#DFF3F2", edgecolor="#36938C",
                           linewidth=1.5, hatch="///", zorder=0))
    ax.text(0.20, -0.90, "起终点区", ha="center", va="bottom", fontsize=9,
            color="#17655F", fontweight="bold")

    # The field envelope and ridge shapes are the schematic from the field map.
    for index in range(6):
        center_y = 0.55 + 0.70 * index
        width = 1.8 if index == 3 else 2.2
        ax.add_patch(Rectangle(((3.0 - width) / 2, center_y - 0.15), width, 0.30,
                               facecolor="#E8D7B7", edgecolor="#9A7950", linewidth=1.2, zorder=1))
        ax.text(1.5, center_y, "垄背", ha="center", va="center", fontsize=9,
                color="#6F5232", zorder=2)

    for index, y in enumerate(furrows, 1):
        ax.text(-0.06, y, f"第 {index} 垄", ha="right", va="center", fontsize=9,
                color=INK, fontweight="bold", zorder=6)

    for number, (sx, sy), (ex, ey), distance, command in segments:
        # Display offsets separate the overlapping entry and crossings.
        if number == 1:
            sx -= 0.055
            ex -= 0.055
        elif number in (2, 3):
            offset = 0.072 if number == 2 else -0.072
            sy += offset
            ey += offset
        color = COLORS[command]
        ax.add_patch(FancyArrowPatch((sx, sy), (ex, ey), arrowstyle="-|>",
                                     mutation_scale=16, linewidth=2.4, color=color,
                                     shrinkA=0, shrinkB=0, zorder=4))
        mx, my = (sx + ex) / 2, (sy + ey) / 2
        if number == 1:
            mx, my = 0.08, 2.13
        elif number in (4, 8, 12, 16):
            mx += 0.21
        elif number in (6, 10, 14):
            mx -= 0.21
        ax.text(mx, my, f"{number:02d}  {distance:.2f} m", ha="center", va="center",
                fontsize=8.7, color=color, fontweight="bold", zorder=5,
                bbox={"boxstyle": "round,pad=0.18", "facecolor": "white", "edgecolor": color})

    ax.scatter(*start, marker="*", s=260, color="#F2B134", edgecolor="#6B4D0F",
               linewidth=1, zorder=7)
    ax.text(0.34, -0.63, f"起点／终点\n({start[0]:.2f}, {start[1]:.2f})", ha="left", va="center",
            fontsize=9, color="#6B4D0F", fontweight="bold",
            bbox={"boxstyle": "round,pad=0.25", "facecolor": "#FFF8DE", "edgecolor": "#D8A620"})

    dimensions = {"arrowstyle": "<->", "color": MUTED, "lw": 1.1}
    ax.annotate("", xy=(0, 4.76), xytext=(3.0, 4.76), arrowprops=dimensions)
    ax.text(1.5, 4.79, "300 cm", ha="center", va="bottom", fontsize=9, color=MUTED)
    ax.annotate("", xy=(3.13, 0), xytext=(3.13, 4.6), arrowprops=dimensions)
    ax.text(3.18, 2.3, "460 cm", ha="left", va="center", rotation=90, fontsize=9, color=MUTED)
    ax.set(xlim=(-0.57, 3.36), ylim=(-1.08, 4.96), aspect="equal")
    ax.xaxis.set_major_locator(MultipleLocator(0.5))
    ax.yaxis.set_major_locator(MultipleLocator(0.5))
    ax.set_axisbelow(True)
    ax.grid(True, color="#DCE3EA", lw=0.65, ls=":")
    ax.tick_params(labelsize=8, colors=MUTED)
    ax.set_xlabel("地图 X / m（+X = +vy）", fontsize=10)
    ax.set_ylabel("地图 Y / m（+Y = −vx）", fontsize=10)
    for spine in ax.spines.values():
        spine.set_color("#B8C4CE")

    directions = {"-vx": "−vx 入场", "+vx": "+vx 换垄／退出",
                  "+vy": "+vy 车体左", "-vy": "−vy 车体右"}
    legend = [Line2D([0], [0], color=COLORS[key], lw=2.6, label=label)
              for key, label in directions.items()]
    ax.legend(handles=legend, loc="lower center", bbox_to_anchor=(0.5, 1.015),
              ncol=4, frameon=False, fontsize=8.2, columnspacing=0.9)


def draw_logic(panel, segments, furrows, trim_segments):
    panel.axis("off")
    panel.set(xlim=(0, 1), ylim=(0, 1), autoscale_on=False)
    panel.text(0.01, 0.99, "垄号按首次进入顺序", fontsize=15, fontweight="bold", color=INK, va="top")
    headers = ((0.025, "垄号"), (0.17, "Y / m"), (0.30, "经过段 · 累计距离"), (0.65, "停车作业"))
    for x, title in headers:
        panel.text(x, 0.925, title, fontsize=9.7, fontweight="bold", color=MUTED, va="center")
    panel.axhline(0.902, color="#B6C2CD", lw=1)

    for index, (y_m, passes) in enumerate(furrows.items(), 1):
        y = 0.874 - (index - 1) * 0.044
        if index % 2:
            panel.axhspan(y - 0.019, y + 0.019, color="#EDF3F7", zorder=0)
        numbers = "/".join(f"{number:02d}" for number, _, _ in passes)
        distance = sum(distance for _, distance, _ in passes)
        work = "，".join(f"{number:02d} 作业" if enabled else f"{number:02d} 仅返回"
                        for number, _, enabled in passes)
        for x, value in ((0.025, f"第 {index} 垄"), (0.17, f"{y_m:.2f}"),
                         (0.30, f"{numbers} · {distance:.2f} m"), (0.65, work)):
            panel.text(x, y, value, fontsize=9.5, color=INK, va="center")

    panel.text(0.01, 0.535, "逐段控制", fontsize=15, fontweight="bold", color=INK, va="center")
    blocks = (
        (0.485, "01　入场 · 4.90 m", "轮速推进｜CAN LEFT 中值纠偏 36 → 41 mm｜前束留 70 mm"),
        (0.422, "02／03／05／07／09／11／13／15　横穿 · 各 2.60 m",
         "轮速推进｜前后束中值纠偏｜行进方向侧束限速"),
        (0.329, "04／08／12　西侧换垄 · 各 0.70 m",
         "轮速推进｜CAN LEFT（车体右）41 mm｜贴壁先让开"),
        (0.265, "06／10／14　东侧换垄 · 各 0.70 m",
         "轮速推进｜CAN RIGHT（车体左）45 mm｜贴壁先让开"),
        (0.201, "16　返回 · 0.70 m", "轮速推进｜即时近距保护"),
    )
    for y, title, detail in blocks:
        panel.text(0.025, y, title, fontsize=10, fontweight="bold", color=INK, va="center")
        panel.text(0.025, y - 0.031, detail, fontsize=9.2, color=MUTED, va="center")

    if trim_segments:
        tolerance_mm = numeric_define(CONTROL_SOURCE, "LASER_WALL_TRIM_TOLERANCE_MM")
        for y, command, side in ((0.370, "-vy", "LEFT"), (0.350, "+vy", "RIGHT")):
            handovers = "／".join(f"{number:02d}→{number + 1:02d}"
                                 for number, direction in trim_segments if direction == command)
            if handovers:
                target_mm = numeric_define(CONTROL_SOURCE, f"LASER_{side}_TARGET_MM")
                panel.text(0.025, y,
                           f"{handovers}｜CAN {side} {target_mm:.0f} ± {tolerance_mm:.0f} mm 确认后换段",
                           fontsize=9.2, color=COLORS[command], va="center")

    total = sum(distance for _, _, _, distance, _ in segments)
    panel.add_patch(Rectangle((0, 0.014), 0.99, 0.128, facecolor="#EAF4FF", edgecolor="#79A8CC"))
    max_mm = 1000 * numeric_define(CONTROL_SOURCE, "ROUTE_WALL_TRIM_MAX_M")
    panel.text(0.025, 0.109, f"换段停稳 100 ms · 贴边最多 {max_mm:.0f} mm · 全程近距保护", fontsize=10.0,
               fontweight="bold", color="#175E9E", va="center")
    panel.text(0.025, 0.075, "01 和横穿剩余 ≤80 mm 时，激光确认近墙可提前收停",
               fontsize=8.7, color="#285677", va="center")
    panel.text(0.025, 0.042, "01～15 激光数据过期时暂停推进，16 按轮速退出",
               fontsize=8.7, color="#285677", va="center")
    panel.text(0.965, 0.111, f"标称 {total:.2f} m", fontsize=12.5, fontweight="bold",
               color="#175E9E", ha="right", va="center")


def main():
    configure_fonts()
    start, segments, furrows, trim_segments = read_route()
    fig = plt.figure(figsize=(15.6, 10.5), dpi=160, facecolor="#F5F7FB")
    grid = fig.add_gridspec(1, 2, width_ratios=[1.13, 1], left=0.05, right=0.975,
                            top=0.837, bottom=0.105, wspace=0.08)
    draw_field(fig.add_subplot(grid[0, 0]), start, segments, furrows)
    draw_logic(fig.add_subplot(grid[0, 1]), segments, furrows, trim_segments)
    fig.suptitle("全局跑图示意", x=0.055, y=0.955, ha="left", fontsize=25,
                 fontweight="bold", color=INK)
    fig.text(0.055, 0.900, "16 段 · 7 条垄 · 8 次横穿 · 6 次换垄", fontsize=11.5, color=MUTED)
    fig.text(0.055, 0.044, "段长由 map_route.c 的相邻目标点计算。重合箭头略微错开，垄背为位置示意。",
             fontsize=9.4, color=MUTED)
    fig.savefig(OUTPUT, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(OUTPUT.resolve())


if __name__ == "__main__":
    main()
