#!/usr/bin/env python3
"""Plot one saved BMI088 stationary warm-up; never connects to hardware.

Example (set PYTHONPATH to the installed plot-packages directory first):
    python tools/plot_odom_warmup.py path/to/cold-warm-capture --output path/to/warmup

Reads completed *block-*.ndjson files and the *start.ndjson calibration snapshot.
Writes only OUTPUT.png and OUTPUT.svg. No coefficients are fitted or written.
The caller must have independently confirmed physical standstill throughout.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import math
from pathlib import Path
import statistics

from analyze_odom_capture import axis_value, get, load, set_times, temperature


AXES = "xyz"
COLORS = ("#2474B5", "#DA7840", "#299376")


def required(row, name):
    value = get(row, name)
    if value is None:
        raise ValueError(f"Startup snapshot is missing {name}")
    return value


def read_warmup(folder):
    starts = sorted(folder.glob("*start.ndjson"))
    blocks = sorted(folder.glob("*block-*.ndjson"))
    if len(starts) != 1 or not blocks:
        raise ValueError("Expected one *start.ndjson snapshot and completed *block-*.ndjson captures")
    snapshots, _, snapshot_errors = load(starts)
    rows, _, errors = load(blocks)
    if snapshot_errors or errors:
        raise ValueError("Capture reports variable read errors; inspect them before plotting")
    if len(snapshots) != 1 or not rows:
        raise ValueError("Expected one startup sample and nonempty block samples")
    start = snapshots[0]
    model = {
        "bias": [required(start, f"s_bmi088_attitude.gyro_bias.{a}") for a in AXES],
        "coefficient": [required(start, f"s_bmi088_attitude.config.gyro_{a}_temp_coeff") for a in AXES],
        "reference_c": required(start, "s_bmi088_attitude.gyro_temp_ref"),
    }
    # The s_odom.gyro_bias getter exposes effective bias, not the startup bias.
    hashes = {r["elf_sha256"] for r in [start, *rows] if r["elf_sha256"]}
    if len(hashes) > 1 or any(r["firmware"] == "mismatch" for r in [start, *rows]):
        raise ValueError("Mixed or mismatched firmware; do not pool this warm-up")
    rows.sort(key=lambda r: r["host_ms"] if r["host_ms"] is not None else r["index"])
    if any(get(r, "s_odom.sample_time_us") is None for r in rows):
        raise ValueError("MCU sample timestamps are required for the uptime axis")
    timing, _ = set_times(rows)
    if timing["clock_resets"] or len({r["epoch"] for r in rows}) != 1:
        raise ValueError("Multiple MCU clock epochs; plot each boot separately")
    origin_s = required(rows[0], "s_odom.sample_time_us") * 1e-6
    points = []
    for row in rows:
        raw = [axis_value(row, "raw", a) for a in AXES]
        residual = [axis_value(row, "residual_bias", a) for a in AXES]
        temp = temperature(row)
        if (row["duplicate"] or row["partial"] or temp is None
                or get(row, "s_odom.temperature_valid") == 0
                or get(row, "s_odom.imu_calibrated") == 0
                or any(x is None for x in [*raw, *residual])):
            continue
        # Do not gate on s_odom.imu_ready/static_window: the service clears
        # ready in-flight, and a constrained stationary output is not evidence
        # of sensor accuracy. Physical standstill is supplied by the experiment.
        startup_only = [v - b for v, b in zip(raw, model["bias"])]
        linear = [v - k * (temp - model["reference_c"])
                  for v, k in zip(startup_only, model["coefficient"])]
        total = [v - b for v, b in zip(linear, residual)]
        points.append({"uptime_s": origin_s + row["t"], "temperature": temp,
                       "raw": raw, "startup_only": startup_only, "linear": linear,
                       "total": total, "residual": residual})
    if len(points) < 10:
        raise ValueError("Too few complete calibrated gyro/temperature samples")
    model["elf"] = next(iter(hashes), "unreported")
    return points, model


def grouped(points, key):
    result = defaultdict(list)
    for point in points:
        result[key(point)].append(point)
    return [result[k] for k in sorted(result)]


def axis_means(group, category):
    return [statistics.fmean(p[category][i] for p in group) for i in range(3)]


def render(points, model, output, bin_width, window_s, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "svg.fonttype": "none", "savefig.facecolor": "white"})
    figure, panels = plt.subplots(3, 1, figsize=(12.8, 12.2))
    figure.subplots_adjust(left=0.09, right=0.98, top=0.91, bottom=0.12, hspace=0.49)
    figure.suptitle("BMI088 stationary warm-up | raw drift and existing correction", fontsize=17, x=0.09, ha="left")
    figure.text(0.09, 0.94, f"{title}  |  ELF {model['elf'][:12]}  |  {len(points):,} observed samples", color="#555555")
    for panel in panels:
        panel.grid(True, color="#E5E8EC", linewidth=0.7)
        panel.set_axisbelow(True)

    times = [p["uptime_s"] / 60 for p in points]
    temperatures = [p["temperature"] for p in points]
    panels[0].plot(times, temperatures, color="#556270", linewidth=1.8)
    panels[0].set(title="A   Self-heating during the recorded interval", xlabel="MCU uptime (min)", ylabel="Temperature (°C)")
    panels[0].text(0.02, 0.93,
                   f"First recorded sample: {points[0]['uptime_s']:.1f} s after MCU start\n"
                   f"Startup calibration reference: {model['reference_c']:.3f} °C",
                   transform=panels[0].transAxes, va="top", fontsize=9,
                   bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.85})

    temperature_groups = [g for g in grouped(points, lambda p: math.floor(p["temperature"] / bin_width)) if len(g) >= 5]
    if not temperature_groups:
        raise ValueError("No temperature bins have at least five complete samples")
    bin_t = [statistics.fmean(p["temperature"] for p in g) for g in temperature_groups]
    bin_raw = [axis_means(g, "raw") for g in temperature_groups]
    origin = points[0]["uptime_s"]
    for i, (axis, color) in enumerate(zip(AXES, COLORS)):
        panels[1].plot(bin_t, [v[i] * 1000 for v in bin_raw], "o-", color=color,
                       linewidth=1.4, markersize=3.6, label=axis.upper())
        for x, group, center in zip(bin_t, temperature_groups, bin_raw):
            chunks = grouped(group, lambda p: math.floor((p["uptime_s"] - origin) / 5.0))
            chunk_means = [axis_means(g, "raw")[i] for g in chunks if len(g) >= 3]
            if len(chunk_means) >= 3:
                panels[1].errorbar(x, center[i] * 1000, yerr=statistics.stdev(chunk_means) * 1000,
                                  color=color, capsize=2, elinewidth=0.8, alpha=0.5)
        model_curve = [(model["bias"][i] + model["coefficient"][i] * (t - model["reference_c"])) * 1000 for t in bin_t]
        panels[1].plot(bin_t, model_curve, ":", color=color, alpha=0.8, linewidth=1.2)
    panels[1].set(title=f"B   Raw gyro mean in {bin_width:g} °C temperature bins", xlabel="Temperature (°C)", ylabel="Raw angular rate (mrad/s)")
    panels[1].legend(loc="best", ncols=3, frameon=True)
    panels[1].text(0, -0.27,
                   "Dotted: existing startup + linear temperature model; no fit performed\n"
                   "Whiskers: ±1 SD of 5 s group means where available; descriptive spread, not a confidence interval",
                   transform=panels[1].transAxes, fontsize=8.5, color="#555555", va="top")

    time_groups = [g for g in grouped(points, lambda p: math.floor((p["uptime_s"] - origin) / window_s))
                   if g[-1]["uptime_s"] - g[0]["uptime_s"] >= 0.8 * window_s]
    if not time_groups:
        raise ValueError(f"No windows have at least {0.8 * window_s:g} s of observed time span")
    window_t = [statistics.fmean(p["uptime_s"] for p in g) / 60 for g in time_groups]
    categories = [("startup_only", "--", "Startup bias removed"),
                  ("linear", "-", "+ existing linear temperature correction")]
    residual_max = max(abs(v) for p in points for v in p["residual"])
    if residual_max > 1e-9:
        categories.append(("total", "-.", "+ recorded online residual"))
    for category, linestyle, _ in categories:
        values = [axis_means(g, category) for g in time_groups]
        for i, color in enumerate(COLORS):
            panels[2].plot(window_t, [v[i] * 1000 for v in values], linestyle,
                           color=color, linewidth=1.5, marker="o", markersize=2.6)
    panels[2].axhline(0, color="#666666", linewidth=0.8, alpha=0.6)
    panels[2].set(title=f"C   Reconstructed residual rate: {window_s:g} s time-bin means",
                  xlabel="MCU uptime (min)", ylabel="Residual angular rate (mrad/s)")
    handles = [Line2D([0], [0], color="#4E5661", linestyle=style, label=label)
               for _, style, label in categories]
    panels[2].legend(handles=handles, loc="best", frameon=True, fontsize=8.5)
    residual_note = ("Recorded online residual is zero throughout; the total correction equals the linear-model curve."
                     if residual_max <= 1e-9 else
                     f"Recorded online residual varies; max absolute component {residual_max * 1000:.3f} mrad/s.")
    figure.text(0.09, 0.067, residual_note + "\n"
                "Axis colors: X blue, Y orange, Z green. Running SWD fields are not atomic; only grouped trends are interpreted.\n"
                "Physical standstill is externally confirmed. One heating run cannot establish repeatability or thermal hysteresis; locked yaw is not used.",
                fontsize=8.5, color="#555555", va="top")
    output.parent.mkdir(parents=True, exist_ok=True)
    png, svg = output.with_suffix(".png"), output.with_suffix(".svg")
    figure.savefig(png, dpi=180)
    figure.savefig(svg)
    plt.close(figure)
    print(f"Saved {png.resolve()}\nSaved {svg.resolve()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--output", type=Path, help="Output filename prefix; default: CAPTURE_DIR/warmup")
    parser.add_argument("--temperature-bin", type=float, default=0.5)
    parser.add_argument("--window-seconds", type=float, default=30.0)
    args = parser.parse_args()
    if any(not math.isfinite(x) or x <= 0 for x in (args.temperature_bin, args.window_seconds)):
        parser.error("Bin and window sizes must be positive")
    try:
        points, model = read_warmup(args.capture_dir)
        render(points, model, args.output or args.capture_dir / "warmup",
               args.temperature_bin, args.window_seconds, args.capture_dir.name)
    except (OSError, ValueError, ImportError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
