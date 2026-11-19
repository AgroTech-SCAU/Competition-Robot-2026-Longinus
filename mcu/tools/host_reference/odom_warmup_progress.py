#!/usr/bin/env python3
"""Summarize completed warm-up NDJSON blocks without contacting the MCU."""
import argparse
from collections import Counter, defaultdict, deque
import json
from pathlib import Path
import statistics

from analyze_odom_capture import axis_value, get, load, set_times, stats, temperature


def counts(values):
    values = list(values)
    return {"true": sum(v is True for v in values),
            "false": sum(v is False for v in values),
            "missing": sum(v is None for v in values)}


def boolean(row, name):
    value = get(row, name)
    return None if value is None else bool(value)


def controls_nonzero(row):
    values = [get(row, "s_chassis.kine.control." + axis) for axis in ("vx", "vy", "wz")]
    return any(v != 0 for v in values) if all(v is not None for v in values) else None


def state_summary(rows):
    paths = {"output_enabled": "s_output_enabled", "wheel_valid": "s_odom.wheel_valid",
             "stationary": "s_fusion.stationary", "imu_ready": "s_odom.imu_ready",
             "imu_calibrated": "s_odom.imu_calibrated", "temperature_valid": "s_odom.temperature_valid"}
    result = {label: counts(boolean(r, path) for r in rows) for label, path in paths.items()}
    result["control_nonzero"] = counts(controls_nonzero(r) for r in rows)
    return result


def valid_temperature(row):
    return (boolean(row, "s_odom.temperature_valid") is True
            and row["firmware"] != "mismatch" and temperature(row) is not None)


def temperature_trend(thermal, end):
    minutes = defaultdict(list)
    for row in thermal:
        minutes[min(4, int((row["t"] - (end - 300)) // 60))].append(row)
    minute_means = [{"minutes_before_latest": (300 - statistics.fmean(r["t"] - (end - 300) for r in group)) / 60,
                     "mean_temperature_c": statistics.fmean(temperature(r) for r in group),
                     "samples": len(group)} for _, group in sorted(minutes.items())]
    slope = None
    if len(minute_means) > 1:
        xs = [-m["minutes_before_latest"] for m in minute_means]
        ys = [m["mean_temperature_c"] for m in minute_means]
        xm, ym = statistics.fmean(xs), statistics.fmean(ys)
        denominator = sum((x - xm) ** 2 for x in xs)
        if denominator:
            slope = sum((x - xm) * (y - ym) for x, y in zip(xs, ys)) / denominator
    return minute_means, slope


def thermal_stability(unique, block_ends):
    window, recent_checks = deque(), deque(maxlen=5)
    next_row, epoch, first_t = 0, None, None
    stable_since, stable_since_mcu_s = None, None
    candidate, duration = False, 0.0
    for name, marker in block_ends:
        end = marker["t"]
        while next_row < len(unique) and (unique[next_row]["epoch"], unique[next_row]["t"]) <= (marker["epoch"], end):
            row = unique[next_row]
            next_row += 1
            if row["epoch"] != epoch:
                epoch, first_t = row["epoch"], row["t"]
                stable_since, stable_since_mcu_s = None, None
                window.clear()
            if valid_temperature(row):
                window.append(row)
        while window and window[0]["t"] < end - 300:
            window.popleft()
        minute_means, slope = temperature_trend(window, end)
        values = [temperature(row) for row in window]
        span = max(values) - min(values) if values else None
        observed = end - first_t if first_t is not None else 0.0
        full_window = observed >= 300 and len(minute_means) == 5
        candidate = (observed >= 600 and full_window and slope is not None
                     and abs(slope) <= 0.03 and span is not None and span <= 0.375)
        mcu_s = get(marker, "s_odom.sample_time_us") * 1e-6
        if not candidate:
            stable_since, stable_since_mcu_s = None, None
        elif stable_since is None:
            stable_since, stable_since_mcu_s = end, mcu_s
        duration = end - stable_since if stable_since is not None else 0.0
        recent_checks.append({"block": name, "mcu_s": mcu_s, "observed_s": observed,
                              "full_window": full_window, "slope_c_per_min": slope,
                              "range_c": span, "candidate": candidate})
    return {"candidate": candidate, "stable_since_mcu_s": stable_since_mcu_s,
            "duration_s": duration, "confirmed": candidate and duration >= 180,
            "recent_checks": list(recent_checks)}


def summarize(directory, expected_count):
    rows, files, block_ends, skipped, diagnostics = [], [], [], {}, Counter()
    for path in sorted(directory.glob("*-block-*.ndjson")):
        try:
            block, messages, errors = load([path])
        except (OSError, ValueError) as exc:
            skipped[path.name] = str(exc)
            continue
        if len(block) != expected_count:
            skipped[path.name] = f"{len(block)} of {expected_count} sample records; not complete"
            continue
        rows.extend(block)
        files.append(path.name)
        block_ends.append((path.name, block[-1]))
        diagnostics.update(messages)
        diagnostics["variable_errors"] += len(errors)
    if not rows:
        return {"completed_blocks": 0, "sample_rows": 0, "skipped_files": skipped}

    timing, gap_limit = set_times(rows)
    unique, seen = [], set()
    for row in rows:
        stamp = get(row, "s_odom.sample_time_us")
        key = (row["epoch"], stamp)
        if stamp is None or key in seen:
            continue
        seen.add(key)
        unique.append(row)
    if not unique:
        return {"completed_blocks": len(files), "sample_rows": len(rows), "unique_mcu_samples": 0}
    latest_epoch = unique[-1]["epoch"]
    current = [r for r in unique if r["epoch"] == latest_epoch]
    end = current[-1]["t"]
    recent = [r for r in current if r["t"] >= end - 30]
    thermal = [r for r in current if r["t"] >= end - 300 and valid_temperature(r)]
    minute_means, slope = temperature_trend(thermal, end)
    gaps = [{"after_mcu_s": get(a, "s_odom.sample_time_us") * 1e-6,
             "before_mcu_s": get(b, "s_odom.sample_time_us") * 1e-6,
             "gap_s": b["t"] - a["t"]}
            for a, b in zip(unique, unique[1:])
            if a["epoch"] == b["epoch"] and b["t"] - a["t"] > gap_limit]
    epochs = []
    for epoch in sorted({r["epoch"] for r in unique}):
        group = [r for r in unique if r["epoch"] == epoch]
        epochs.append({"epoch": epoch, "samples": len(group),
                       "first_mcu_s": get(group[0], "s_odom.sample_time_us") * 1e-6,
                       "last_mcu_s": get(group[-1], "s_odom.sample_time_us") * 1e-6})
    return {"completed_blocks": len(files), "last_completed_file": files[-1],
            "sample_rows": len(rows), "unique_mcu_samples": len(unique),
            "duplicates_removed": len(rows) - len(unique), "epochs": epochs,
            "timing": timing, "gaps": gaps, "skipped_files": skipped,
            "diagnostics": dict(diagnostics),
            "latest_30s": {"samples": len(recent),
                "temperature_c": stats(temperature(r) for r in recent if valid_temperature(r)),
                "raw_gyro_rad_s": {a: stats(axis_value(r, "raw", a) for r in recent) for a in "xyz"},
                "residual_bias_rad_s": {a: stats(axis_value(r, "residual_bias", a) for r in recent) for a in "xyz"},
                "states": state_summary(recent)},
            "latest_5min": {"full_history_available": end - current[0]["t"] >= 300,
                "temperature_c": stats(temperature(r) for r in thermal),
                "valid_temperature_span_s": thermal[-1]["t"] - thermal[0]["t"] if thermal else 0,
                "slope_c_per_min": slope, "minute_means": minute_means},
            "thermal_stability": thermal_stability(unique, block_ends),
            "all_states": state_summary(unique),
            "current_epoch_counters": {
                "spi_failure_count": stats(get(r, "s_bmi088_spi_failure_count") for r in current),
                "discontinuity_count": stats(get(r, "s_odom.discontinuity_count") for r in current)},
            "basis": ["Completed means the expected number of sample records parsed; acquisition success is checked separately.",
                "Temperature uses valid-temperature rows and unique MCU timestamps; no imu_ready/calibration/stationary filtering.",
                "Raw gyro statistics use all finite readings at unique MCU timestamps, regardless of readiness.",
                "Slope is an equal-weight regression of up to five one-minute temperature means in the latest MCU epoch.",
                "Low-rate running reads are non-atomic and cannot establish full-rate noise or stationary pose accuracy."]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--count", type=int, default=150, help="Expected samples in each completed block")
    args = parser.parse_args()
    if args.count <= 0:
        parser.error("--count must be positive")
    print(json.dumps(summarize(args.directory, args.count), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
