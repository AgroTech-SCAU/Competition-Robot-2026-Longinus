#!/usr/bin/env python3
"""Offline timing/IMU assessment of the six requested legacy captures."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics

from analyze_odom_capture import get, load, set_times


def quantile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    index = (len(ordered) - 1) * fraction
    lo, hi = math.floor(index), math.ceil(index)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (index - lo)


def summary(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return {"samples": 0}
    return {"samples": len(values), "mean": statistics.fmean(values),
            "std": statistics.stdev(values) if len(values) > 1 else None,
            "min": min(values), "max": max(values), "median": quantile(values, .5),
            "p95": quantile(values, .95), "p99": quantile(values, .99)}


def commands(row):
    return [get(row, "s_chassis.kine.control." + a) for a in ("vx", "vy", "wz")]


def commanded(row):
    return any(v is not None and v != 0 for v in commands(row))


def reported_stationary(row):
    return (get(row, "s_fusion.stationary") == 1 and not commanded(row)
            and get(row, "s_odom.wheel_valid") != 0 and row["firmware"] != "mismatch")


def projected_yaw_rate(row):
    roll, pitch, gy, gz = [get(row, name) for name in
        ("s_odom.angle.x", "s_odom.angle.y", "s_odom.gyro_corrected.y", "s_odom.gyro_yaw_fusion")]
    if any(v is None for v in (roll, pitch, gy, gz)) or abs(math.cos(pitch)) <= .2:
        return None
    return (gy * math.sin(roll) + gz * math.cos(roll)) / math.cos(pitch)


def sparse_integral(rows, value):
    left = right = duration = 0.0
    pairs = 0
    for a, b in zip(rows, rows[1:]):
        va, vb = value(a), value(b)
        if va is None or vb is None or a["epoch"] != b["epoch"] or b["t"] <= a["t"]:
            continue
        dt = b["t"] - a["t"]
        left += va * dt
        right += vb * dt
        duration += dt
        pairs += 1
    if not pairs:
        return None
    return {"pairs": pairs, "duration_s": duration,
            "trapezoid_deg": math.degrees((left + right) / 2),
            "left_rectangle_deg": math.degrees(left), "right_rectangle_deg": math.degrees(right),
            "note": "Includes linear interpolation across logging gaps; not a 250 Hz integral or a true-angle measurement. Left/right difference is not an error bound."}


def assess(name, paths, snapshots):
    rows, errors, messages = [], [], Counter()
    block_counts = {}
    for path in paths:
        block, msg, err = load([path])
        messages.update(msg)
        errors.extend(err)
        block_counts[path.name] = len(block)
        for row in block:
            row["block"] = path.name
            row["source_index"] = len(rows)
            rows.append(row)
    timing, _ = set_times(rows)
    pairs = [(a, b) for a, b in zip(rows, rows[1:])
             if a["epoch"] == b["epoch"] and b["t"] > a["t"]]
    intervals = [b["t"] - a["t"] for a, b in pairs]
    nominal = statistics.median(intervals)
    gap_threshold = 1.5 * nominal
    gaps = [{"before_index": a["source_index"], "after_index": b["source_index"],
             "start_mcu_s": get(a, "s_odom.sample_time_us") * 1e-6,
             "end_mcu_s": get(b, "s_odom.sample_time_us") * 1e-6,
             "duration_s": b["t"] - a["t"], "block_boundary": a["block"] != b["block"]}
            for a, b in pairs if b["t"] - a["t"] > gap_threshold]
    boundaries = [b["t"] - a["t"] for a, b in pairs if a["block"] != b["block"]]
    host_span = (rows[-1]["host_ms"] - rows[0]["host_ms"]) * .001
    unique = [r for r in rows if not r["duplicate"]]
    rest = [r for r in unique if reported_stationary(r)]
    gyros = {name: summary(get(r, path) for r in rest) for name, path in {
        "raw_x_rad_s": "s_odom.gyro.x", "raw_y_rad_s": "s_odom.gyro.y", "raw_z_rad_s": "s_odom.gyro.z",
        "corrected_x_rad_s": "s_odom.gyro_corrected.x", "corrected_y_rad_s": "s_odom.gyro_corrected.y",
        "corrected_z_rad_s": "s_odom.gyro_corrected.z", "scaled_body_z_rad_s": "s_odom.gyro_yaw_fusion"}.items()}
    gyros["projected_euler_yaw_rad_s"] = summary(projected_yaw_rate(r) for r in rest)
    projected_rows = [r for r in unique if projected_yaw_rate(r) is not None]
    rate_differences = [projected_yaw_rate(r) - get(r, "s_odom.gyro_yaw_fusion") for r in projected_rows]
    moving_projection = [r for r in projected_rows if abs(get(r, "s_odom.gyro_yaw_fusion")) > .05]
    motion_indices = [i for i, r in enumerate(unique) if commanded(r)]
    motion = unique[max(0, motion_indices[0] - 1):min(len(unique), motion_indices[-1] + 2)] if motion_indices else []
    def integrals(rr):
        return {"samples": len(rr),
                "start_mcu_s": get(rr[0], "s_odom.sample_time_us") * 1e-6 if rr else None,
                "end_mcu_s": get(rr[-1], "s_odom.sample_time_us") * 1e-6 if rr else None,
                "note": "Net signed integral over this entire span, including any reversals; do not interpret it as a single-direction turn.",
                "body_z": sparse_integral(rr, lambda r: get(r, "s_odom.gyro_yaw_fusion")),
                "projected_euler_yaw": sparse_integral(rr, projected_yaw_rate)}
    temperature_snapshots = []
    for path in snapshots:
        block, _, err = load([path])
        for r in block:
            if get(r, "s_odom.temperature_c") is not None:
                temperature_snapshots.append({"file": str(path),
                    "mcu_s": get(r, "s_odom.sample_time_us") * 1e-6,
                    "temperature_c": get(r, "s_odom.temperature_c"),
                    "temperature_valid": get(r, "s_odom.temperature_valid"),
                    "firmware_consistency": r["firmware"]})
    examples = []
    selected_indices = {"held-steer-positive-01": (399, 400), "physical-creep-01": (249, 250)}
    if name in selected_indices:
        i, j = selected_indices[name]
        a, b = rows[i], rows[j]
        dt = b["t"] - a["t"]
        angle_delta = (get(b, "s_odom.angle.z") - get(a, "s_odom.angle.z") + math.pi) % (2 * math.pi) - math.pi
        rate_integral = .5 * dt * (get(a, "s_odom.velocity.z") + get(b, "s_odom.velocity.z"))
        examples.append({"indices": [i, j], "duration_s": dt,
                         "stored_fused_yaw_increment_deg": math.degrees(angle_delta),
                         "endpoint_fused_rates_rad_s": [get(a, "s_odom.velocity.z"), get(b, "s_odom.velocity.z")],
                         "sparse_fused_rate_trapezoid_deg": math.degrees(rate_integral),
                         "stored_minus_sparse_deg": math.degrees(angle_delta - rate_integral),
                         "note": "This is a discrepancy between two representations of the same estimator output, not an externally measured angle error."})
    return {"capture": name, "source_files": [str(p) for p in paths], "samples": len(rows),
        "block_counts": block_counts, "errors": errors, "other_records": dict(messages),
        "elf_sha256": sorted({r["elf_sha256"] for r in rows}), "partial_samples": sum(r["partial"] for r in rows),
        "timing": {"mcu_first_s": get(rows[0], "s_odom.sample_time_us") * 1e-6,
                   "mcu_last_s": get(rows[-1], "s_odom.sample_time_us") * 1e-6,
                   "mcu_elapsed_s": timing["span_s"], "duplicates": timing["duplicates"], "clock_resets": timing["clock_resets"],
                   "intervals_s": summary(intervals),
                   "within_block_intervals_s": summary(b["t"] - a["t"] for a, b in pairs if a["block"] == b["block"]),
                   "block_boundary_intervals_s": summary(boundaries),
                   "gap_threshold_s": gap_threshold, "gap_count": len(gaps),
                   "gap_interval_total_s": sum(g["duration_s"] for g in gaps),
                   "gap_excess_above_median_total_s": sum(g["duration_s"] - nominal for g in gaps),
                   "gaps": gaps, "host_elapsed_s": host_span,
                   "host_minus_mcu_elapsed_ms": (host_span - timing["span_s"]) * 1000,
                   "host_minus_mcu_interval_ms": summary((b["host_ms"] - a["host_ms"]) - (b["t"] - a["t"]) * 1000 for a, b in pairs),
                   "note": "Host-vs-MCU elapsed comparison includes sequential-read latency and scheduling; it does not establish either clock's absolute accuracy."},
        "reported_stationary": {"samples": len(rest), "fraction": len(rest) / len(unique),
            "command_xyz_all_available": all(all(v is not None for v in commands(r)) for r in rest),
            "gyro": gyros,
            "tracked_stationary_gyro_mean_z_rad_s": summary(get(r, "s_fusion.stationary_gyro_mean.z") for r in rest),
            "acc_m_s2": {a: summary(get(r, "s_odom.acc." + a) for r in rest) for a in "xyz"},
            "note": "Conditioned on firmware stationary flag and no observed nonzero command; not independently proven physical rest. A tracked gyro mean is a filter state, not raw gyro noise."},
        "attitude_observed_deg": {**{a: summary(math.degrees(v) for r in unique if (v := get(r, "s_odom.angle." + axis)) is not None)
                                   for a, axis in [("roll", "x"), ("pitch", "y")]},
            "note": "Observed estimates, not absolute roll/pitch errors; physical slope and mounting tilt are not independently measured."},
        "projection": {"formula": "(corrected_gyro_y*sin(roll) + scaled_body_gyro_z*cos(roll))/cos(pitch)",
            "sampled_command_directions": sorted({"positive" if get(r, "s_chassis.kine.control.wz") > 0 else "negative"
                for r in unique if get(r, "s_chassis.kine.control.wz") not in (None, 0)}),
            "available_samples": len(projected_rows), "instantaneous_difference_rad_s": summary(rate_differences),
            "ratio_when_abs_body_z_above_0_05": summary(projected_yaw_rate(r) / get(r, "s_odom.gyro_yaw_fusion") for r in moving_projection),
            "all_capture_sparse_integrals": integrals(unique), "commanded_span_sparse_integrals": integrals(motion)},
        "temperature": {"trend_values_c": summary(get(r, "s_odom.temperature_c") for r in unique),
            "supporting_snapshot_values": temperature_snapshots,
            "valid_snapshot_range_c": summary(s["temperature_c"] for s in temperature_snapshots if s["temperature_valid"] == 1),
            "note": "Sparse supporting snapshots do not establish a continuous thermal range or a thermal drift model."},
        "sampling_examples": examples}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", type=Path, default=Path(__file__).resolve().parents[1] / "mcu/build/OdomTune/captures")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    base = args.captures
    cases = [
        ("held-steer-positive-01", sorted((base / "held-steer-positive-01").glob("sample-*.ndjson")), [base / "held-steer-positive-01/start.ndjson", base / "held-steer-positive-01/end.ndjson"]),
        ("held-steer-negative-01", sorted((base / "held-steer-negative-01").glob("sample-*.ndjson")), [base / "held-steer-negative-01/start.ndjson", base / "held-steer-negative-01/end.ndjson"]),
        ("physical-creep-01", sorted((base / "physical-creep-01").glob("sample-*.ndjson")), [base / "physical-creep-01/before-return.ndjson", base / "physical-creep-01/after-return.ndjson"]),
        ("drive-zero-hold", [base / "physical-creep-01/drive-zero-hold.ndjson"], []),
        ("hot-spin-192-positive", sorted((base / "hot-spin-192-positive").glob("sample-*.ndjson")), []),
        ("zero-steer-after", [base / "zero-steer-after.ndjson"], [base / "zero-steer-after-start.ndjson"]),
    ]
    result = {"scope": "Offline legacy IMU/timing observations; no hardware access and no parameter fitting",
              "quantile_method": "Linear interpolation at (n-1)*q; standard deviations use n-1",
              "limitations": [
                  "No independent calibrated angle, roll, pitch, position or clock reference is present; these records cannot establish absolute accuracy or error percentages.",
                  "The operator used RC alignment, not hand repositioning; alignment uncertainty and steering-induced chassis rotation remain unquantified.",
                  "5 Hz variable reads are sequential, not atomic, and omit most 250 Hz samples. Block reconnect gaps can miss entire correction pulses; sparse integration error has no rigorous upper bound from these records alone.",
                  "Stationary-conditioned gyro statistics are decimated and selected by the estimator's gate. They are not a full-band 250 Hz sensor-noise estimate or a unique basis for Q/R tuning.",
                  "Physical creep can coexist with a stationary flag. A locked fused pose does not prove physical rest or sensor precision.",
                  "Missing axes or temperature trends remain unavailable; single snapshots cannot supply noise statistics or a thermal compensation fit.",
                  "These logs predate the new MCU independent yaw integrals and the pending drive fix; they cannot validate those changes."],
              "datasets": [assess(*case) for case in cases]}
    output = args.output or base / "legacy-error-assessment/imu-timing.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(output)
    for d in result["datasets"]:
        t = d["timing"]
        print(d["capture"], d["samples"], "span_s", round(t["mcu_elapsed_s"], 6),
              "median/p95/p99/max_s", [round(t["intervals_s"][key], 6) for key in ("median", "p95", "p99", "max")],
              "gaps", t["gap_count"], "host_minus_mcu_ms", round(t["host_minus_mcu_elapsed_ms"], 3))


if __name__ == "__main__":
    main()
