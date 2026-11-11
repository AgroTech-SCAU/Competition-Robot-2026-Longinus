#!/usr/bin/env python3
"""Analyze saved EmberProbe read.js --count NDJSON; never connects to a probe.

Example:
    python tools/analyze_odom_capture.py capture.ndjson --output summary.json
Both scalar variable paths and composite item.tree reports are accepted.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import statistics


def number(value):
    if isinstance(value, bool):
        return float(value)
    try:
        parsed = float(value)
        return parsed if math.isfinite(parsed) else None
    except (TypeError, ValueError):
        return None


def leaves(node, path):
    if not isinstance(node, dict):
        return
    if isinstance(node.get("members"), list):
        for member in node["members"]:
            yield from leaves(member, path + "." + str(member["name"]))
    elif isinstance(node.get("elements"), list):
        for element in node["elements"]:
            yield from leaves(element, path + "[" + str(element["index"]) + "]")
    elif "value" in node:
        yield path, number(node.get("valueText") if node.get("valueText") is not None else node["value"])


def load(paths):
    rows, diagnostics, errors = [], Counter(), []
    for path in paths:
        with path.open(encoding="utf-8-sig") as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                try:
                    item = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{path}:{line_number}: invalid NDJSON: {exc.msg}") from exc
                if item.get("type") != "sample":
                    diagnostics[str(item.get("type", "untyped"))] += 1
                    errors.extend(item.get("variableErrors", []))
                    continue
                flat = {}
                for name, value in item.get("values", {}).items():
                    # C compilation-unit qualification does not change this task's unique variable names.
                    name = name.rsplit("::", 1)[-1]
                    if isinstance(value, dict):
                        flat.update(leaves(value.get("tree", value), name))
                    else:
                        flat[name] = number(value)
                consistency = item.get("firmwareConsistency")
                elf = item.get("elf")
                rows.append({"values": flat, "host_ms": number(item.get("timestamp")),
                             "firmware": consistency.get("status") if isinstance(consistency, dict) else consistency,
                             "elf": elf.get("path") if isinstance(elf, dict) else elf,
                             "elf_sha256": elf.get("sha256") if isinstance(elf, dict) else item.get("elfSha256"),
                             "partial": bool(item.get("partial")), "index": len(rows)})
                errors.extend(item.get("variableErrors", []))
    return rows, dict(diagnostics), errors


def get(row, *names):
    for name in names:
        value = row["values"].get(name)
        if value is not None:
            return value
    return None


def flag(row, name):
    value = get(row, "s_odom." + name)
    return None if value is None else bool(value)


def stats(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return {"samples": 0}
    ordered = sorted(values)
    return {"samples": len(values), "mean": statistics.fmean(values),
            "sample_std": statistics.stdev(values) if len(values) > 1 else None,
            "min": ordered[0], "max": ordered[-1], "first": values[0], "last": values[-1],
            "p95_abs": sorted(map(abs, values))[round((len(values)-1)*0.95)]}


def set_times(rows):
    sources, duplicates, resets, previous, elapsed, epoch = Counter(), 0, 0, None, 0.0, 0
    steps = []
    for row in rows:
        value = get(row, "s_odom.sample_time_us", "s_bmi088_sample.gyro_timestamp_us")
        source = "mcu_us"
        if value is None:
            value, source = row["host_ms"], "host_ms"
        sources[source if value is not None else "missing"] += 1
        row["epoch"] = epoch
        row["t"] = None
        row["duplicate"] = False
        if value is None:
            previous = None
            continue
        if previous is not None and previous[1] == source:
            delta = value - previous[0]
            if source == "mcu_us" and delta < -(2**31):
                delta += 2**32
            delta *= 1e-6 if source == "mcu_us" else 1e-3
            if delta < 0:
                resets += 1
                epoch += 1
            elif delta == 0:
                duplicates += 1
                row["duplicate"] = True
            else:
                elapsed += delta
                steps.append(delta)
        elif previous is not None:
            epoch += 1
        row["epoch"], row["t"] = epoch, elapsed
        previous = value, source
    median = statistics.median(steps) if steps else None
    gap_limit = max(0.25, 3 * median) if median else 0.25
    return {"timestamp_sources": dict(sources), "span_s": elapsed, "duplicates": duplicates,
            "clock_resets": resets, "positive_intervals": stats(steps),
            "median_observed_rate_hz": 1 / median if median else None,
            "gap_threshold_s": gap_limit, "observed_gaps": sum(s > gap_limit for s in steps)}, gap_limit


def axis_value(row, category, axis):
    paths = {
        "raw": (f"s_odom.gyro.{axis}", f"s_bmi088_sample.gyro.{axis}"),
        "corrected": (f"s_odom.gyro_corrected.{axis}", f"s_bmi088_attitude.gyro_filtered.{axis}"),
        "effective_bias": (f"s_odom.gyro_bias_effective.{axis}", f"s_bmi088_attitude.gyro_bias_effective.{axis}"),
        "startup_bias": (f"s_bmi088_attitude.gyro_bias.{axis}",),
        "residual_bias": (f"s_bmi088_attitude.gyro_bias_residual.{axis}",),
    }
    return get(row, *paths[category])


def usable(row):
    return row["firmware"] != "mismatch" and not row["duplicate"] and all(
        flag(row, f) is not False for f in ("imu_calibrated", "imu_ready", "timing_valid"))


def sparse_integral(rows, category, axis, gap_limit):
    total, duration, intervals = 0.0, 0.0, 0
    for first, second in zip(rows, rows[1:]):
        a, b = axis_value(first, category, axis), axis_value(second, category, axis)
        if not (usable(first) and usable(second)) or a is None or b is None:
            continue
        if first["t"] is None or second["t"] is None or first["epoch"] != second["epoch"]:
            continue
        dt = second["t"] - first["t"]
        if not 0 < dt <= gap_limit:
            continue
        total += 0.5 * (a+b) * dt
        duration += dt
        intervals += 1
    return {"radians": total, "degrees": math.degrees(total), "integrated_duration_s": duration,
            "intervals": intervals, "mean_rate_deg_per_min": math.degrees(total)*60/duration if duration else None}


def temperature(row):
    return get(row, "s_odom.temperature_c", "s_bmi088_sample.temperature")


def temperature_fit(rows, width, assume_stationary=False):
    eligible = [r for r in rows if usable(r) and flag(r, "temperature_valid") is not False
                and (assume_stationary or flag(r, "static_window") is not False) and temperature(r) is not None]
    bins = defaultdict(list)
    for row in eligible:
        bins[math.floor(temperature(row)/width)].append(row)
    table = []
    for key, group in sorted(bins.items()):
        table.append({"lower_c": key*width, "upper_c": (key+1)*width,
                      "mean_c": statistics.fmean(temperature(r) for r in group), "samples": len(group),
                      "raw_rad_s": {axis: stats(axis_value(r, "raw", axis) for r in group) for axis in "xyz"}})
    temps = [temperature(r) for r in eligible]
    span = max(temps)-min(temps) if temps else 0
    fits = {}
    for axis in "xyz":
        if len({r["epoch"] for r in eligible}) > 1:
            fits[axis] = {"status": "multiple_clock_epochs_no_pooled_fit"}
            continue
        points = [(b["mean_c"], b["raw_rad_s"][axis]["mean"]) for b in table if b["raw_rad_s"][axis]["samples"]]
        if len(points) < 2:
            fits[axis] = {"status": "insufficient_temperature_bins"}
            continue
        xm = statistics.fmean(p[0] for p in points); ym = statistics.fmean(p[1] for p in points)
        denominator = sum((x-xm)**2 for x, _ in points)
        if not denominator:
            fits[axis] = {"status": "insufficient_temperature_span"}
            continue
        slope = sum((x-xm)*(y-ym) for x,y in points)/denominator
        intercept = ym-slope*xm
        fits[axis] = {"status": "exploratory_only" if span >= 1 and len(points) >= 3 else "insufficient_temperature_span",
                      "slope_rad_s_per_c": slope, "intercept_rad_s": intercept,
                      "equal_weight_bin_residual_rms_rad_s": math.sqrt(statistics.fmean((y-(intercept+slope*x))**2 for x,y in points))}
    return {"temperature_c": stats(temps), "span_c": span, "bin_width_c": width,
            "bins": table, "raw_gyro_fit": fits,
            "stationarity_source": "externally_confirmed_by_caller" if assume_stationary else "firmware_static_window_when_observed",
            "fit_basis": "Equal weight per temperature bin; raw gyro only; vehicle must actually remain stationary"}


def pose_summary(rows, gap_limit):
    segments, current = [], []
    previous = None
    for r in rows:
        x=get(r,"s_odom.odom_interface.x","s_odom.odom.x")
        y=get(r,"s_odom.odom_interface.y","s_odom.odom.y")
        yaw=get(r,"s_odom.angle.z")
        accepted = usable(r) and flag(r,"fusion_ready") is True and flag(r,"static_window") is True
        accepted = accepted and x is not None and y is not None and yaw is not None and r["t"] is not None
        continuous = (previous is not None and r["epoch"] == previous["epoch"] and
                      r["index"] == previous["index"] + 1 and r["t"] is not None and
                      previous["t"] is not None and 0 < r["t"]-previous["t"] <= gap_limit)
        if not accepted or not continuous:
            if current:
                segments.append(current)
            current = []
        if accepted:
            current.append((r["t"], x, y, yaw))
        previous = r if accepted else None
    if current:
        segments.append(current)
    if not segments:
        return {"samples": 0, "continuous_segments": 0}
    points=max(segments, key=lambda p: p[-1][0]-p[0][0])
    if len(points)<2:
        return {"samples": len(points), "continuous_segments": len(segments)}
    first,last=points[0],points[-1];dt=last[0]-first[0]
    yaw_delta=sum(math.remainder(b[3]-a[3],2*math.pi) for a,b in zip(points,points[1:]))
    drift=math.hypot(last[1]-first[1],last[2]-first[2])
    return {"samples":len(points), "continuous_segments":len(segments),
            "selection":"longest continuous valid stationary segment; never across resets/movement/gaps",
            "span_s":dt,"delta_x_m":last[1]-first[1],"delta_y_m":last[2]-first[2],
            "endpoint_drift_m":drift,"yaw_delta_deg":math.degrees(yaw_delta),
            "position_drift_m_per_min":drift*60/dt if dt else None,
            "yaw_drift_deg_per_min":math.degrees(yaw_delta)*60/dt if dt else None,
            "interpretation":"Stationary constrained output; zero drift alone does not validate sensor bias"}


def gyro_norm_summary(rows, threshold=0.01):
    norms=[]
    for row in rows:
        xyz=[axis_value(row,"corrected",axis) for axis in "xyz"]
        if all(value is not None for value in xyz):
            norms.append(math.sqrt(sum(value*value for value in xyz)))
    result=stats(norms)
    if norms:
        ordered=sorted(norms)
        result.update({"p50":statistics.median(norms),"p99":ordered[round((len(ordered)-1)*0.99)],
                       "p999":ordered[round((len(ordered)-1)*0.999)],
                       "threshold_rad_s":threshold,"above_threshold":sum(v>threshold for v in norms),
                       "above_threshold_fraction":sum(v>threshold for v in norms)/len(norms)})
    return result


def read_coherence(rows):
    result={}
    for axis in "xyz":
        residual=[]
        for row in rows:
            raw=axis_value(row,"raw",axis)
            bias=axis_value(row,"effective_bias",axis)
            corrected=axis_value(row,"corrected",axis)
            if raw is not None and bias is not None and corrected is not None:
                residual.append(raw-bias-corrected)
        summary=stats(residual)
        summary["identity_abs_error_above_1e_6"] = sum(abs(v)>1e-6 for v in residual)
        result[axis]=summary
    return {"raw_minus_effective_bias_minus_corrected_rad_s":result,
            "interpretation":"Nonzero identities usually indicate mixed-cycle running reads; descriptive only, not a sensor fault test"}


def time_windows(rows, width_s=10.0):
    windows=defaultdict(list)
    for row in rows:
        if row["t"] is not None:
            windows[(row["epoch"],math.floor(row["t"]/width_s))].append(row)
    result=[]
    for (epoch,index),group in sorted(windows.items()):
        result.append({"epoch":epoch,"start_s":index*width_s,"end_s":(index+1)*width_s,
                       "samples":len(group),"temperature_c":stats(temperature(r) for r in group),
                       "raw_rad_s":{a:stats(axis_value(r,"raw",a) for r in group) for a in "xyz"},
                       "corrected_rad_s":{a:stats(axis_value(r,"corrected",a) for r in group) for a in "xyz"},
                       "corrected_norm_rad_s":gyro_norm_summary(group)})
    return result


def analyze(paths, bin_width=0.5, assume_stationary=False):
    rows, diagnostics, errors=load(paths)
    timing,gap_limit=set_times(rows)
    good=[r for r in rows if usable(r)]
    flags={name:{"true":sum(flag(r,name) is True for r in rows),"false":sum(flag(r,name) is False for r in rows),
                 "unobserved":sum(flag(r,name) is None for r in rows)}
           for name in ("imu_calibrated","imu_ready","timing_valid","wheel_valid","acc_valid","temperature_valid","fusion_ready","static_window")}
    groups={category:{axis:stats(axis_value(r,category,axis) for r in good) for axis in "xyz"}
            for category in ("raw","corrected","effective_bias","startup_bias","residual_bias")}
    wheel_rows=[r for r in good if flag(r,"wheel_valid") is True]
    return {"files":[str(p.resolve()) for p in paths],"sample_rows":len(rows),"usable_rows":len(good),
            "fully_qualified_imu_rows":sum(all(flag(r,f) is True for f in ("imu_calibrated","imu_ready","timing_valid")) for r in good),
            "firmware_consistency":dict(Counter(str(r["firmware"] or "unreported") for r in rows)),
            "elf_paths":sorted({str(r["elf"]) for r in rows if r["elf"]}),
            "elf_sha256":sorted({r["elf_sha256"] for r in rows if r["elf_sha256"]}),
            "diagnostic_rows":diagnostics,"variable_errors":errors,"partial_rows":sum(r["partial"] for r in rows),
            "flags":flags,"timing":timing,"gyro_rad_s":groups,
            "firmware_process_dt_s":stats(get(r,"s_odom.process_dt_s") for r in good),
            "corrected_gyro_norm_rad_s":gyro_norm_summary(good),
            "read_coherence":read_coherence(good),
            "ten_second_windows":time_windows(good),
            "sparse_gyro_integrals":{category:{a:sparse_integral(rows,category,a,gap_limit) for a in "xyz"} for category in ("raw","corrected")},
            "temperature":temperature_fit(rows,bin_width,assume_stationary),
            "wheel_velocity":{name:{a:stats(get(r,f"s_odom.{name}.{a}") for r in wheel_rows) for a in "xyz"} for name in ("wheel_raw","wheel_corrected","velocity")},
            "stationary_pose":pose_summary(rows,gap_limit),
            "discontinuity_counter":stats(get(r,"s_odom.discontinuity_count") for r in rows),
            "limits":["10-30 Hz observations describe slow drift; their standard deviation is not a calibrated 250 Hz measurement-noise variance or noise spectrum.",
                      "Sparse gyro integrals omit rejected/gapped intervals and cannot reproduce the firmware's full-rate trajectory.",
                      "Temperature-register reads are not independent temperature measurements; BMI088 updates internally about every 1.28 s.",
                      "Running multi-variable reads are not atomic; partial/mixed-cycle values and sensor flags must be reviewed.",
                      "A single stationary warm-up cannot validate dynamic scale, wheel geometry, slip, position accuracy or a repeatable thermal model."]}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("captures",type=Path,nargs="+")
    parser.add_argument("--temperature-bin",type=float,default=0.5,metavar="CELSIUS")
    parser.add_argument("--output",type=Path,help="Optional summary JSON; otherwise print to stdout")
    parser.add_argument("--assume-stationary",action="store_true",
                        help="Caller confirms physical standstill for raw temperature fitting only; does not override data/pose validity")
    args=parser.parse_args()
    if not math.isfinite(args.temperature_bin) or args.temperature_bin<=0:
        parser.error("--temperature-bin must be positive")
    try:
        report=analyze(args.captures,args.temperature_bin,args.assume_stationary)
    except (OSError,ValueError) as exc:
        parser.exit(1,str(exc)+"\n")
    text=json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+"\n"
    if args.output:
        args.output.write_text(text,encoding="utf-8")
        print(f"Analyzed {report['sample_rows']} samples -> {args.output.resolve()}")
    else:
        print(text,end="")


if __name__=="__main__":
    main()
