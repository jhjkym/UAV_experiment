#!/usr/bin/env python3
"""Offline trajectory tracking metrics for CSV or structured JSON samples."""

import argparse
import csv
import json
import math
import statistics
from bisect import bisect_right
from pathlib import Path


TARGET_COLUMNS = ("target_x", "target_y", "target_z", "target_vx", "target_vy", "target_vz")
ACTUAL_COLUMNS = ("actual_x", "actual_y", "actual_z", "actual_vx", "actual_vy", "actual_vz")
REQUIRED_COLUMNS = ("time",) + TARGET_COLUMNS + ACTUAL_COLUMNS


class MetricsError(ValueError):
    pass


def _finite(value):
    return math.isfinite(value)


def _load_csv(path):
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        missing = [name for name in REQUIRED_COLUMNS if name not in (reader.fieldnames or [])]
        if missing:
            raise MetricsError("missing columns: " + ", ".join(missing))
        rows = []
        for row in reader:
            rows.append({name: float(row[name]) for name in REQUIRED_COLUMNS})
        return rows


def _load_json(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    rows = data.get("samples", data) if isinstance(data, dict) else data
    if not isinstance(rows, list):
        raise MetricsError("JSON input must be a list or contain a samples list")
    normalized = []
    for row in rows:
      missing = [name for name in REQUIRED_COLUMNS if name not in row]
      if missing:
          raise MetricsError("missing columns: " + ", ".join(missing))
      normalized.append({name: float(row[name]) for name in REQUIRED_COLUMNS})
    return normalized


def load_samples(path):
    suffix = Path(path).suffix.lower()
    if suffix == ".json":
        return _load_json(path)
    if suffix == ".csv":
        return _load_csv(path)
    raise MetricsError("input must be .csv or .json")


def _split_series(rows, prefix):
    return [
        {
            "time": row["time"],
            "x": row[f"{prefix}_x"],
            "y": row[f"{prefix}_y"],
            "z": row[f"{prefix}_z"],
            "vx": row[f"{prefix}_vx"],
            "vy": row[f"{prefix}_vy"],
            "vz": row[f"{prefix}_vz"],
        }
        for row in rows
    ]


def _clean_series(series):
    nan_inf = 0
    duplicates = 0
    non_increasing = 0
    by_time = {}
    last_time = None
    for row in series:
        values = [row[key] for key in ("time", "x", "y", "z", "vx", "vy", "vz")]
        if not all(_finite(value) for value in values):
            nan_inf += sum(1 for value in values if not _finite(value))
            continue
        if last_time is not None and row["time"] <= last_time:
            non_increasing += 1
        last_time = row["time"]
        if row["time"] in by_time:
            duplicates += 1
        by_time[row["time"]] = row
    cleaned = [by_time[key] for key in sorted(by_time)]
    return cleaned, {
        "nan_or_inf_values": nan_inf,
        "duplicate_timestamps": duplicates,
        "non_increasing_timestamps": non_increasing,
    }


def _interp(series, t):
    if t < series[0]["time"] or t > series[-1]["time"]:
        raise MetricsError("interpolation requested outside series range")
    index = bisect_right([row["time"] for row in series], t)
    if index == 0:
        return series[0]
    if index >= len(series):
        return series[-1]
    before = series[index - 1]
    after = series[index]
    dt = after["time"] - before["time"]
    if dt <= 0.0:
        raise MetricsError("cannot interpolate duplicate or non-increasing samples")
    ratio = (t - before["time"]) / dt
    out = {"time": t}
    for key in ("x", "y", "z", "vx", "vy", "vz"):
        out[key] = before[key] + ratio * (after[key] - before[key])
    return out


def _rms(values):
    if not values:
        return None
    return math.sqrt(sum(value * value for value in values) / len(values))


def _mean(values):
    if not values:
        return None
    return sum(values) / len(values)


def _detect_gaps(times):
    if len(times) < 3:
        return []
    intervals = [b - a for a, b in zip(times[:-1], times[1:])]
    nominal = statistics.median(intervals)
    if nominal <= 0.0:
        return []
    return [
        {"start": times[i], "end": times[i + 1], "dt": intervals[i]}
        for i in range(len(intervals))
        if intervals[i] > 2.5 * nominal
    ]


def _estimate_delay(times, target_errors, actual_errors):
    if len(times) < 8:
        return None
    dt_values = [b - a for a, b in zip(times[:-1], times[1:])]
    nominal_dt = statistics.median(dt_values)
    if nominal_dt <= 0.0:
        return None
    max_lag = min(20, len(times) // 4)
    best = None
    for lag in range(-max_lag, max_lag + 1):
        pairs = []
        for i, target_value in enumerate(target_errors):
            j = i + lag
            if 0 <= j < len(actual_errors):
                pairs.append((target_value, actual_errors[j]))
        if len(pairs) < 4:
            continue
        xs = [p[0] for p in pairs]
        ys = [p[1] for p in pairs]
        mx = _mean(xs)
        my = _mean(ys)
        denom = math.sqrt(sum((x - mx) ** 2 for x in xs) * sum((y - my) ** 2 for y in ys))
        corr = 0.0 if denom == 0.0 else sum((x - mx) * (y - my) for x, y in pairs) / denom
        if best is None or corr > best["correlation"]:
            best = {"lag_samples": lag, "delay_sec": lag * nominal_dt, "correlation": corr}
    if best is None:
        return None
    best["method"] = (
        "discrete cross-correlation of target and actual horizontal displacement "
        "magnitudes on the common resampled grid; this is an estimate only and "
        "is not proof of true system latency"
    )
    return best


def compute_metrics(rows, estimate_delay=False):
    if not rows:
        raise MetricsError("input contains no samples")
    target, target_stats = _clean_series(_split_series(rows, "target"))
    actual, actual_stats = _clean_series(_split_series(rows, "actual"))
    if len(target) < 2 or len(actual) < 2:
        raise MetricsError("at least two finite target and actual samples are required")

    start = max(target[0]["time"], actual[0]["time"])
    end = min(target[-1]["time"], actual[-1]["time"])
    if not end > start:
        raise MetricsError("target and actual time ranges do not overlap")

    target_times = [row["time"] for row in target if start <= row["time"] <= end]
    actual_times = [row["time"] for row in actual if start <= row["time"] <= end]
    grid = sorted(set(target_times + actual_times))
    if len(grid) < 2:
        raise MetricsError("not enough overlapping samples for metrics")

    ex, ey, ez = [], [], []
    ev = []
    h_errors, pos3_errors = [], []
    actual_speeds = []
    actual_accels = []
    target_disp = []
    actual_disp = []
    last_actual = None
    for t in grid:
        tgt = _interp(target, t)
        act = _interp(actual, t)
        dx = act["x"] - tgt["x"]
        dy = act["y"] - tgt["y"]
        dz = act["z"] - tgt["z"]
        ex.append(dx)
        ey.append(dy)
        ez.append(dz)
        h_errors.append(math.hypot(dx, dy))
        pos3_errors.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        dvx = act["vx"] - tgt["vx"]
        dvy = act["vy"] - tgt["vy"]
        dvz = act["vz"] - tgt["vz"]
        ev.append(math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz))
        actual_speeds.append(math.sqrt(act["vx"] ** 2 + act["vy"] ** 2 + act["vz"] ** 2))
        target_disp.append(math.hypot(tgt["x"] - target[0]["x"], tgt["y"] - target[0]["y"]))
        actual_disp.append(math.hypot(act["x"] - actual[0]["x"], act["y"] - actual[0]["y"]))
        if last_actual is not None:
            dt = act["time"] - last_actual["time"]
            if dt > 0.0:
                actual_accels.append(
                    math.sqrt(
                        ((act["vx"] - last_actual["vx"]) / dt) ** 2
                        + ((act["vy"] - last_actual["vy"]) / dt) ** 2
                        + ((act["vz"] - last_actual["vz"]) / dt) ** 2
                    )
                )
        last_actual = act

    target_duration = target[-1]["time"] - target[0]["time"]
    actual_duration = actual[-1]["time"] - actual[0]["time"]
    common_duration = end - start
    summary = {
        "sample_count": len(grid),
        "valid_duration_sec": common_duration,
        "interpolation": "linear on the common target/actual time range",
        "position_mean_error_m": {"x": _mean(ex), "y": _mean(ey), "z": _mean(ez)},
        "position_rms_error_m": {"x": _rms(ex), "y": _rms(ey), "z": _rms(ez)},
        "position_3d_rms_error_m": _rms(pos3_errors),
        "position_3d_max_error_m": max(pos3_errors),
        "horizontal_rms_error_m": _rms(h_errors),
        "horizontal_max_error_m": max(h_errors),
        "height_mean_error_m": _mean(ez),
        "height_rms_error_m": _rms(ez),
        "velocity_rms_error_mps": _rms(ev),
        "max_actual_speed_mps": max(actual_speeds),
        "max_actual_acceleration_mps2": max(actual_accels) if actual_accels else 0.0,
        "coverage": {
            "target_time_coverage": common_duration / target_duration if target_duration > 0 else 0.0,
            "actual_time_coverage": common_duration / actual_duration if actual_duration > 0 else 0.0,
        },
        "dropped_or_anomalous_intervals": {
            "target": _detect_gaps([row["time"] for row in target]),
            "actual": _detect_gaps([row["time"] for row in actual]),
        },
        "input_quality": {
            "target": target_stats,
            "actual": actual_stats,
            "total_nan_or_inf_values": target_stats["nan_or_inf_values"] + actual_stats["nan_or_inf_values"],
        },
    }
    if estimate_delay:
        summary["delay_estimate"] = _estimate_delay(grid, target_disp, actual_disp)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input")
    parser.add_argument("--estimate-delay", action="store_true")
    parser.add_argument("--output", "-o")
    args = parser.parse_args()
    try:
        summary = compute_metrics(load_samples(args.input), estimate_delay=args.estimate_delay)
    except MetricsError as exc:
        raise SystemExit(str(exc))
    text = json.dumps(summary, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    else:
        print(text)


if __name__ == "__main__":
    main()
