#!/usr/bin/env python3
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from trajectory_tracking_metrics import MetricsError, compute_metrics


def row(t, target, actual, target_v=(1.0, 0.0, 0.0), actual_v=(1.0, 0.0, 0.0)):
    return {
        "time": t,
        "target_x": target[0],
        "target_y": target[1],
        "target_z": target[2],
        "actual_x": actual[0],
        "actual_y": actual[1],
        "actual_z": actual[2],
        "target_vx": target_v[0],
        "target_vy": target_v[1],
        "target_vz": target_v[2],
        "actual_vx": actual_v[0],
        "actual_vy": actual_v[1],
        "actual_vz": actual_v[2],
    }


class TrackingMetricsTest(unittest.TestCase):
    def test_zero_error_and_fields(self):
        rows = [row(float(i), (i, 0.0, 1.0), (i, 0.0, 1.0)) for i in range(5)]
        summary = compute_metrics(rows)
        self.assertEqual(summary["sample_count"], 5)
        self.assertAlmostEqual(summary["position_3d_rms_error_m"], 0.0)
        self.assertAlmostEqual(summary["horizontal_max_error_m"], 0.0)
        for key in (
            "position_mean_error_m",
            "position_rms_error_m",
            "velocity_rms_error_mps",
            "max_actual_speed_mps",
            "max_actual_acceleration_mps2",
            "coverage",
            "input_quality",
        ):
            self.assertIn(key, summary)

    def test_fixed_bias_and_rms(self):
        rows = [
            row(float(i), (i, 0.0, 1.0), (i + 1.0, -2.0, 1.5))
            for i in range(4)
        ]
        summary = compute_metrics(rows)
        self.assertAlmostEqual(summary["position_mean_error_m"]["x"], 1.0)
        self.assertAlmostEqual(summary["position_mean_error_m"]["y"], -2.0)
        self.assertAlmostEqual(summary["height_mean_error_m"], 0.5)
        self.assertAlmostEqual(summary["horizontal_rms_error_m"], math.hypot(1.0, 2.0))
        self.assertAlmostEqual(summary["position_3d_rms_error_m"], math.sqrt(1.0 + 4.0 + 0.25))

    def test_different_sampling_rates_resample(self):
        rows = [
            row(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            row(0.5, (float("nan"), 0.0, 0.0), (0.5, 0.0, 0.0)),
            row(1.0, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            row(1.5, (float("nan"), 0.0, 0.0), (1.5, 0.0, 0.0)),
            row(2.0, (2.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
        ]
        summary = compute_metrics(rows)
        self.assertGreaterEqual(summary["sample_count"], 4)
        self.assertAlmostEqual(summary["position_3d_max_error_m"], 0.0)

    def test_time_not_overlapping_rejected(self):
        rows = [
            row(0.0, (0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0)),
            row(1.0, (1.0, 0.0, 0.0), (float("nan"), 0.0, 0.0)),
            row(10.0, (float("nan"), 0.0, 0.0), (10.0, 0.0, 0.0)),
            row(11.0, (float("nan"), 0.0, 0.0), (11.0, 0.0, 0.0)),
        ]
        with self.assertRaises(MetricsError):
            compute_metrics(rows)

    def test_duplicate_timestamp_and_nan_inf_statistics(self):
        rows = [
            row(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            row(1.0, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            row(1.0, (1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            row(2.0, (2.0, 0.0, 0.0), (float("nan"), 0.0, 0.0)),
            row(8.0, (8.0, 0.0, 0.0), (8.0, 0.0, 0.0)),
        ]
        summary = compute_metrics(rows)
        self.assertGreaterEqual(summary["input_quality"]["target"]["duplicate_timestamps"], 1)
        self.assertGreaterEqual(summary["input_quality"]["total_nan_or_inf_values"], 1)
        self.assertTrue(summary["dropped_or_anomalous_intervals"]["target"])

    def test_empty_and_single_sample_rejected(self):
        with self.assertRaises(MetricsError):
            compute_metrics([])
        with self.assertRaises(MetricsError):
            compute_metrics([row(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))])


if __name__ == "__main__":
    unittest.main()
