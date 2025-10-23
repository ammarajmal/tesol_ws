#!/usr/bin/env python3
"""Utilities for comparing camera-based displacement measurements with LDS data.

This module provides a command line interface that loads displacement measurements
recorded by multiple cameras and a Laser Displacement Sensor (LDS).  The script
interpolates all signals onto a shared time base, computes time-domain and
frequency-domain error metrics, and (optionally) produces plots and JSON reports.

Example usage
-------------

```
python3 analysis/compare_displacements.py \
    --camera-files camera1.csv camera2.csv camera3.csv \
    --lds-file lds.csv \
    --time-column stamp \
    --value-column x \
    --lds-time-column stamp \
    --lds-value-column displacement \
    --time-scale 1e-9 \
    --output-dir results --plot
```

The CSV files are expected to contain at least a time column and a column with
x-axis displacement measurements.  The ``--time-scale`` option is convenient for
ROS timestamps that are recorded in nanoseconds.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:  # Optional dependency used only when generating plots.
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover - plotting is optional.
    plt = None


@dataclass
class TimeSeries:
    """Container storing a single-channel time series."""

    label: str
    time: np.ndarray
    values: np.ndarray

    def trimmed(self, start: float, end: float) -> "TimeSeries":
        """Return a copy with samples outside ``[start, end]`` removed."""

        mask = (self.time >= start) & (self.time <= end)
        return TimeSeries(self.label, self.time[mask], self.values[mask])


def parse_arguments(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare multi-camera displacement measurements with an LDS signal.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--camera-files",
        nargs="+",
        required=True,
        help="CSV files containing camera displacement measurements.",
    )
    parser.add_argument(
        "--lds-file", required=True, help="CSV file containing the LDS displacement signal."
    )
    parser.add_argument(
        "--time-column",
        default="timestamp",
        help="Time column used by the camera CSV files.",
    )
    parser.add_argument(
        "--value-column",
        default="x",
        help="Displacement column used by the camera CSV files.",
    )
    parser.add_argument(
        "--lds-time-column",
        default="timestamp",
        help="Time column used by the LDS CSV file.",
    )
    parser.add_argument(
        "--lds-value-column",
        default="x",
        help="Displacement column used by the LDS CSV file.",
    )
    parser.add_argument(
        "--time-scale",
        type=float,
        default=1.0,
        help="Scale applied to camera timestamps (e.g., 1e-9 for ROS nanoseconds).",
    )
    parser.add_argument(
        "--lds-time-scale",
        type=float,
        default=None,
        help="Optional scale for LDS timestamps; defaults to --time-scale.",
    )
    parser.add_argument(
        "--delimiter",
        default=",",
        help="Delimiter used by all CSV files.",
    )
    parser.add_argument(
        "--resample-rate",
        type=float,
        default=None,
        help="Target sampling rate (Hz).  If omitted it is inferred from the data.",
    )
    parser.add_argument(
        "--aggregate",
        choices=["mean", "median"],
        default="mean",
        help="How to aggregate the resampled camera signals.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory where reports and plots are stored.",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Generate diagnostic plots (requires matplotlib).",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Optional path for a JSON summary (overrides --output-dir/summary.json).",
    )
    parser.add_argument(
        "--camera-labels",
        nargs="+",
        default=None,
        help="Readable labels for each camera (defaults to filenames).",
    )
    parser.add_argument(
        "--save-resampled",
        action="store_true",
        help="Write the resampled time series to CSV files in the output directory.",
    )
    parser.add_argument(
        "--baseline-only",
        action="store_true",
        help="Skip per-camera metrics and focus solely on aggregated vs. LDS results.",
    )

    return parser.parse_args(argv)


def load_csv_series(
    path: Path,
    label: str,
    time_column: str,
    value_column: str,
    delimiter: str,
    time_scale: float,
) -> TimeSeries:
    with path.open("r", newline="") as file:
        reader = csv.DictReader(file, delimiter=delimiter)
        if reader.fieldnames is None:
            raise ValueError(f"File '{path}' is missing a header row.")
        if time_column not in reader.fieldnames:
            raise ValueError(
                f"Time column '{time_column}' not found in '{path}'. Available columns: {reader.fieldnames}"
            )
        if value_column not in reader.fieldnames:
            raise ValueError(
                f"Value column '{value_column}' not found in '{path}'. Available columns: {reader.fieldnames}"
            )

        time: List[float] = []
        values: List[float] = []
        for row in reader:
            try:
                time.append(float(row[time_column]) * time_scale)
                values.append(float(row[value_column]))
            except (ValueError, TypeError):
                # Skip rows with malformed data.
                continue

    time_array = np.asarray(time, dtype=np.float64)
    value_array = np.asarray(values, dtype=np.float64)
    if time_array.size == 0:
        raise ValueError(f"File '{path}' does not contain any valid rows for the requested columns.")

    order = np.argsort(time_array)
    time_sorted = time_array[order]
    values_sorted = value_array[order]
    unique_times, unique_indices = np.unique(time_sorted, return_index=True)
    values_unique = values_sorted[unique_indices]
    return TimeSeries(label=label, time=unique_times, values=values_unique)


def infer_resample_rate(series_list: Sequence[TimeSeries]) -> float:
    cadences: List[float] = []
    for series in series_list:
        if series.time.size < 2:
            continue
        diffs = np.diff(series.time)
        positive_diffs = diffs[diffs > 0]
        if positive_diffs.size == 0:
            continue
        cadences.append(np.median(positive_diffs))
    if not cadences:
        raise ValueError("Unable to infer a resample rate; provide --resample-rate explicitly.")
    min_step = min(cadences)
    if min_step <= 0:
        raise ValueError("Inferred a non-positive sampling step; check the timestamps.")
    return 1.0 / min_step


def build_time_grid(series_list: Sequence[TimeSeries], sample_rate: float) -> np.ndarray:
    start = max(series.time[0] for series in series_list)
    end = min(series.time[-1] for series in series_list)
    if end <= start:
        raise ValueError("Time series do not overlap; cannot build a common time grid.")
    step = 1.0 / sample_rate
    count = int(math.floor((end - start) / step)) + 1
    return start + np.arange(count, dtype=np.float64) * step


def resample_series(series: TimeSeries, new_time: np.ndarray) -> np.ndarray:
    trimmed = series.trimmed(new_time[0], new_time[-1])
    if trimmed.time.size < 2:
        raise ValueError(
            f"Time series '{series.label}' does not contain enough samples after trimming to the overlap window."
        )
    return np.interp(new_time, trimmed.time, trimmed.values)


def aggregate_signals(signals: np.ndarray, method: str) -> np.ndarray:
    if method == "mean":
        return np.nanmean(signals, axis=0)
    if method == "median":
        return np.nanmedian(signals, axis=0)
    raise ValueError(f"Unknown aggregation method: {method}")


def compute_time_domain_metrics(reference: np.ndarray, test: np.ndarray) -> Dict[str, float]:
    error = test - reference
    mae = float(np.mean(np.abs(error)))
    rmse = float(np.sqrt(np.mean(error**2)))
    bias = float(np.mean(error))
    max_error = float(np.max(np.abs(error)))
    if reference.size > 1 and np.std(reference) > 0 and np.std(test) > 0:
        corr = float(np.corrcoef(reference, test)[0, 1])
    else:
        corr = float("nan")
    return {
        "mae": mae,
        "rmse": rmse,
        "bias": bias,
        "max_abs_error": max_error,
        "corr_coeff": corr,
    }


def compute_frequency_spectrum(signal: np.ndarray, sample_rate: float) -> Tuple[np.ndarray, np.ndarray]:
    detrended = signal - np.mean(signal)
    n = detrended.size
    if n < 2:
        raise ValueError("Need at least two samples to compute a frequency spectrum.")
    fft = np.fft.rfft(detrended)
    amplitude = np.abs(fft) / n
    amplitude[1:-1] *= 2  # Preserve total energy when using a one-sided spectrum.
    freqs = np.fft.rfftfreq(n, d=1.0 / sample_rate)
    return freqs, amplitude


def compute_frequency_domain_metrics(
    reference: np.ndarray, test: np.ndarray, sample_rate: float
) -> Dict[str, float]:
    freqs, ref_amp = compute_frequency_spectrum(reference, sample_rate)
    _, test_amp = compute_frequency_spectrum(test, sample_rate)
    diff = test_amp - ref_amp
    rmse = float(np.sqrt(np.mean(diff**2)))
    mae = float(np.mean(np.abs(diff)))
    if ref_amp.size > 1:
        ref_idx = int(np.argmax(ref_amp[1:])) + 1
        test_idx = int(np.argmax(test_amp[1:])) + 1
        dominant_freq_ref = float(freqs[ref_idx])
        dominant_freq_test = float(freqs[test_idx])
    else:
        dominant_freq_ref = float(freqs[0])
        dominant_freq_test = float(freqs[0])
    if ref_amp.size > 1 and np.std(ref_amp[1:]) > 0 and np.std(test_amp[1:]) > 0:
        spectrum_corr = float(np.corrcoef(ref_amp[1:], test_amp[1:])[0, 1])
    else:
        spectrum_corr = float("nan")
    return {
        "spectrum_mae": mae,
        "spectrum_rmse": rmse,
        "dominant_freq_reference": dominant_freq_ref,
        "dominant_freq_test": dominant_freq_test,
        "dominant_freq_error": float(abs(dominant_freq_ref - dominant_freq_test)),
        "spectrum_corr": spectrum_corr,
    }


def export_resampled_series(
    output_dir: Path,
    time_base: np.ndarray,
    lds_signal: np.ndarray,
    camera_signals: Dict[str, np.ndarray],
    aggregate_signal: np.ndarray,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    header = ["time", "lds"] + list(camera_signals.keys()) + ["aggregate"]
    rows = zip(
        time_base,
        lds_signal,
        *camera_signals.values(),
        aggregate_signal,
    )
    output_path = output_dir / "resampled_signals.csv"
    with output_path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(header)
        for row in rows:
            writer.writerow(row)


def plot_results(
    output_dir: Path,
    time_base: np.ndarray,
    lds_signal: np.ndarray,
    camera_signals: Dict[str, np.ndarray],
    aggregate_signal: np.ndarray,
    sample_rate: float,
) -> None:
    if plt is None:
        raise RuntimeError("matplotlib is not available; install it or omit --plot.")

    output_dir.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=(12, 8))
    ax1 = plt.subplot(3, 1, 1)
    ax1.plot(time_base, lds_signal, label="LDS", linewidth=2)
    for name, signal in camera_signals.items():
        ax1.plot(time_base, signal, label=name, alpha=0.7)
    ax1.plot(time_base, aggregate_signal, label="Aggregate", linewidth=2, linestyle="--")
    ax1.set_title("Displacement comparison (time domain)")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Displacement")
    ax1.legend(loc="upper right", ncol=2)
    ax1.grid(True, linestyle=":")

    ax2 = plt.subplot(3, 1, 2)
    ax2.plot(time_base, aggregate_signal - lds_signal, label="Aggregate error", color="tab:red")
    ax2.set_title("Aggregate error vs. LDS")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Error")
    ax2.grid(True, linestyle=":")

    ax3 = plt.subplot(3, 1, 3)
    freqs, lds_amp = compute_frequency_spectrum(lds_signal, sample_rate)
    _, agg_amp = compute_frequency_spectrum(aggregate_signal, sample_rate)
    ax3.semilogy(freqs, lds_amp, label="LDS")
    ax3.semilogy(freqs, agg_amp, label="Aggregate")
    ax3.set_title("Frequency domain comparison")
    ax3.set_xlabel("Frequency [Hz]")
    ax3.set_ylabel("Amplitude")
    ax3.grid(True, which="both", linestyle=":")
    ax3.legend()

    plt.tight_layout()
    plt.savefig(output_dir / "comparison.png", dpi=200)
    plt.close()


def summarise_metrics(
    aggregated_metrics: Dict[str, float],
    frequency_metrics: Dict[str, float],
    per_camera: Dict[str, Dict[str, float]],
    sample_rate: float,
) -> Dict[str, object]:
    summary: Dict[str, object] = {
        "sample_rate_hz": float(sample_rate),
        "aggregate": {
            "time_domain": aggregated_metrics,
            "frequency_domain": frequency_metrics,
        },
    }
    if per_camera:
        summary["per_camera"] = per_camera
    return summary


def main(argv: Optional[Sequence[str]] = None) -> Dict[str, object]:
    args = parse_arguments(argv)

    camera_paths = [Path(p) for p in args.camera_files]
    if args.camera_labels and len(args.camera_labels) != len(camera_paths):
        raise ValueError("Number of camera labels must match number of camera files.")
    camera_labels = args.camera_labels or [path.stem for path in camera_paths]

    lds_path = Path(args.lds_file)

    time_scale = args.time_scale
    lds_time_scale = args.lds_time_scale if args.lds_time_scale is not None else time_scale

    camera_series = [
        load_csv_series(
            path,
            label,
            time_column=args.time_column,
            value_column=args.value_column,
            delimiter=args.delimiter,
            time_scale=time_scale,
        )
        for path, label in zip(camera_paths, camera_labels)
    ]
    lds_series = load_csv_series(
        lds_path,
        label="LDS",
        time_column=args.lds_time_column,
        value_column=args.lds_value_column,
        delimiter=args.delimiter,
        time_scale=lds_time_scale,
    )

    resample_rate = args.resample_rate or infer_resample_rate([lds_series] + camera_series)
    time_base = build_time_grid([lds_series] + camera_series, resample_rate)

    lds_resampled = resample_series(lds_series, time_base)
    camera_resampled = {
        series.label: resample_series(series, time_base) for series in camera_series
    }

    stacked = np.vstack(list(camera_resampled.values()))
    aggregate_signal = aggregate_signals(stacked, args.aggregate)

    per_camera_metrics: Dict[str, Dict[str, float]] = {}
    if not args.baseline_only:
        for label, signal in camera_resampled.items():
            per_camera_metrics[label] = compute_time_domain_metrics(lds_resampled, signal)

    aggregate_time_metrics = compute_time_domain_metrics(lds_resampled, aggregate_signal)
    frequency_metrics = compute_frequency_domain_metrics(
        lds_resampled, aggregate_signal, resample_rate
    )

    summary = summarise_metrics(aggregate_time_metrics, frequency_metrics, per_camera_metrics, resample_rate)

    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        if args.save_resampled:
            export_resampled_series(
                args.output_dir, time_base, lds_resampled, camera_resampled, aggregate_signal
            )
        if args.plot:
            plot_results(
                args.output_dir, time_base, lds_resampled, camera_resampled, aggregate_signal, resample_rate
            )
        if args.summary_json is None:
            summary_path = args.output_dir / "summary.json"
            with summary_path.open("w", encoding="utf-8") as file:
                json.dump(summary, file, indent=2)

    if args.summary_json:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        with args.summary_json.open("w", encoding="utf-8") as file:
            json.dump(summary, file, indent=2)

    print(json.dumps(summary, indent=2))
    return summary


if __name__ == "__main__":  # pragma: no cover - CLI entry point.
    main()
