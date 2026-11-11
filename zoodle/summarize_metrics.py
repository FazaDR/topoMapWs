#!/usr/bin/env python3

import argparse
import csv
import math
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


DEFAULT_CSV_PATTERNS = ["*_metrics.csv", "*metrics*.csv"]


def discover_csv_files(paths):
    files = []
    for base in paths:
        p = Path(base)
        if p.is_dir():
            for pattern in DEFAULT_CSV_PATTERNS:
                files.extend(sorted(p.glob(pattern)))
        elif p.is_file():
            files.append(p)
    deduped = []
    seen = set()
    for f in files:
        key = str(f.resolve())
        if key not in seen:
            seen.add(key)
            deduped.append(f)
    return deduped


def safe_float(value, default=0.0):
    try:
        if value is None or value == "":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_int(value, default=0):
    try:
        if value is None or value == "":
            return default
        return int(float(value))
    except (TypeError, ValueError):
        return default


def load_rows(path):
    with path.open("r", newline="") as fh:
        reader = csv.DictReader(fh)
        rows = list(reader)
    return rows


def lidar_frame_count(bag_path):
    bag_path = Path(bag_path)
    metadata_path = bag_path / "metadata.yaml" if bag_path.is_dir() else bag_path
    if not metadata_path.is_file():
        raise FileNotFoundError(f"rosbag metadata file not found: {metadata_path}")

    lines = metadata_path.read_text(encoding="utf-8").splitlines()
    for index, line in enumerate(lines):
        if line.strip() != "name: /points/points":
            continue
        for following_line in lines[index + 1:index + 32]:
            match = re.match(r"\s*message_count:\s*(\d+)\s*$", following_line)
            if match:
                return int(match.group(1))
        break

    raise ValueError(f"/points/points message count not found in {metadata_path}")


def summarize_csv(path, total_frames=None):
    rows = load_rows(path)
    if not rows:
        return {
            "file": str(path),
            "total_frames": total_frames or 0,
            "processed_frames": 0,
            "skipped_frames": total_frames or 0,
            "processed_ratio": 0.0,
            "avg_processing_ms": 0.0,
            "p95_processing_ms": 0.0,
            "max_processing_ms": 0.0,
            "node_count": 0,
        }

    processing_ms = [safe_float(row.get("processing_ms"), 0.0) for row in rows]
    missed_deadline = [safe_int(row.get("missed_deadline"), 0) for row in rows]
    last_row = rows[-1]

    processed_frames = safe_int(last_row.get("processed_frames"), len(rows))
    skipped_frames = safe_int(last_row.get("skipped_frames"), 0)
    old_deadline_misses = safe_int(last_row.get("missed_frames"), sum(missed_deadline))
    supplied_total_frames = total_frames is not None
    total_frames = total_frames if supplied_total_frames else processed_frames + skipped_frames
    if supplied_total_frames:
        skipped_frames = max(0, total_frames - processed_frames)
    elif "skipped_frames" not in last_row:
        skipped_frames = max(0, total_frames - processed_frames)
    processed_ratio = (
        processed_frames / total_frames if total_frames else 0.0
    )
    node_count = safe_int(last_row.get("node_count"), 0)
    if node_count == 0:
        node_count = safe_int(last_row.get("layer1_node_count"), 0)

    avg_processing_ms = sum(processing_ms) / len(processing_ms) if processing_ms else 0.0
    sorted_ms = sorted(processing_ms)
    p95 = sorted_ms[min(len(sorted_ms) - 1, max(0, math.ceil(0.95 * len(sorted_ms)) - 1))]

    return {
        "file": str(path),
        "total_frames": total_frames,
        "processed_frames": processed_frames,
        "skipped_frames": skipped_frames,
        "processed_ratio": processed_ratio,
        "avg_processing_ms": avg_processing_ms,
        "p95_processing_ms": p95,
        "max_processing_ms": max(processing_ms, default=0.0),
        "node_count": node_count,
        "legacy_deadline_misses": old_deadline_misses,
    }


def print_summary_table(summaries):
    headers = [
        "file",
        "processed/total",
        "skipped",
        "processed_ratio",
        "avg_ms",
        "p95_ms",
        "max_ms",
        "node_count",
    ]

    rows = []
    for summary in summaries:
        rows.append([
            Path(summary["file"]).name,
            f"{summary['processed_frames']}/{summary['total_frames']}",
            summary["skipped_frames"],
            f"{summary['processed_ratio']:.2%}",
            f"{summary['avg_processing_ms']:.2f}",
            f"{summary['p95_processing_ms']:.2f}",
            f"{summary['max_processing_ms']:.2f}",
            summary["node_count"],
        ])

    widths = [len(h) for h in headers]
    for row in rows:
        widths = [max(widths[i], len(str(row[i]))) for i in range(len(headers))]

    def format_row(values):
        return " | ".join(str(v).ljust(widths[i]) for i, v in enumerate(values))

    print(format_row(headers))
    print("-+-".join("-" * width for width in widths))
    for row in rows:
        print(format_row(row))


def plot_summary(summaries, output_path):
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), constrained_layout=True)

    for summary in summaries:
        path = Path(summary["file"])
        rows = load_rows(path)
        if not rows:
            continue

        frames = [int(float(row.get("frame", idx + 1))) for idx, row in enumerate(rows)]
        proc = [safe_float(row.get("processing_ms"), 0.0) for row in rows]
        skipped = [safe_int(row.get("skipped_frames"), 0) for row in rows]

        axes[0].plot(frames, proc, marker="o", markersize=3, linewidth=1.5, label=path.stem)
        axes[1].plot(frames, skipped, marker="o", markersize=3, linewidth=1.5, label=path.stem)

    axes[0].set_title("Per-frame processing latency")
    axes[0].set_xlabel("Frame")
    axes[0].set_ylabel("Processing time (ms)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].set_title("Cumulative skipped frames")
    axes[1].set_xlabel("Frame")
    axes[1].set_ylabel("Skipped frames")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    fig.savefig(output_path, dpi=200)
    print(f"Saved plot to {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Summarize point-cloud metric CSV files and optionally plot latency results.")
    parser.add_argument("paths", nargs="*", default=["."], help="CSV files or directories to inspect. Defaults to the current directory.")
    parser.add_argument(
        "--bag",
        help="rosbag2 directory or metadata.yaml used to obtain the total /points/points frame count.",
    )
    parser.add_argument(
        "--total-frames",
        type=int,
        help="Total LiDAR frames, if a rosbag is not available.",
    )
    parser.add_argument("--plot", action="store_true", help="Plot processing time and missed-deadline flags for the selected CSVs.")
    parser.add_argument("--output", default="metrics_summary.png", help="Output filename for the plot image.")
    args = parser.parse_args()

    csv_files = discover_csv_files(args.paths)

    if not csv_files:
        print("No metric CSV files found.")
        return 1

    if args.bag and args.total_frames is not None:
        parser.error("--bag and --total-frames are mutually exclusive")

    total_frames = args.total_frames
    if args.bag:
        try:
            total_frames = lidar_frame_count(args.bag)
        except (FileNotFoundError, ValueError) as error:
            parser.error(str(error))

    summaries = [summarize_csv(path, total_frames) for path in csv_files]
    print_summary_table(summaries)

    if args.plot:
        plot_summary(summaries, args.output)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
