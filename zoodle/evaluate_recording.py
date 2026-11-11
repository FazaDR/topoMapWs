#!/usr/bin/env python3

import argparse
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(
        description="Replay a rosbag point cloud through the C++ mapping pipelines."
    )
    parser.add_argument("bag", type=Path, help="rosbag2 directory")
    parser.add_argument(
        "--mapping",
        default="all",
        help="atcdt, atcdt_free, mlatc, mlatc_free, both, or all",
    )
    parser.add_argument("--sample-period-ms", type=float, default=100.0)
    parser.add_argument("--metrics-dir", type=Path, default=Path("."))
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the launch command without running it",
    )
    args = parser.parse_args()

    if not args.bag.exists():
        parser.error(f"bag path does not exist: {args.bag}")
    if args.sample_period_ms <= 0.0:
        parser.error("--sample-period-ms must be positive")

    args.metrics_dir.mkdir(parents=True, exist_ok=True)
    command = [
        "ros2",
        "launch",
        "mapping",
        "evaluate_recording.launch.py",
        f"bag:={args.bag}",
        f"mapping:={args.mapping}",
        f"sample_period_ms:={args.sample_period_ms}",
        f"metrics_csv:={args.metrics_dir / 'atcdt_replay_metrics.csv'}",
        f"atcdt_free_metrics_csv:={args.metrics_dir / 'atcdt_free_replay_metrics.csv'}",
        f"mlatc_metrics_csv:={args.metrics_dir / 'mlatc_replay_metrics.csv'}",
        f"mlatc_free_metrics_csv:={args.metrics_dir / 'mlatc_free_replay_metrics.csv'}",
    ]

    print(" ".join(str(argument) for argument in command))
    if args.dry_run:
        return 0
    return subprocess.call(command)


if __name__ == "__main__":
    raise SystemExit(main())