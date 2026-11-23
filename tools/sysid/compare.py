#!/usr/bin/env python3
"""Compare bundled BAM XL330 M1-M6 versus optional locally fitted versions.

Run inside a clone of Rhoban/bam with:
    uv run python /path/to/compare_xl330_bam.py --logdir data_processed \
        --custom-dir my_params --output-dir comparison
This runs the BAM reference simulator and reports position MAE on each processed log.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import sys

import numpy as np

try:
    from bam.model import load_model
    from bam.simulate import Simulator
except ImportError as exc:
    sys.exit(
        f"BAM not installed / not in PYTHONPATH: {exc}. Run this inside the BAM repository using uv run."
    )


def evaluate(model, log):
    predicted, predicted_velocity, _ = Simulator(model).rollout_log(
        log, simulate_control=True
    )
    actual_q = np.array([entry["position"] for entry in log["entries"]], dtype=float)
    actual_dq = np.array([entry["speed"] for entry in log["entries"]], dtype=float)
    q = np.array(predicted, dtype=float)
    dq = np.array(predicted_velocity, dtype=float)
    return q.tolist(), {
        "mae_deg": float(np.mean(np.abs(q - actual_q)) * 180.0 / np.pi),
        "rmse_deg": float(np.sqrt(np.mean((q - actual_q) ** 2)) * 180.0 / np.pi),
        "speed_mae_rad_s": float(np.mean(np.abs(dq - actual_dq))),
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--logdir",
        required=True,
        type=Path,
        help="Folder of processed BAM logs (containing 'dt').",
    )
    p.add_argument(
        "--custom-dir",
        type=Path,
        default=None,
        help="Optional folder of fitted m1.json ... m6.json",
    )
    p.add_argument("--output-dir", type=Path, default=Path("comparison"))
    p.add_argument(
        "--holdout-kp",
        type=int,
        default=400,
        help="P gain to report as a validation holdout; default=400",
    )
    args = p.parse_args()

    logs = []
    for path in sorted(args.logdir.glob("*.json")):
        with path.open() as stream:
            log = json.load(stream)
        if log.get("result", "complete") != "complete":
            sys.exit(f"Incomplete recording: {path}")
        if "arm_mass" not in log:
            log["arm_mass"] = log.get("arm-mass", 0)
        if "dt" not in log:
            sys.exit(f"{path} is raw; run 'uv run python -m bam.process' first")
        if len(log.get("entries", [])) < 2:
            sys.exit(f"Too few samples: {path}")
        logs.append((path.name, log))
    if not logs:
        sys.exit(f"No JSON logs in {args.logdir}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    model_params = {}
    for m in range(1, 7):
        name = f"m{m}"
        candidates = [("BAM", None)]
        if args.custom_dir and (args.custom_dir / f"{name}.json").is_file():
            candidates.append(("custom", args.custom_dir / f"{name}.json"))
        for source, custom_path in candidates:
            model = (
                load_model(str(custom_path))
                if custom_path
                else load_model(motor_name="xl330", model=name)
            )
            for filename, log in logs:
                try:
                    prediction, metrics = evaluate(model, log)
                    report = {
                        "trajectory": log["trajectory"],
                        "entries": log["entries"],
                        "predicted": prediction,
                        "mae_deg": metrics["mae_deg"],
                    }
                    (args.output_dir / f"{source}-{name}-{filename}").write_text(
                        json.dumps(report)
                    )
                except Exception as exc:
                    raise RuntimeError(
                        f"Failed for {source}/{name}, log={filename}"
                    ) from exc
                rows.append(
                    {
                        "source": source,
                        "model": name,
                        "file": filename,
                        "trajectory": log.get("trajectory", "unknown"),
                        "kp": log.get("kp"),
                        "mass_kg": log.get("mass"),
                        "length_m": log.get("length"),
                        "set": (
                            "validation"
                            if log.get("kp") == args.holdout_kp
                            else "train"
                        ),
                        **metrics,
                    }
                )
            if custom_path:
                with custom_path.open() as f:
                    model_params[name] = json.load(f)

    # Fit a first-order position response per gain on independent training logs.
    # Torque-off trajectories require a passive dynamics model.
    for kp in {log["kp"] for _, log in logs}:
        trials = [
            (name, log)
            for name, log in logs
            if log["kp"] == kp and all(e["torque_enable"] for e in log["entries"])
        ]
        training = [
            (name, log)
            for name, log in trials
            if kp != args.holdout_kp and log["trajectory"] != "steps"
        ]
        if not training:
            continue
        dt = training[0][1]["dt"]
        x, y = [], []
        for _, log in training:
            if log["dt"] != dt:
                raise ValueError("Use the same processed dt for the first-order fit")
            pos = np.array([e["position"] for e in log["entries"]])
            goal = np.array([e["goal_position"] for e in log["entries"]])
            x.extend(zip(goal[:-1], -pos[:-1]))
            y.extend(np.diff(pos))
        coeff, _, rank, _ = np.linalg.lstsq(x, y, rcond=None)
        b, decay = coeff
        if rank < 2 or not 0 < decay < 1 or b <= 0:
            continue
        for name, log in trials:
            if log["dt"] != dt:
                continue
            entries = log["entries"]
            pred = [entries[0]["position"]]
            for e in entries[:-1]:
                pred.append((1 - decay) * pred[-1] + b * e["goal_position"])
            mae = float(
                np.rad2deg(
                    np.mean(np.abs(np.array(pred) - [e["position"] for e in entries]))
                )
            )
            report = dict(
                trajectory=log["trajectory"],
                entries=entries,
                predicted=pred,
                mae_deg=mae,
                gain=float(b / decay),
                time_constant_s=float(-dt / np.log1p(-decay)),
                training_files=[filename for filename, _ in training],
            )
            (args.output_dir / f"first-order-{name}").write_text(json.dumps(report))

    keys = list(rows[0])
    with (args.output_dir / "per_log_metrics.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)

    aggregate = defaultdict(list)
    trajectory = defaultdict(list)
    for row in rows:
        aggregate[(row["source"], row["model"], row["set"])].append(row["mae_deg"])
        trajectory[(row["source"], row["model"], row["trajectory"])].append(
            row["mae_deg"]
        )

    print(f"Processed logs: {len(logs)}, holdout Kp={args.holdout_kp}")
    print("Mean position MAE in degrees (mean of per-log MAE, same metric as BAM fit):")
    print(f"{'source':<9}{'model':<8}{'train':>12}{'validation':>14}")
    summary = []
    for source in ("BAM", "custom"):
        for m in range(1, 7):
            model = f"m{m}"
            if not any(x["source"] == source and x["model"] == model for x in rows):
                continue
            scores = {}
            for split in ("train", "validation"):
                values = aggregate[(source, model, split)]
                scores[split] = float(np.mean(values)) if values else None
            train = f"{scores['train']:.3f}" if scores["train"] is not None else "--"
            valid = (
                f"{scores['validation']:.3f}"
                if scores["validation"] is not None
                else "--"
            )
            print(f"{source:<9}{model:<8}{train:>12}{valid:>14}")
            summary.append(
                {
                    "source": source,
                    "model": model,
                    "train_mae_deg": scores["train"],
                    "validation_mae_deg": scores["validation"],
                }
            )

    with (args.output_dir / "summary_metrics.csv").open("w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=["source", "model", "train_mae_deg", "validation_mae_deg"]
        )
        writer.writeheader()
        writer.writerows(summary)

    print("\nMAE by trajectory (deg):")
    for (source, model, traj), values in sorted(trajectory.items()):
        print(f"  {source:<7} {model:<3} {traj:<19}: {np.mean(values):.3f}")

    if model_params:
        print("\nCustom vs BAM bundled parameter differences:")
        for name, custom in model_params.items():
            # Compare via distributed package's own JSON, not a duplicate reference table.
            from bam.model import _resolve_json_path  # BAM path helper as of main

            with Path(_resolve_json_path(None, "xl330", name)).open() as f:
                ref = json.load(f)
            print(f"  {name}:")
            for key, val in custom.items():
                if key in ("model", "actuator") or not isinstance(val, (int, float)):
                    continue
                orig = ref.get(key)
                if not isinstance(orig, (int, float)):
                    continue
                diff = val - orig
                pct = f"{100*diff/abs(orig):+.1f}%" if abs(orig) > 1e-9 else "n/a"
                print(f"    {key:<32} {orig:.6g} -> {val:.6g} ({pct})")

    print(f"\nSaved: {args.output_dir}/summary_metrics.csv and per_log_metrics.csv")


if __name__ == "__main__":
    main()
