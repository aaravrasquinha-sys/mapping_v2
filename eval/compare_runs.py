#!/usr/bin/env python3
"""Compare evaluated runs side by side against the fixed 'clean' targets (PLAN.md).

For every run folder under RUNS_ROOT that has metrics.json (or the ones named): a table with
pass/fail marks, pairwise agreement between runs on the same dataset (translation RMSE after
SE(3) alignment over common frames), and one image stacking every run's views.png.
Writes RUNS_ROOT/compare.md and RUNS_ROOT/compare.png.

Usage: python eval/compare_runs.py RUNS_ROOT [RUN_NAME ...] [--dataset NAME]
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mv2lib.traj import read_tum, umeyama  # noqa: E402

# (label, getter, target test, format)  -- targets as fixed in PLAN.md
CHECKS = [
    ("posed", lambda m: m["coverage"]["fraction"], lambda v: v >= 1.0, "{:.1%}"),
    ("pieces", lambda m: m["coverage"]["pieces"], lambda v: v == 1, "{}"),
    ("speed>1.5", lambda m: m["speed"]["frames_over_limit"], lambda v: v == 0, "{}"),
    ("jump frames", lambda m: len(m["outlier_frames"]), lambda v: v == 0, "{}"),
    ("gyro<=1deg", lambda m: m["gyro"]["frac_le_1deg"], lambda v: v >= 0.99, "{:.1%}"),
    ("gyro max", lambda m: m["gyro"]["err_max_deg"], lambda v: v <= 3.0, "{:.2f}°"),
    ("bias ok", lambda m: m["gyro"]["bias_plausible"], lambda v: bool(v), "{}"),
    ("revisit med", lambda m: m["revisits"]["trans_median_m"], lambda v: v <= 0.02, "{:.3f} m"),
    ("revisit p90", lambda m: m["revisits"]["trans_p90_m"], lambda v: v <= 0.05, "{:.3f} m"),
    ("rev rot med", lambda m: m["revisits"]["rot_median_deg"], lambda v: v <= 0.5, "{:.2f}°"),
    ("rev rot p90", lambda m: m["revisits"]["rot_p90_deg"], lambda v: v <= 1.5, "{:.2f}°"),
    ("start-end", lambda m: m["revisits"]["start_end"]["trans_m"], lambda v: v <= 0.05, "{:.3f} m"),
    ("sharpness", lambda m: m["sharpness"]["ratio"], lambda v: v <= 1.5, "{:.2f}"),
    ("floor tilt", lambda m: m["map"]["floor"]["tilt_deg"], lambda v: v <= 1.0, "{:.2f}°"),
    ("floor flat", lambda m: m["map"]["floor"]["flatness_rms_m"], lambda v: v <= 0.01, "{:.3f} m"),
    ("ATE (GT)", lambda m: m["groundtruth"]["ate_rmse_m"], None, "{:.4f} m"),
    ("runtime", lambda m: m["run_info"]["runtime_s"], None, "{} s"),
]


def cell(m, get, test, fmt):
    try:
        v = get(m)
    except (KeyError, TypeError):
        return "n/a", None
    if v is None:
        return "n/a", None
    ok = None if test is None else bool(test(v))
    return fmt.format(v) + ("" if ok is None else (" ✓" if ok else " ✗")), ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs_root")
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--dataset", help="only runs on this dataset (folder name)")
    a = ap.parse_args()
    root = os.path.abspath(a.runs_root)
    names = a.runs or sorted(d for d in os.listdir(root) if os.path.isfile(os.path.join(root, d, "metrics.json")))
    runs = []
    for n in names:
        m = json.load(open(os.path.join(root, n, "metrics.json")))
        mf = os.path.join(root, n, "map_frame.json")
        if os.path.isfile(mf):
            m["map"] = json.load(open(mf))
        if a.dataset and os.path.basename(m["dataset"].rstrip("/")) != a.dataset:
            continue
        runs.append((n, m))
    if not runs:
        sys.exit("no evaluated runs found")

    lines = ["# Run comparison", "", "Targets: PLAN.md ('clean'). ✓ meets target, ✗ misses it.", ""]
    lines.append("| metric | " + " | ".join(n for n, _ in runs) + " |")
    lines.append("|---|" + "---|" * len(runs))
    for label, get, test, fmt in CHECKS:
        lines.append(f"| {label} | " + " | ".join(cell(m, get, test, fmt)[0] for _, m in runs) + " |")
    score = []
    for n, m in runs:
        oks = [cell(m, g, t, f)[1] for _, g, t, f in CHECKS]
        score.append(f"{n}: {sum(o is True for o in oks)}/{sum(o is not None for o in oks)} targets met")
    lines += ["", *score, ""]

    # pairwise agreement (same dataset only)
    trajs = {}
    for n, m in runs:
        st, T = read_tum(os.path.join(root, n, "trajectory.tum"))
        trajs[n] = (os.path.basename(m["dataset"].rstrip("/")), dict(zip(np.round(st, 4), T[:, :3, 3])))
    if len(runs) > 1:
        lines += ["## Agreement between runs (translation RMSE after SE(3) alignment, common frames)", "",
                  "| | " + " | ".join(n for n, _ in runs) + " |", "|---|" + "---|" * len(runs)]
        for n1, _ in runs:
            row = []
            for n2, _ in runs:
                d1, p1 = trajs[n1]
                d2, p2 = trajs[n2]
                common = sorted(set(p1) & set(p2))
                if n1 == n2 or d1 != d2 or len(common) < 10:
                    row.append("–")
                    continue
                A = np.array([p1[k] for k in common])
                B = np.array([p2[k] for k in common])
                T = umeyama(A, B)
                e = np.linalg.norm(A @ T[:3, :3].T + T[:3, 3] - B, axis=1)
                row.append(f"{np.sqrt((e ** 2).mean()):.3f} m")
            lines.append(f"| {n1} | " + " | ".join(row) + " |")
    out_md = os.path.join(root, "compare.md")
    open(out_md, "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))

    imgs = []
    for n, _ in runs:
        p = os.path.join(root, n, "views.png")
        if os.path.isfile(p):
            im = cv2.imread(p)
            im = cv2.resize(im, (1800, int(im.shape[0] * 1800 / im.shape[1])))
            cv2.putText(im, n, (10, im.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 200), 2)
            imgs.append(im)
    if imgs:
        cv2.imwrite(os.path.join(root, "compare.png"), np.vstack(imgs))
        print(f"\nimages: {os.path.join(root, 'compare.png')}")
    print(f"table: {out_md}")


if __name__ == "__main__":
    main()
