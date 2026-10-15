#!/usr/bin/env python3
"""Convert an Open3D trajectory .log (5 lines per frame: ids, then the 4x4 world-from-camera pose)
into TUM, taking timestamps from the colour file names in WORK/rgb (same sorted order Open3D used).

Usage: python eval/open3d_log_to_tum.py WORK_DIR TRAJ_LOG OUT_TUM
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mv2lib.traj import write_tum  # noqa: E402


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    work, log, out = sys.argv[1:]
    names = sorted(f for f in os.listdir(os.path.join(work, "rgb")) if f.endswith(".png"))
    stamps = np.array([float(os.path.splitext(f)[0]) for f in names])
    lines = [ln for ln in open(log) if ln.strip()]
    if len(lines) % 5:
        sys.exit(f"ERROR: {log} is not a 5-lines-per-frame .log file")
    poses = []
    for k in range(0, len(lines), 5):
        poses.append(np.array([[float(x) for x in lines[k + r].split()] for r in range(1, 5)]))
    poses = np.array(poses)
    if len(poses) != len(stamps):
        print(f"WARNING: {len(poses)} poses for {len(stamps)} images; using the first {min(len(poses), len(stamps))}")
    n = min(len(poses), len(stamps))
    write_tum(out, stamps[:n], poses[:n], header="timestamp tx ty tz qx qy qz qw  (Open3D camera poses)")
    print(f"{n} poses -> {out}")


if __name__ == "__main__":
    main()
