#!/usr/bin/env python3
"""Convert an engine's trajectory into the project format.

Input: a TUM-style file (stamp x y z qx qy qz qw [id]) whose body frame is the colour
camera OPTICAL frame (RTAB-Map --poses_camera, RTAB-Map rgbd_dataset *_poses.txt (format 1),
ORB-SLAM3 CameraTrajectory.txt, Open3D via its own converter). Any world frame is fine.
Output: RUN_DIR/trajectory.tum - one pose per dataset frame that has one, first posed frame at
the identity. The evaluator's gyro check confirms the body-frame convention.

Usage: python eval/convert_traj.py DATASET ENGINE_TRAJ RUN_DIR [--tol-ms 2]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mv2lib.dataset import Dataset  # noqa: E402
from mv2lib.traj import anchor_first, match_to_frames, read_tum, valid_mask, write_tum  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("engine_traj")
    ap.add_argument("run_dir")
    ap.add_argument("--tol-ms", type=float, default=2.0)
    a = ap.parse_args()
    ds = Dataset(a.dataset)
    st, T = read_tum(a.engine_traj)
    if len(st) == 0:
        sys.exit(f"ERROR: no poses in {a.engine_traj}")
    poses, n = match_to_frames(ds.stamps, st, T, tol_s=a.tol_ms / 1000.0)
    if n == 0:
        sys.exit("ERROR: no trajectory stamp matches a dataset frame (wrong dataset or time base?)")
    poses = anchor_first(poses)
    m = valid_mask(poses)
    os.makedirs(a.run_dir, exist_ok=True)
    out = os.path.join(a.run_dir, "trajectory.tum")
    write_tum(out, ds.stamps[m], poses[m],
              header="timestamp tx ty tz qx qy qz qw  (colour optical frame, first posed frame = identity)")
    print(f"{n}/{len(ds)} dataset frames have a pose ({len(st)} poses in file) -> {out}")
    if n < len(st):
        print(f"WARNING: {len(st) - n} poses in the file matched no dataset frame")


if __name__ == "__main__":
    main()
