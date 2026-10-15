#!/usr/bin/env python3
"""Evaluator regression test on a synthetic dataset (tests/make_synth_rgbd.py output).

The ground-truth trajectory must score clean; deliberately broken versions must be caught.
Usage: python tests/test_evaluator.py SYNTH_DATASET
"""
import json
import os
import subprocess
import sys
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from mv2lib.traj import read_tum, write_tum  # noqa: E402

ds = os.path.abspath(sys.argv[1])
st, T = read_tum(os.path.join(ds, "groundtruth.txt"))
work = tempfile.mkdtemp(prefix="mv2_evaltest_")
pairs = os.path.join(work, "revisit_pairs.json")
subprocess.run([sys.executable, os.path.join(HERE, "..", "eval", "revisit_pairs.py"), ds, pairs, "--min-gap-s", "10"],
               check=True, capture_output=True)
O = np.eye(4)
O[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]          # RTAB-Map base_from_optical


def run(name, stamps, poses):
    src = os.path.join(work, name + ".txt")
    write_tum(src, stamps, poses)
    rd = os.path.join(work, name)
    subprocess.run([sys.executable, os.path.join(HERE, "..", "eval", "convert_traj.py"), ds, src, rd],
                   check=True, capture_output=True)
    subprocess.run([sys.executable, os.path.join(HERE, "..", "eval", "evaluate.py"), ds, rd, "--pairs", pairs],
                   check=True, capture_output=True)
    return json.load(open(os.path.join(rd, "metrics.json")))


fail = 0


def expect(label, cond):
    global fail
    print(("TEST PASS: " if cond else "TEST FAIL: ") + label)
    fail |= not cond


m = run("gt", st, T)
expect("ground truth: 100 % posed, 1 piece", m["coverage"]["fraction"] == 1.0 and m["coverage"]["pieces"] == 1)
expect("ground truth: no speed violations, no outlier frames", m["speed"]["frames_over_limit"] == 0 and not m["outlier_frames"])
expect("ground truth: gyro windows max < 1 deg", m["gyro"]["err_max_deg"] < 1.0)
expect("ground truth: convention OK, bias plausible", m["gyro"]["convention_ok"] and m["gyro"]["bias_plausible"])
expect("ground truth: gravity error < 1 deg", m["groundtruth"]["gravity_error_deg"] < 1.0)
expect("ground truth: ATE < 1 mm", m["groundtruth"]["ate_rmse_m"] < 0.001)
expect("ground truth: revisits found, median < 1 cm, incl. start-end",
       m["revisits"]["pairs_evaluated"] >= 3 and m["revisits"]["trans_median_m"] < 0.01 and m["revisits"]["start_end"])
expect("ground truth: sharpness ratio < 1.1", m["sharpness"]["ratio"] < 1.1)

# jump: everything after frame 400 shifted 0.3 m and rotated 10 deg
J = T.copy()
D = np.eye(4)
D[:3, :3] = Rot.from_euler("z", 10, degrees=True).as_matrix()
D[:3, 3] = [0.3, 0, 0]
J[400:] = D @ J[400:]
m = run("jump", st, J)
expect("jump: speed violation detected", m["speed"]["frames_over_limit"] > 0)
expect("jump: gyro window error > 5 deg", m["gyro"]["err_max_deg"] > 5.0)
expect("jump: ATE > 5 cm", m["groundtruth"]["ate_rmse_m"] > 0.05)

# single-frame spike: frame 500 alone is 20 cm / 5 deg off (out and back; the windows cancel it)
Sp = T.copy()
D = np.eye(4)
D[:3, :3] = Rot.from_euler("x", 5, degrees=True).as_matrix()
D[:3, 3] = [0.2, 0, 0]
Sp[500] = T[500] @ D
m = run("spike", st, Sp)
expect("spike: frames 500/501 flagged as outliers", {500, 501} <= set(m["outlier_frames"]))

# slow yaw drift of 2 deg/s about world z (smooth: speed stays plausible, gyro disagrees)
Dr = T.copy()
for i in range(len(T)):
    Rz = np.eye(4)
    Rz[:3, :3] = Rot.from_euler("z", 2.0 * (st[i] - st[0]), degrees=True).as_matrix()
    Dr[i] = Rz @ T[i]
m = run("drift", st, Dr)
expect("drift: no speed violation (smooth)", m["speed"]["frames_over_limit"] == 0)
expect("drift: flagged as implausible gyro bias", not m["gyro"]["bias_plausible"])
expect("drift: revisits p90 > 10 cm", m["revisits"]["trans_p90_m"] > 0.10)

# jitter: independent 1.5 cm / 0.3 deg noise on every pose (smeared, slightly doubled surfaces)
rng = np.random.default_rng(0)
Ji = T.copy()
for i in range(len(T)):
    N = np.eye(4)
    N[:3, :3] = Rot.from_rotvec(np.radians(0.3) * rng.normal(size=3) / np.sqrt(3)).as_matrix()
    N[:3, 3] = 0.015 * rng.normal(size=3) / np.sqrt(3)
    Ji[i] = T[i] @ N
m = run("jitter", st, Ji)
expect("jitter: sharpness ratio > 1.3", m["sharpness"]["ratio"] > 1.3)

# missing stretch: frames 300-349 without pose
keep = np.ones(len(st), bool)
keep[300:350] = False
m = run("gap", st[keep], T[keep])
expect("gap: 2 pieces, gap of 50 frames", m["coverage"]["pieces"] == 2 and m["coverage"]["longest_gap_frames"] == 50)

# wrong body frame: poses of RTAB-Map's base frame instead of the optical frame
B = np.array([Ti @ np.linalg.inv(O) for Ti in T])
m = run("baseframe", st, B)
expect("wrong body frame: convention MISMATCH flagged", not m["gyro"]["convention_ok"])

print("ALL TESTS PASSED" if not fail else "SOME TESTS FAILED", f"(work dir {work})")
sys.exit(fail)
