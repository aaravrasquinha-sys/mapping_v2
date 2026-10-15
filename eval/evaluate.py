#!/usr/bin/env python3
"""Score one trajectory against its dataset, without ground truth.

Input: a dataset folder and a run folder holding trajectory.tum (camera optical frame,
made by eval/convert_traj.py). Output: RUN/metrics.json, one row appended to
RUNS_ROOT/results.csv, and a console summary.

Metrics (targets are in PLAN.md):
  coverage    frames with a pose, longest gap, number of separate pieces
  speed       translation speed over 0.2 s windows (jumps show up as impossible speeds)
  gyro        rotation over 0.5 s windows vs the integrated gyro (IMU time offset and gyro
              bias are estimated first); also detects a wrong body-frame convention
  gravity     'up' from the accelerometer, rotated into the map with the trajectory and
              averaged over the whole walk (needs neither a still start nor full-rate IMU)
  revisits    trajectory vs relative poses measured directly between revisits (eval/revisit_pairs.py);
              the start-end pair is reported separately
  sharpness   surface thickness of the multi-frame cloud / of single frames (doubled or smeared walls)
  groundtruth only if DATASET/groundtruth.txt exists (synthetic tests): ATE after SE(3) alignment

Usage: python eval/evaluate.py DATASET RUN_DIR [--results CSV]
"""
import argparse
import csv
import datetime
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mv2lib.dataset import Dataset  # noqa: E402
from mv2lib.traj import (anchor_first, interp_rotations, match_to_frames, read_tum,  # noqa: E402
                         rot_angle_deg, umeyama, valid_mask)

SPEED_LIMIT = 1.5          # m/s, brisk walk
WIN_SPEED_S = 0.2
WIN_GYRO_S = 0.5
BIAS_LIMIT_DPS = 1.0      # D435i gyro zero-rate offset is typically well below this


def load_run_traj(ds, run_dir):
    p = os.path.join(run_dir, "trajectory.tum")
    if not os.path.isfile(p):
        sys.exit(f"ERROR: {p} not found (run eval/convert_traj.py first)")
    st, T = read_tum(p)
    poses, n = match_to_frames(ds.stamps, st, T)
    return anchor_first(poses), n, len(st)


# ----------------------------------------------------------------------------- metrics

def coverage(poses):
    m = valid_mask(poses)
    pieces, gap, longest_gap = 0, 0, 0
    prev = False
    for v in m:
        if v and not prev:
            pieces += 1
        gap = 0 if v else gap + 1
        longest_gap = max(longest_gap, gap)
        prev = v
    return {"frames": int(len(m)), "posed": int(m.sum()), "fraction": float(m.mean()) if len(m) else 0.0,
            "pieces": pieces, "longest_gap_frames": int(longest_gap),
            "first_posed": int(np.argmax(m)) if m.any() else None}


def speed(stamps, poses):
    fps = 1.0 / np.median(np.diff(stamps))
    m = max(1, int(round(WIN_SPEED_S * fps / 2)))
    v, w = [], []
    for k in range(m, len(poses) - m):
        a, b = poses[k - m], poses[k + m]
        if not (np.all(np.isfinite(a)) and np.all(np.isfinite(b))):
            continue
        dt = stamps[k + m] - stamps[k - m]
        v.append((k, np.linalg.norm(b[:3, 3] - a[:3, 3]) / dt))
        w.append(rot_angle_deg(a[:3, :3].T @ b[:3, :3]) / dt)
    if not v:
        return {"error": "too few poses"}
    vs = np.array([x[1] for x in v])
    bad = [int(x[0]) for x in v if x[1] > SPEED_LIMIT]
    return {"window_s": WIN_SPEED_S, "max_mps": float(vs.max()), "p999_mps": float(np.percentile(vs, 99.9)),
            "median_mps": float(np.median(vs)), "frames_over_limit": len(bad), "limit_mps": SPEED_LIMIT,
            "first_bad_frames": bad[:20], "max_rot_dps": float(max(w))}


def traj_rates(stamps, poses):
    """Angular velocity (camera frame, rad/s) between consecutive posed frames."""
    out = []
    for k in range(len(poses) - 1):
        a, b = poses[k], poses[k + 1]
        if np.all(np.isfinite(a)) and np.all(np.isfinite(b)):
            dt = stamps[k + 1] - stamps[k]
            out.append((0.5 * (stamps[k] + stamps[k + 1]), Rot.from_matrix(a[:3, :3].T @ b[:3, :3]).as_rotvec() / dt,
                        stamps[k], stamps[k + 1]))
    return out


def gyro_check(ds, poses, R_ci):
    tg, wg = ds.imu("gyro")
    if tg is None or len(tg) < 10:
        return {"error": "no gyro data"}
    wc = wg @ R_ci.T                           # gyro in camera axes
    rates = traj_rates(ds.stamps, poses)
    if len(rates) < 50:
        return {"error": "too few consecutive poses"}
    # cumulative integral of gyro (vector) -> mean rate over any interval
    dtg = np.diff(tg, prepend=tg[0])
    C = np.cumsum(wc * dtg[:, None], axis=0)

    def mean_rate(t0, t1, lag):
        c0 = np.array([np.interp(t0 - lag, tg, C[:, i]) for i in range(3)])
        c1 = np.array([np.interp(t1 - lag, tg, C[:, i]) for i in range(3)])
        return (c1 - c0) / (t1 - t0)

    sub = rates[:: max(1, len(rates) // 3000)]
    traj_mag = np.array([np.linalg.norm(r[1]) for r in sub])
    best = (None, -2)
    for lag in np.arange(-0.2, 0.2001, 0.002):        # camera time = gyro time + lag
        g_mag = np.array([np.linalg.norm(mean_rate(r[2], r[3], lag)) for r in sub])
        if g_mag.std() < 1e-9 or traj_mag.std() < 1e-9:
            continue
        c = np.corrcoef(traj_mag, g_mag)[0, 1]
        if c > best[1]:
            best = (lag, c)
    lag = best[0] if best[0] is not None else 0.0
    # gyro bias (camera axes): median difference over all frame intervals
    diffs = np.array([mean_rate(r[2], r[3], lag) - r[1] for r in rates])
    bias = np.median(diffs, axis=0)
    # rotation over windows: trajectory vs integrated gyro
    n = max(1, int(round(WIN_GYRO_S / np.median(np.diff(ds.stamps)))))
    errs = []
    order = np.argsort(tg)
    tg_s, wc_s = tg[order] + lag, wc[order] - bias
    for k in range(0, len(poses) - n, max(1, n // 3)):
        a, b = poses[k], poses[k + n]
        if not (np.all(np.isfinite(a)) and np.all(np.isfinite(b))):
            continue
        t0, t1 = ds.stamps[k], ds.stamps[k + n]
        i0, i1 = np.searchsorted(tg_s, t0), np.searchsorted(tg_s, t1)
        if i1 - i0 < 2:
            continue
        R = np.eye(3)
        ts = np.concatenate([[t0], tg_s[i0:i1], [t1]])
        ws = np.concatenate([wc_s[i0:i0 + 1], wc_s[i0:i1]])   # rate held over each sub-interval
        for j in range(len(ts) - 1):
            R = R @ Rot.from_rotvec(ws[j] * (ts[j + 1] - ts[j])).as_matrix()
        errs.append((k, rot_angle_deg(R.T @ (a[:3, :3].T @ b[:3, :3]))))
    if not errs:
        return {"error": "no complete windows"}
    e = np.array([x[1] for x in errs])
    # convention check: does a different body frame fit the gyro much better?
    alt = {}
    # if the engine wrote RTAB-Map's base frame (x fwd, y left, z up), the optical-frame rate is O^T r
    O = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)    # base_from_optical
    for name, M in (("rtabmap_base_instead_of_optical", O.T),):
        da = np.array([np.linalg.norm(mean_rate(r[2], r[3], lag) - bias - M @ r[1]) for r in rates[::10]])
        alt[name] = float(np.median(da))
    d0 = np.array([np.linalg.norm(mean_rate(r[2], r[3], lag) - bias - r[1]) for r in rates[::10]])
    convention_ok = all(float(np.median(d0)) <= v for v in alt.values())
    return {"window_s": WIN_GYRO_S, "time_offset_ms": round(1000 * lag, 1), "offset_correlation": round(best[1], 3),
            "gyro_bias_dps": np.degrees(bias).round(3).tolist(),
            "bias_plausible": bool(np.all(np.abs(np.degrees(bias)) <= BIAS_LIMIT_DPS)), "windows": len(e),
            "err_p50_deg": float(np.median(e)), "err_p99_deg": float(np.percentile(e, 99)),
            "err_max_deg": float(e.max()), "frac_le_1deg": float((e <= 1.0).mean()),
            "worst_windows": [{"frame": int(k), "deg": round(v, 2)} for k, v in sorted(errs, key=lambda x: -x[1])[:10]],
            "rate_residual_median_dps": float(np.degrees(np.median(d0))),
            "convention_ok": bool(convention_ok), "convention_alternatives_dps": {k: float(np.degrees(v)) for k, v in alt.items()}}


def gravity(ds, poses, R_ci, lag_s):
    ta, fa = ds.imu("accel")
    if ta is None or len(ta) < 10:
        return {"error": "no accel data"}
    R_wc = interp_rotations(ta + lag_s, ds.stamps, poses)
    ok = np.all(np.isfinite(R_wc.reshape(len(ta), -1)), axis=1)
    if ok.sum() < 10:
        return {"error": "no overlap between accel and trajectory"}
    fc = fa[ok] @ R_ci.T
    fw = np.einsum("nij,nj->ni", R_wc[ok], fc)
    mean = fw.mean(0)
    up = mean / np.linalg.norm(mean)
    # rotation taking 'up' to +z, then yaw so the first camera looks along +x
    z = np.array([0, 0, 1.0])
    v = np.cross(up, z)
    s, c = np.linalg.norm(v), up @ z
    if s < 1e-12:
        R_tilt = np.eye(3)
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R_tilt = np.eye(3) + vx + vx @ vx * ((1 - c) / s ** 2)
    fwd = R_tilt @ np.array([0, 0, 1.0])       # first camera optical axis (first pose = identity)
    yaw = np.arctan2(fwd[1], fwd[0])
    R_align = Rot.from_euler("z", -yaw).as_matrix() @ R_tilt
    # spread: how much 'up' changes between first and second half (consistency)
    half = len(fw) // 2
    u1, u2 = fw[:half].mean(0), fw[half:].mean(0)
    return {"samples": int(ok.sum()), "up_in_traj_frame": up.round(6).tolist(),
            "specific_force_mean_mps2": float(np.linalg.norm(mean)),
            "halves_disagree_deg": float(np.degrees(np.arccos(np.clip(u1 @ u2 / np.linalg.norm(u1) / np.linalg.norm(u2), -1, 1)))),
            "R_map_from_traj": R_align.round(9).tolist()}


def revisits(ds, poses, pairs_path):
    """Trajectory vs directly measured revisit poses (eval/revisit_pairs.py)."""
    if not pairs_path or not os.path.isfile(pairs_path):
        return {"error": "no revisit pairs (run eval/revisit_pairs.py DATASET RUNS_ROOT/_cache/<dataset>/revisit_pairs.json)"}
    P = json.load(open(pairs_path))
    if os.path.basename(P.get("dataset", "").rstrip("/")) != ds.name:
        return {"error": f"pairs file is for another dataset: {P.get('dataset')}"}
    rows = []
    for p in P["pairs"]:
        i, j = p["i"], p["j"]
        if not (np.all(np.isfinite(poses[i])) and np.all(np.isfinite(poses[j]))):
            rows.append((p, None, None))
            continue
        D = np.linalg.inv(np.array(p["T_i_from_j"])) @ (np.linalg.inv(poses[i]) @ poses[j])
        rows.append((p, float(np.linalg.norm(D[:3, 3])), rot_angle_deg(D[:3, :3])))
    ev = [(p, t, r) for p, t, r in rows if t is not None]
    if not ev:
        return {"error": "no pair has both frames posed", "pairs_total": len(rows)}
    tt = np.array([t for _, t, _ in ev])
    rr = np.array([r for _, _, r in ev])
    se = [(p, t, r) for p, t, r in ev if p.get("start_end")]
    worst = sorted(ev, key=lambda x: -x[1])[:10]
    return {"pairs_total": len(rows), "pairs_evaluated": len(ev),
            "trans_median_m": float(np.median(tt)), "trans_p90_m": float(np.percentile(tt, 90)), "trans_max_m": float(tt.max()),
            "rot_median_deg": float(np.median(rr)), "rot_p90_deg": float(np.percentile(rr, 90)), "rot_max_deg": float(rr.max()),
            "start_end": ({"trans_m": min(t for _, t, _ in se), "rot_deg": min(r for _, _, r in se), "pairs": len(se)} if se else None),
            "worst": [{"i": p["i"], "j": p["j"], "gap_s": round(p["gap_s"], 1), "trans_m": round(t, 4), "rot_deg": round(r, 3)}
                      for p, t, r in worst]}


def sharpness(ds, poses, n_frames=120, radius=0.05, seed=0):
    """Surface thickness in the multi-frame cloud / in single frames (median local plane spread).
    1.0 = the trajectory adds no inconsistency; doubled or smeared surfaces (< ~radius apart) raise it."""
    from scipy.spatial import cKDTree
    from fuse import edge_filter
    rng = np.random.default_rng(seed)
    k = ds.depth_noise_model()
    zmax = float(np.clip(np.sqrt(0.04 / k), 1.5, 6.0))
    idx = np.where(valid_mask(poses))[0]
    if len(idx) < 10:
        return {"error": "too few poses"}
    idx = idx[np.linspace(0, len(idx) - 1, min(n_frames, len(idx))).round().astype(int)]
    v, u = np.mgrid[0:ds.height:2, 0:ds.width:2]
    Kinv = np.linalg.inv(ds.K)

    def frame_points(i, n):
        z = edge_filter(ds.depth_m(i), k)[::2, ::2]
        ok = (z > 0.3) & (z < zmax)
        pix = np.stack([u[ok], v[ok], np.ones(ok.sum())], 1) * z[ok][:, None]
        pc = pix @ Kinv.T
        if n and len(pc) > n:
            pc = pc[rng.choice(len(pc), n, replace=False)]
        return pc

    def spread(pts, centres, tree):
        out = []
        for c in centres:
            nb = tree.query_ball_point(c, radius)
            if len(nb) < 20:
                continue
            q = pts[nb] - pts[nb].mean(0)
            ev = np.linalg.eigvalsh(q.T @ q / len(nb))
            if ev[1] < 0.25 * radius ** 2 / 4:        # not a surface patch (edge, thin object)
                continue
            out.append(np.sqrt(max(ev[0], 0)))
        return np.array(out)

    multi, single = [], []
    for n, i in enumerate(idx):
        p_cam = frame_points(i, 4000)
        multi.append(p_cam @ poses[i][:3, :3].T + poses[i][:3, 3])
        if n % 6 == 0:                                   # single-frame baseline on a subset of frames
            dense = frame_points(i, 0)
            if len(dense) > 100:
                tree = cKDTree(dense)
                single.append(spread(dense, dense[rng.choice(len(dense), 150)], tree))
    M = np.concatenate(multi)
    tree = cKDTree(M)
    sm = spread(M, M[rng.choice(len(M), 4000, replace=False)], tree)
    ss = np.concatenate(single) if single else np.array([])
    if len(sm) < 100 or len(ss) < 100:
        return {"error": "too few surface patches"}
    return {"radius_m": radius, "frames": int(len(idx)), "spread_multi_median_m": float(np.median(sm)),
            "spread_single_median_m": float(np.median(ss)), "ratio": float(np.median(sm) / np.median(ss)),
            "ratio_p90": float(np.percentile(sm, 90) / np.percentile(ss, 90))}


def groundtruth(ds, poses):
    p = ds.groundtruth_path()
    if not p:
        return None
    st, T = read_tum(p)
    gt, _ = match_to_frames(ds.stamps, st, T)
    m = valid_mask(poses) & valid_mask(gt)
    if m.sum() < 3:
        return {"error": "no overlap with ground truth"}
    A = umeyama(poses[m][:, :3, 3], gt[m][:, :3, 3])
    est = A[:3, :3] @ poses[m][:, :3, 3].T + A[:3, 3:4]
    e = np.linalg.norm(est.T - gt[m][:, :3, 3], axis=1)
    # relative error over 1 s
    fps = 1.0 / np.median(np.diff(ds.stamps))
    n = int(round(fps))
    rpe_t, rpe_r = [], []
    idx = np.where(m)[0]
    for k in idx:
        if k + n < len(poses) and m[k + n]:
            Re = np.linalg.inv(poses[k]) @ poses[k + n]
            Rg = np.linalg.inv(gt[k]) @ gt[k + n]
            D = np.linalg.inv(Rg) @ Re
            rpe_t.append(np.linalg.norm(D[:3, 3]))
            rpe_r.append(rot_angle_deg(D[:3, :3]))
    # gravity truth: world z of GT expressed in the anchored trajectory frame
    g0 = gt[np.argmax(m)]
    up_true = (g0[:3, :3].T @ np.array([0, 0, 1.0]))
    return {"ate_rmse_m": float(np.sqrt((e ** 2).mean())), "ate_max_m": float(e.max()),
            "rpe_1s_trans_rmse_m": float(np.sqrt(np.mean(np.square(rpe_t)))) if rpe_t else None,
            "rpe_1s_rot_rmse_deg": float(np.sqrt(np.mean(np.square(rpe_r)))) if rpe_r else None,
            "up_true_in_traj_frame": up_true.round(6).tolist()}


# ----------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("run_dir")
    ap.add_argument("--results", help="results CSV (default: <parent of RUN_DIR>/results.csv)")
    ap.add_argument("--pairs", help="revisit pairs JSON (default: <parent of RUN_DIR>/_cache/<dataset>/revisit_pairs.json)")
    ap.add_argument("--no-sharpness", action="store_true")
    a = ap.parse_args()
    ds = Dataset(a.dataset)
    run_dir = os.path.abspath(a.run_dir)
    poses, n_matched, n_file = load_run_traj(ds, run_dir)
    warnings = []
    if n_matched < n_file:
        warnings.append(f"{n_file - n_matched} trajectory poses did not match any dataset frame")

    R_ci = ds.T_colour_from_imu("gyro")
    if R_ci is None:
        warnings.append("no IMU->colour extrinsics in calib.yaml; assuming IMU axes = camera axes")
        R_ci = np.eye(3)
    else:
        R_ci = R_ci[:3, :3]

    M = {"dataset": ds.root, "run": run_dir, "evaluated_at": datetime.datetime.now().isoformat(timespec="seconds")}
    M["coverage"] = coverage(poses)
    M["speed"] = speed(ds.stamps, poses)
    M["gyro"] = gyro_check(ds, poses, R_ci)
    lag = M["gyro"].get("time_offset_ms", 0.0) / 1000.0 if "error" not in M["gyro"] else 0.0
    M["gravity"] = gravity(ds, poses, R_ci, lag)
    pairs = a.pairs or os.path.join(os.path.dirname(run_dir), "_cache", ds.name, "revisit_pairs.json")
    M["revisits"] = revisits(ds, poses, pairs)
    M["sharpness"] = {"skipped": True} if a.no_sharpness else sharpness(ds, poses)
    gt = groundtruth(ds, poses)
    if gt is not None:
        M["groundtruth"] = gt
        if "up_in_traj_frame" in M["gravity"] and "up_true_in_traj_frame" in gt:
            u, v = np.array(M["gravity"]["up_in_traj_frame"]), np.array(gt["up_true_in_traj_frame"])
            gt["gravity_error_deg"] = float(np.degrees(np.arccos(np.clip(u @ v, -1, 1))))
    run_json = os.path.join(run_dir, "run.json")
    M["run_info"] = json.load(open(run_json)) if os.path.isfile(run_json) else {}
    M["warnings"] = warnings
    with open(os.path.join(run_dir, "metrics.json"), "w") as fh:
        json.dump(M, fh, indent=2)

    c, s, g, gr = M["coverage"], M["speed"], M["gyro"], M["gravity"]
    row = {"run": os.path.basename(run_dir), "engine": M["run_info"].get("engine", ""),
           "variant": M["run_info"].get("variant", ""), "dataset": ds.name,
           "posed_frac": round(c["fraction"], 4), "pieces": c["pieces"],
           "max_speed_mps": round(s.get("max_mps", float("nan")), 3), "speed_violations": s.get("frames_over_limit", ""),
           "gyro_p99_deg": round(g.get("err_p99_deg", float("nan")), 3), "gyro_max_deg": round(g.get("err_max_deg", float("nan")), 3),
           "gyro_frac_le_1deg": round(g.get("frac_le_1deg", float("nan")), 4),
           "convention_ok": g.get("convention_ok", ""), "gyro_bias_ok": g.get("bias_plausible", ""),
           "revisit_med_m": round(M["revisits"].get("trans_median_m", float("nan")), 4),
           "revisit_p90_m": round(M["revisits"].get("trans_p90_m", float("nan")), 4),
           "revisit_med_deg": round(M["revisits"].get("rot_median_deg", float("nan")), 3),
           "revisit_p90_deg": round(M["revisits"].get("rot_p90_deg", float("nan")), 3),
           "start_end_m": round(M["revisits"]["start_end"]["trans_m"], 4) if M["revisits"].get("start_end") else "",
           "sharpness": round(M["sharpness"].get("ratio", float("nan")), 3),
           "ate_rmse_m": round(gt["ate_rmse_m"], 4) if gt and "ate_rmse_m" in gt else "",
           "runtime_s": M["run_info"].get("runtime_s", ""), "evaluated_at": M["evaluated_at"]}
    res = a.results or os.path.join(os.path.dirname(run_dir), "results.csv")
    new = not os.path.isfile(res)
    with open(res, "a", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(row.keys()))
        if new:
            w.writeheader()
        w.writerow(row)

    print("=" * 70)
    print(f"run: {run_dir}")
    print(f"coverage: {c['posed']}/{c['frames']} frames posed ({100 * c['fraction']:.1f} %), pieces {c['pieces']}, "
          f"longest gap {c['longest_gap_frames']} frames")
    if "error" not in s:
        print(f"speed: max {s['max_mps']:.2f} m/s, p99.9 {s['p999_mps']:.2f}, frames over {SPEED_LIMIT} m/s: "
              f"{s['frames_over_limit']}; max rotation rate {s['max_rot_dps']:.0f} deg/s")
    if "error" not in g:
        print(f"gyro: offset {g['time_offset_ms']} ms (corr {g['offset_correlation']}), bias {g['gyro_bias_dps']} deg/s; "
              f"0.5 s windows: p50 {g['err_p50_deg']:.2f}, p99 {g['err_p99_deg']:.2f}, max {g['err_max_deg']:.2f} deg, "
              f"<=1 deg {100 * g['frac_le_1deg']:.1f} %; convention {'OK' if g['convention_ok'] else 'MISMATCH'}")
        if not g["bias_plausible"]:
            print(f"WARNING: estimated gyro bias above {BIAS_LIMIT_DPS} deg/s: slow rotation drift in the "
                  "trajectory, or an uncalibrated gyro (revisit checks decide which)")
    else:
        print("gyro:", g["error"])
    if "error" not in gr:
        print(f"gravity: |f| {gr['specific_force_mean_mps2']:.3f} m/s^2, halves disagree {gr['halves_disagree_deg']:.2f} deg")
    rv, sh = M["revisits"], M["sharpness"]
    if "error" not in rv:
        se = rv["start_end"]
        print(f"revisits: {rv['pairs_evaluated']}/{rv['pairs_total']} pairs; translation median {100 * rv['trans_median_m']:.1f} cm, "
              f"p90 {100 * rv['trans_p90_m']:.1f} cm, max {100 * rv['trans_max_m']:.1f} cm; rotation median "
              f"{rv['rot_median_deg']:.2f}, p90 {rv['rot_p90_deg']:.2f} deg"
              + (f"; start-end {100 * se['trans_m']:.1f} cm / {se['rot_deg']:.2f} deg" if se else "; start-end: no overlapping views"))
    else:
        print("revisits:", rv["error"])
    if "ratio" in sh:
        print(f"sharpness: surface thickness multi/single = {sh['ratio']:.2f} (p90 {sh['ratio_p90']:.2f}); "
              f"{1000 * sh['spread_multi_median_m']:.1f} mm vs {1000 * sh['spread_single_median_m']:.1f} mm")
    elif not sh.get("skipped"):
        print("sharpness:", sh.get("error"))
    if gt:
        print("groundtruth:", {k: (round(v, 4) if isinstance(v, float) else v) for k, v in gt.items() if k != "up_true_in_traj_frame"})
    for wmsg in warnings:
        print("WARNING:", wmsg)
    print(f"metrics: {os.path.join(run_dir, 'metrics.json')}   results: {res}")


if __name__ == "__main__":
    main()
