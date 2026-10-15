#!/usr/bin/env python3
"""Find revisits in a dataset and measure their relative poses directly (once per dataset).

Engine-independent yardstick for drift and bad loop closures:
- frames sampled at 2 Hz; ORB features with valid depth
- every pair of samples at least --min-gap-s apart is matched (appearance), so a wrong
  trajectory cannot hide a revisit by never placing the two views together
- geometric verification: PnP-RANSAC (3D from depth of frame i, 2D in frame j), then
  colour-assisted ICP (Open3D colored ICP, coarse to fine) on the two RGB-D clouds; strict acceptance
  on inliers, overlap, residual and geometry (overlaps that are a single plane or crease are rejected)
  (on a synthetic walk with exact ground truth, accepted pairs were within 5 mm / 0.05 deg)
- the first and last seconds of the walk are always tried against each other (start-end check)

Output: JSON with T_i_from_j per accepted pair, used by eval/evaluate.py (--pairs, or found
automatically in RUNS_ROOT/_cache/<dataset name>/revisit_pairs.json).

Usage: python eval/revisit_pairs.py DATASET OUT_JSON [--rate-hz 2] [--min-gap-s 20] [--max-pairs 400]
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mv2lib.dataset import Dataset  # noqa: E402
from mv2lib.traj import rot_angle_deg  # noqa: E402

Z_MIN, Z_MAX = 0.3, 4.0
N_FEAT = 1000
MIN_MATCHES, MIN_INLIERS, MIN_INLIER_RATIO = 40, 30, 0.3
ICP_FITNESS, ICP_RMSE = 0.3, 0.012          # at 1.5 cm on 1 cm clouds
MIN_NORMAL_SPREAD = 0.05                    # reject overlaps that are one plane or a crease
MAX_ICP_CHANGE_T, MAX_ICP_CHANGE_R = 0.10, 5.0
TRIES_PER_CELL = 3                          # appearance hits verified per (bin_i, bin_j) cell at most


def features(ds, i, orb):
    img = cv2.cvtColor(ds.colour(i), cv2.COLOR_BGR2GRAY)
    z = ds.depth_m(i)
    kps, des = orb.detectAndCompute(img, None)
    if des is None:
        return None
    uv = np.array([k.pt for k in kps])
    zi = z[np.clip(uv[:, 1].round().astype(int), 0, z.shape[0] - 1), np.clip(uv[:, 0].round().astype(int), 0, z.shape[1] - 1)]
    ok = (zi > Z_MIN) & (zi < Z_MAX)
    K = ds.K
    xyz = np.c_[(uv[:, 0] - K[0, 2]) / K[0, 0] * zi, (uv[:, 1] - K[1, 2]) / K[1, 1] * zi, zi]
    return {"uv": uv.astype(np.float32), "xyz": xyz.astype(np.float32), "has_z": ok, "des": des}


def cloud(ds, i, voxel=0.01):
    """Coloured point cloud of frame i (camera frame), with normals."""
    z = ds.depth_m(i)
    z[(z < Z_MIN) | (z > Z_MAX)] = 0
    rgb = cv2.cvtColor(ds.colour(i), cv2.COLOR_BGR2RGB)
    intr = o3d.camera.PinholeCameraIntrinsic(ds.width, ds.height, ds.K[0, 0], ds.K[1, 1], ds.K[0, 2], ds.K[1, 2])
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(np.ascontiguousarray(rgb)), o3d.geometry.Image(np.ascontiguousarray(z)),
        depth_scale=1.0, depth_trunc=Z_MAX, convert_rgb_to_intensity=False)
    pc = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intr).voxel_down_sample(voxel)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=4 * voxel, max_nn=30))
    return pc


def normal_spread(src, tgt, T, dist=0.03):
    """Smallest/largest eigenvalue of sum(n n^T) over overlapping target normals: ~0 when the overlap
    is one plane or two planes (translation along them is unconstrained by geometry)."""
    s = o3d.geometry.PointCloud(src).transform(T)
    d = np.asarray(s.compute_point_cloud_distance(tgt))
    if (d < dist).sum() < 100:
        return 0.0
    tree = o3d.geometry.KDTreeFlann(tgt)
    n_t = np.asarray(tgt.normals)
    pts = np.asarray(s.points)[d < dist]
    idx = [tree.search_knn_vector_3d(p, 1)[1][0] for p in pts[:: max(1, len(pts) // 3000)]]
    N = n_t[idx]
    ev = np.linalg.eigvalsh(N.T @ N)
    return float(ev[0] / ev[-1])


def verify(ds, fi, fj, i, j, matcher, clouds):
    """Relative pose T_i_from_j (maps points in camera j to camera i) or None."""
    if fi is None or fj is None:
        return None
    knn = matcher.knnMatch(fi["des"], fj["des"], k=2)
    good = [m for m, *rest in knn if rest and m.distance < 0.8 * rest[0].distance and fi["has_z"][m.queryIdx]]
    if len(good) < MIN_MATCHES:
        return None
    obj = fi["xyz"][[m.queryIdx for m in good]]
    img = fj["uv"][[m.trainIdx for m in good]]
    ok, rvec, tvec, inl = cv2.solvePnPRansac(obj, img, ds.K, None, iterationsCount=1000, reprojectionError=3.0,
                                             confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    if not ok or inl is None or len(inl) < MIN_INLIERS or len(inl) / len(good) < MIN_INLIER_RATIO:
        return None
    inl = inl.ravel()
    rvec, tvec = cv2.solvePnPRefineLM(obj[inl], img[inl], ds.K, None, rvec, tvec)
    T_j_from_i = np.eye(4)
    T_j_from_i[:3, :3] = cv2.Rodrigues(rvec)[0]
    T_j_from_i[:3, 3] = tvec.ravel()
    T0 = np.linalg.inv(T_j_from_i)                      # i_from_j
    for k in (i, j):
        if k not in clouds:
            clouds[k] = cloud(ds, k)
    ci, cj = clouds[i], clouds[j]
    T = T0
    try:
        for vox, it in ((0.04, 50), (0.02, 30), (0.01, 14)):      # coarse to fine, colour + geometry
            r = o3d.pipelines.registration.registration_colored_icp(
                cj.voxel_down_sample(vox), ci.voxel_down_sample(vox), 1.5 * vox, T,
                o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(1e-6, 1e-6, it))
            T = r.transformation
    except RuntimeError:
        return None
    fit = o3d.pipelines.registration.evaluate_registration(cj, ci, 0.015, T)
    D = np.linalg.inv(T0) @ T
    spread = normal_spread(cj, ci, T)
    if (fit.fitness < ICP_FITNESS or fit.inlier_rmse > ICP_RMSE or np.linalg.norm(D[:3, 3]) > MAX_ICP_CHANGE_T
            or rot_angle_deg(D[:3, :3]) > MAX_ICP_CHANGE_R or spread < MIN_NORMAL_SPREAD):
        return None
    return {"T_i_from_j": T.round(9).tolist(), "pnp_inliers": int(len(inl)), "matches": len(good),
            "icp_fitness": float(fit.fitness), "icp_rmse_m": float(fit.inlier_rmse),
            "icp_change_m": float(np.linalg.norm(D[:3, 3])), "normal_spread": spread}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("out")
    ap.add_argument("--rate-hz", type=float, default=2.0)
    ap.add_argument("--min-gap-s", type=float, default=20.0)
    ap.add_argument("--max-pairs", type=int, default=400)
    ap.add_argument("--bin-s", type=float, default=5.0, help="keep the best pair per (bin_i, bin_j) cell")
    a = ap.parse_args()
    t_start = time.time()
    cv2.setRNGSeed(0)
    ds = Dataset(a.dataset)
    fps = 1.0 / np.median(np.diff(ds.stamps))
    step = max(1, int(round(fps / a.rate_hz)))
    samples = list(range(0, len(ds), step))
    # always include the very first and very last frames (start-end check)
    if samples[-1] != len(ds) - 1:
        samples.append(len(ds) - 1)
    orb = cv2.ORB_create(nfeatures=N_FEAT)
    print(f"{len(samples)} sampled frames, extracting features...", flush=True)
    feats = {i: features(ds, i, orb) for i in samples}
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    t = ds.stamps
    cand = [(i, j) for ai, i in enumerate(samples) for j in samples[ai + 1:] if t[j] - t[i] >= a.min_gap_s]
    print(f"{len(cand)} candidate pairs (gap >= {a.min_gap_s} s); matching...", flush=True)
    # quick appearance score for all candidates, verify the best per time-bin cell first
    scored = []
    for n, (i, j) in enumerate(cand):
        fi, fj = feats[i], feats[j]
        if fi is None or fj is None:
            continue
        knn = matcher.knnMatch(fi["des"], fj["des"], k=2)
        g = sum(1 for m, *rest in knn if rest and m.distance < 0.8 * rest[0].distance)
        if g >= MIN_MATCHES:
            scored.append((g, i, j))
        if n % 20000 == 0 and n:
            print(f"  {n}/{len(cand)}", flush=True)
    scored.sort(reverse=True)
    T_end = t[-1]
    start_end = [(g, i, j) for g, i, j in scored if t[i] - t[0] <= 3.0 and T_end - t[j] <= 3.0]
    cells, pairs, clouds, tried, tries = set(), [], {}, 0, {}
    print(f"{len(scored)} appearance hits; verifying (at most {TRIES_PER_CELL} per {a.bin_s:g} s x {a.bin_s:g} s cell)...",
          flush=True)
    for g, i, j in start_end[:20] + scored:
        cell = (int((t[i] - t[0]) // a.bin_s), int((t[j] - t[0]) // a.bin_s))
        se = t[i] - t[0] <= 3.0 and T_end - t[j] <= 3.0
        if cell in cells or len(pairs) >= a.max_pairs or (tries.get(cell, 0) >= TRIES_PER_CELL and not se):
            continue
        tries[cell] = tries.get(cell, 0) + 1
        tried += 1
        if tried % 200 == 0:
            print(f"  verified {tried} candidates, {len(pairs)} pairs accepted", flush=True)
        r = verify(ds, feats[i], feats[j], i, j, matcher, clouds)
        if r is None:
            continue
        cells.add(cell)
        r.update({"i": i, "j": j, "name_i": ds.names[i], "name_j": ds.names[j], "gap_s": float(t[j] - t[i]),
                  "start_end": bool(t[i] - t[0] <= 3.0 and T_end - t[j] <= 3.0)})
        pairs.append(r)
        if len(clouds) > 400:
            clouds.clear()
    out = {"dataset": ds.root, "rate_hz": a.rate_hz, "min_gap_s": a.min_gap_s, "samples": len(samples),
           "candidates": len(cand), "appearance_hits": len(scored), "verified_tried": tried,
           "pairs": sorted(pairs, key=lambda p: (p["i"], p["j"])),
           "seconds": round(time.time() - t_start, 1)}
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w") as fh:
        json.dump(out, fh, indent=1)
    n_se = sum(p["start_end"] for p in pairs)
    print(f"{len(pairs)} verified revisit pairs ({n_se} start-end) from {len(scored)} appearance hits, "
          f"{out['seconds']} s -> {a.out}")


if __name__ == "__main__":
    main()
