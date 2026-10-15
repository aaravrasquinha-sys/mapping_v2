#!/usr/bin/env python3
"""Replay video of a finished run: camera image with features | map growing as it is seen.

Left: the colour frame with ORB features (green), time, frame number, and a red LOST banner when
the engine gave no pose for that frame. Right: a fixed oblique view of the final map in which each
surface point appears from the first frame that saw it, with the camera path so far (red) and the
current camera (yellow frustum). Made offline from the run's outputs; it never affects mapping.

Usage: python finish/render_video.py DATASET RUN_DIR [--out replay.mp4] [--step 1] [--azimuth 225]
                                     [--elevation 50] [--max-frames N]
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import open3d as o3d

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "eval"))
from mv2lib.dataset import Dataset  # noqa: E402
from mv2lib.traj import match_to_frames, read_tum  # noqa: E402
from render_views import view_matrix  # noqa: E402

W_PANEL, H_PANEL = 640, 480
CUT_Z = 2.0


def first_seen(points, ds, poses, every=10):
    """Index of the first frame (checked every `every` frames) in which each point is visible."""
    seen = np.full(len(points), np.iinfo(np.int32).max, np.int32)
    for i in range(0, len(ds), every):
        if not np.all(np.isfinite(poses[i])):
            continue
        Tcw = np.linalg.inv(poses[i])
        pc = points @ Tcw[:3, :3].T + Tcw[:3, 3]
        z = pc[:, 2]
        cand = np.where((z > 0.3) & (seen == np.iinfo(np.int32).max))[0]
        if not len(cand):
            continue
        uv = pc[cand, :2] / z[cand, None] * [ds.K[0, 0], ds.K[1, 1]] + [ds.K[0, 2], ds.K[1, 2]]
        u, v = np.round(uv[:, 0]).astype(int), np.round(uv[:, 1]).astype(int)
        inside = (u >= 0) & (u < ds.width) & (v >= 0) & (v < ds.height)
        cand, u, v = cand[inside], u[inside], v[inside]
        d = ds.depth_m(i)[v, u]
        vis = (d > 0) & (np.abs(d - z[cand]) < 0.05)
        seen[cand[vis]] = i
    return seen


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("run_dir")
    ap.add_argument("--out")
    ap.add_argument("--step", type=int, default=1, help="use every N-th frame (video plays N times faster)")
    ap.add_argument("--azimuth", type=float, default=225)
    ap.add_argument("--elevation", type=float, default=50)
    ap.add_argument("--max-frames", type=int)
    a = ap.parse_args()
    t_start = time.time()
    ds = Dataset(a.dataset)
    run = os.path.abspath(a.run_dir)
    st, T = read_tum(os.path.join(run, "trajectory_map.tum"))
    poses, _ = match_to_frames(ds.stamps, st, T)
    cloud = o3d.io.read_point_cloud(os.path.join(run, "map_cloud.ply"))
    pts = np.asarray(cloud.points)
    cols = (np.asarray(cloud.colors)[:, ::-1] * 255).astype(np.uint8)
    keep = pts[:, 2] < CUT_Z
    pts, cols = pts[keep], cols[keep]
    print(f"{len(pts)} map points; finding when each is first seen...", flush=True)
    seen = first_seen(pts, ds, poses)

    # fixed oblique orthographic view fitted to the whole map
    R = view_matrix(a.azimuth, a.elevation)
    p = pts @ R.T
    lo, hi = p[:, :2].min(0), p[:, :2].max(0)
    centre = 0.5 * (lo + hi)
    scale = min((W_PANEL - 40) / (hi[0] - lo[0]), (H_PANEL - 40) / (hi[1] - lo[1]))

    def to_px(xyz):
        q = np.atleast_2d(xyz) @ R.T
        return np.c_[(q[:, 0] - centre[0]) * scale + W_PANEL / 2, (q[:, 1] - centre[1]) * scale + H_PANEL / 2], q[:, 2]

    uv, depth = to_px(pts)
    uv = uv.astype(int)
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < W_PANEL) & (uv[:, 1] >= 0) & (uv[:, 1] < H_PANEL)
    order = np.argsort(seen[ok], kind="stable")
    uv, depth, cols_v, seen_v = uv[ok][order], depth[ok][order], cols[ok][order], seen[ok][order]
    canvas = np.full((H_PANEL, W_PANEL, 3), 255, np.uint8)
    zbuf = np.full((H_PANEL, W_PANEL), np.inf)
    ptr = 0

    out = a.out or os.path.join(run, "replay.mp4")
    fps = 1.0 / np.median(np.diff(ds.stamps)) / a.step
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (2 * W_PANEL, H_PANEL))
    if not vw.isOpened():
        sys.exit("ERROR: could not open the video writer (mp4v)")
    orb = cv2.ORB_create(nfeatures=500)
    frames = list(range(0, len(ds), a.step))[: a.max_frames]
    path = []
    fx, fy, cx, cy = ds.K[0, 0], ds.K[1, 1], ds.K[0, 2], ds.K[1, 2]
    corners = np.array([[(0 - cx) / fx, (0 - cy) / fy, 1], [(ds.width - cx) / fx, (0 - cy) / fy, 1],
                        [(ds.width - cx) / fx, (ds.height - cy) / fy, 1], [(0 - cx) / fx, (ds.height - cy) / fy, 1]]) * 0.35
    for n, i in enumerate(frames):
        # reveal points first seen up to this frame (z-buffered, near wins)
        end = np.searchsorted(seen_v, i, side="right")
        if end > ptr:
            u, v, d = uv[ptr:end, 0], uv[ptr:end, 1], depth[ptr:end]
            for du in (0, 1):
                for dv in (0, 1):
                    uu, vv = np.clip(u + du, 0, W_PANEL - 1), np.clip(v + dv, 0, H_PANEL - 1)
                    closer = d < zbuf[vv, uu]
                    zbuf[vv[closer], uu[closer]] = d[closer]
                    canvas[vv[closer], uu[closer]] = cols_v[ptr:end][closer]
            ptr = end
        right = canvas.copy()
        posed = np.all(np.isfinite(poses[i]))
        if posed:
            path.append(poses[i][:3, 3])
        if len(path) > 1:
            q, _ = to_px(np.array(path))
            cv2.polylines(right, [q.astype(np.int32).reshape(-1, 1, 2)], False, (0, 0, 220), 2)
        if posed:
            c = poses[i][:3, 3]
            wc = corners @ poses[i][:3, :3].T + c
            q, _ = to_px(np.vstack([c, wc]))
            q = q.astype(int)
            for k in range(4):
                cv2.line(right, tuple(q[0]), tuple(q[k + 1]), (0, 200, 255), 2)
                cv2.line(right, tuple(q[k + 1]), tuple(q[(k + 1) % 4 + 1]), (0, 200, 255), 2)
        left = ds.colour(i)
        kps = orb.detect(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY), None)
        for k in kps:
            cv2.circle(left, (int(k.pt[0]), int(k.pt[1])), 3, (0, 255, 0), 1)
        if left.shape[:2] != (H_PANEL, W_PANEL):
            left = cv2.resize(left, (W_PANEL, H_PANEL))
        t_rel = ds.stamps[i] - ds.stamps[0]
        cv2.putText(left, f"t {t_rel:6.1f} s  frame {i}  features {len(kps)}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if not posed:
            cv2.rectangle(left, (0, H_PANEL - 50), (W_PANEL, H_PANEL), (0, 0, 200), -1)
            cv2.putText(left, "LOST (no pose)", (10, H_PANEL - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv2.putText(right, os.path.basename(run), (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 60, 60), 1)
        vw.write(np.hstack([left, right]))
        if n % 500 == 0:
            print(f"  {n}/{len(frames)}", flush=True)
    vw.release()
    print(f"video: {out}  ({len(frames)} frames at {fps:.1f} fps, {time.time() - t_start:.0f} s)")


if __name__ == "__main__":
    main()
