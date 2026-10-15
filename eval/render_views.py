#!/usr/bin/env python3
"""Render fixed views of a run's map (same viewpoints for every engine), headless.

Map frame (from eval/fuse.py): z up, floor at 0, first camera looking along +x.
Panels: top view (everything above 2.0 m cut away so the ceiling does not hide the room,
with the camera path in red) and two oblique views (45 deg down, from the -x/-y and +x/+y
corners). Orthographic point splatting with a depth buffer: no GPU or display needed.

Usage: python eval/render_views.py RUN_DIR [--out views.png]
"""
import argparse
import os
import sys

import cv2
import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mv2lib.traj import read_tum  # noqa: E402

CUT_Z = 2.0
PANEL = 900


def splat(pts, cols, R, size=PANEL, margin=20, scale=None, center=None):
    """Orthographic view: R maps world -> view (x right, y down, z into the screen)."""
    p = pts @ R.T
    if center is None:
        lo, hi = p[:, :2].min(0), p[:, :2].max(0)
        center = 0.5 * (lo + hi)
        scale = (size - 2 * margin) / max(hi - lo)
    uv = ((p[:, :2] - center) * scale + size / 2).astype(int)
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < size) & (uv[:, 1] >= 0) & (uv[:, 1] < size)
    uv, depth, c = uv[ok], p[ok, 2], cols[ok]
    order = np.argsort(-depth)                      # far first, near overwrite
    img = np.full((size, size, 3), 255, np.uint8)
    for du in (0, 1):
        for dv in (0, 1):
            u = np.clip(uv[order, 0] + du, 0, size - 1)
            v = np.clip(uv[order, 1] + dv, 0, size - 1)
            img[v, u] = c[order]
    return img, scale, center


def view_matrix(azimuth_deg, elevation_deg):
    """Camera looking down at the map from the given azimuth/elevation (orthographic)."""
    az, el = np.radians(azimuth_deg), np.radians(elevation_deg)
    fwd = -np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])   # into the scene
    right = np.cross(fwd, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    return np.stack([right, down, fwd])


def render(run, out=None):
    cloud = o3d.io.read_point_cloud(os.path.join(run, "map_cloud.ply"))
    pts = np.asarray(cloud.points)
    cols = (np.asarray(cloud.colors)[:, ::-1] * 255).astype(np.uint8)       # RGB -> BGR
    keep = pts[:, 2] < CUT_Z
    pts, cols = pts[keep], cols[keep]
    panels = []
    # top view: x right, y up on screen
    Rtop = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
    img, s, c = splat(pts, cols, Rtop)
    tp = os.path.join(run, "trajectory_map.tum")
    if os.path.isfile(tp):
        _, T = read_tum(tp)
        q = (T[:, :3, 3] @ Rtop.T)[:, :2]
        q = ((q - c) * s + PANEL / 2).astype(np.int32)
        cv2.polylines(img, [q.reshape(-1, 1, 2)], False, (0, 0, 220), 2)
        cv2.circle(img, tuple(int(x) for x in q[0]), 7, (0, 160, 0), -1)      # start: green
        cv2.circle(img, tuple(int(x) for x in q[-1]), 4, (0, 0, 0), -1)       # end: black
    bar = int(s)                                                              # 1 m scale bar
    cv2.line(img, (30, PANEL - 30), (30 + bar, PANEL - 30), (0, 0, 0), 3)
    cv2.putText(img, "1 m", (30, PANEL - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    cv2.putText(img, f"top (cut at {CUT_Z} m), path red, start green", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    panels.append(img)
    for az, label in ((225, "oblique from -x,-y"), (45, "oblique from +x,+y")):
        img, _, _ = splat(pts, cols, view_matrix(az, 45))
        cv2.putText(img, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        panels.append(img)
    grid = np.hstack(panels)
    cv2.putText(grid, os.path.basename(run.rstrip("/")), (10, PANEL - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (90, 90, 90), 1)
    out = out or os.path.join(run, "views.png")
    cv2.imwrite(out, grid)
    print(f"views: {out}")
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--out")
    a = ap.parse_args()
    render(os.path.abspath(a.run_dir), a.out)
