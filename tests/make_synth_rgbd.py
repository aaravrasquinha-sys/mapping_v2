#!/usr/bin/env python3
"""Make a synthetic RGB-D + IMU walk with exact ground truth, in the exporter's dataset format.

A textured room (6 x 4.5 x 2.6 m) with furniture is ray-cast (Open3D) along a closed walk
around the room at walking speed, with D435-like depth noise, then written exactly like
bagtools/export_db3.py output (rgb/, depth/ at 5000 units/m, TUM lists, frames.csv, IMU CSVs,
calib.yaml, intrinsic.json, rtabmap_calib.yaml, meta.json) plus groundtruth.txt
(TUM, camera optical frame in a z-up world). Used to test engines, converters, evaluator, fuser.

Usage: python tests/make_synth_rgbd.py OUT_DIR [--fps 30] [--speed 0.35] [--seconds N]
"""
import argparse
import csv
import json
import os
import sys

import cv2
import numpy as np
import open3d as o3d
import yaml
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "bagtools"))
from export_db3 import write_rtabmap_calib  # noqa: E402

W, H = 640, 480
FX, FY, CX, CY = 606.75, 606.57, 320.19, 237.06
DEPTH_FX, BASELINE_M, SUBPIX = 385.0, 0.0499, 0.08      # D435-like stereo noise model
ROOM = (6.0, 4.5, 2.6)
G = 9.81


# ----------------------------------------------------------------------------- scene

def box(x0, y0, z0, x1, y1, z1):
    m = o3d.geometry.TriangleMesh.create_box(x1 - x0, y1 - y0, z1 - z0)
    m.translate((x0, y0, z0))
    return m


def build_scene():
    Lx, Ly, Lz = ROOM
    t = 0.1
    parts = [box(-t, -t, -t, Lx + t, Ly + t, 0),            # floor
             box(-t, -t, Lz, Lx + t, Ly + t, Lz + t),        # ceiling
             box(-t, -t, 0, 0, Ly + t, Lz), box(Lx, -t, 0, Lx + t, Ly + t, Lz),
             box(-t, -t, 0, Lx + t, 0, Lz), box(-t, Ly, 0, Lx + t, Ly + t, Lz),
             box(2.3, 1.7, 0, 3.7, 2.8, 0.75),               # table in the middle
             box(0.0, 3.6, 0, 1.2, 4.5, 1.9),                # cupboard in a corner
             box(4.9, 0.0, 0, 6.0, 0.7, 1.0),                # desk
             box(5.4, 2.0, 0, 6.0, 2.6, 1.4),                # shelf
             box(2.6, 0.0, 0.9, 3.4, 0.05, 1.7)]             # picture frame on a wall
    mesh = o3d.geometry.TriangleMesh()
    for p in parts:
        mesh += p
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return scene


def texture(p, n):
    """Procedural colour from the hit point: blotches at three scales -> stable features."""
    def cell_noise(q, size, seed):
        c = np.floor(q / size).astype(np.int64)
        h = (c[:, 0] * 73856093) ^ (c[:, 1] * 19349663) ^ (c[:, 2] * 83492791) ^ seed
        h = (h * 2654435761) & 0xFFFFFFFF
        return np.stack([(h >> 0) & 255, (h >> 8) & 255, (h >> 16) & 255], 1).astype(np.float32)
    # offset points slightly into the surface so the cell does not flicker on the plane itself
    q = p - 0.001 * n
    col = 0.5 * cell_noise(q, 0.30, 11) + 0.35 * cell_noise(q, 0.09, 5) + 0.15 * cell_noise(q, 0.03, 7)
    shade = 0.55 + 0.45 * np.abs(n @ np.array([0.3, 0.5, 0.81]))
    return np.clip(col * shade[:, None], 0, 255).astype(np.uint8)


# ----------------------------------------------------------------------------- trajectory

def path_point(s, total_inset=1.2, r=0.6):
    """Rounded-rectangle loop inside the room, counter-clockwise, arc length s (m)."""
    Lx, Ly, _ = ROOM
    x0, y0, x1, y1 = total_inset, total_inset, Lx - total_inset, Ly - total_inset
    a, b = (x1 - x0) - 2 * r, (y1 - y0) - 2 * r
    arc = 0.5 * np.pi * r
    segs = [("line", (x0 + r, y0), (1, 0), a), ("arc", (x1 - r, y0 + r), -0.5 * np.pi, arc),
            ("line", (x1, y0 + r), (0, 1), b), ("arc", (x1 - r, y1 - r), 0.0, arc),
            ("line", (x1 - r, y1), (-1, 0), a), ("arc", (x0 + r, y1 - r), 0.5 * np.pi, arc),
            ("line", (x0, y1 - r), (0, -1), b), ("arc", (x0 + r, y0 + r), np.pi, arc)]
    L = sum(sg[3] for sg in segs)
    s = s % L
    for kind, p0, d, length in segs:
        if s <= length:
            if kind == "line":
                return np.array(p0) + s * np.array(d), np.arctan2(d[1], d[0]), L
            ang = d + s / r
            c = np.array(p0) + r * np.array([np.cos(ang), np.sin(ang)])
            return c, ang + 0.5 * np.pi, L
        s -= length
    raise RuntimeError


def pose_at(t, T_total, speed, hold=1.5):
    """World-from-camera (optical) 4x4 at time t. Still for `hold` s at both ends, smooth ramps."""
    _, _, L = path_point(0.0)
    move_T = T_total - 2 * hold
    u = np.clip((t - hold) / move_T, 0, 1)
    s = L * (u - np.sin(2 * np.pi * u) / (2 * np.pi))    # smooth start/stop, ends exactly at L
    xy, heading, _ = path_point(s)
    walk = np.sin(np.pi * u) ** 2          # smooth 0 -> 1 -> 0: no jumps when the walk starts/stops
    yaw = heading - np.deg2rad(35)                         # look toward the outer wall
    pitch = np.deg2rad(-12 + 4 * np.sin(2 * np.pi * t / 7.0) * walk)
    roll = np.deg2rad(2 * np.sin(2 * np.pi * t / 5.0) * walk)
    z = 1.40 + 0.015 * np.sin(2 * np.pi * t / 1.1) * walk
    fwd = np.array([np.cos(yaw) * np.cos(pitch), np.sin(yaw) * np.cos(pitch), np.sin(pitch)])
    right = np.cross(fwd, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = np.stack([right, down, fwd], 1)                    # columns = camera x, y, z in world
    R = R @ Rot.from_euler("z", roll).as_matrix()          # roll about the optical axis
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [xy[0], xy[1], z]
    return T


# ----------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--speed", type=float, default=0.35, help="mean walking speed m/s")
    ap.add_argument("--seconds", type=float, default=None, help="default: one full loop")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    os.makedirs(os.path.join(a.out, "rgb"), exist_ok=True)
    os.makedirs(os.path.join(a.out, "depth"), exist_ok=True)

    _, _, L = path_point(0.0)
    T_total = a.seconds or (L / a.speed + 3.0)
    n = int(T_total * a.fps) + 1
    t0 = 1792054584.0
    scene = build_scene()

    K = o3d.core.Tensor([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=o3d.core.Dtype.Float64)
    rows, gt = [], []
    for k in range(n):
        t = k / a.fps
        Twc = pose_at(t, T_total, a.speed)
        rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
            K, o3d.core.Tensor(np.linalg.inv(Twc)), W, H)
        hit = scene.cast_rays(rays)
        tdist = hit["t_hit"].numpy()
        normals = hit["primitive_normals"].numpy().reshape(-1, 3)
        r = rays.numpy().reshape(-1, 6)
        pts = r[:, :3] + r[:, 3:] * tdist.reshape(-1, 1)
        valid = np.isfinite(tdist.reshape(-1))
        col = np.zeros((H * W, 3), np.uint8)
        col[valid] = texture(pts[valid], normals[valid])
        rgb = col.reshape(H, W, 3)
        # depth = z in camera frame
        pc = (np.linalg.inv(Twc) @ np.c_[pts, np.ones(len(pts))].T).T[:, 2]
        z = np.where(valid, pc, 0).reshape(H, W)
        sigma = z ** 2 * SUBPIX / (DEPTH_FX * BASELINE_M)
        zn = z + rng.normal(size=z.shape) * sigma
        zn[(z <= 0.2) | (z > 6.0) | (rng.random(z.shape) < 0.01)] = 0
        tum = np.clip(np.round(zn * 5000), 0, 65535).astype(np.uint16)
        name = f"{t0 + t:017.6f}"
        cv2.imwrite(os.path.join(a.out, "rgb", name + ".png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(a.out, "depth", name + ".png"), tum)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        rows.append({"name": name, "t_colour_s": f"{t0 + t:.6f}", "colour_frame_number": k,
                     "t_depth_s": f"{t0 + t:.6f}", "depth_frame_number": k, "dt_depth_minus_colour_ms": "0.000",
                     "flag": "", "blur_lapvar": f"{cv2.Laplacian(gray, cv2.CV_64F).var():.1f}",
                     "brightness": f"{gray.mean():.1f}", "valid_depth_frac": f"{(tum > 0).mean():.4f}"})
        q = Rot.from_matrix(Twc[:3, :3]).as_quat()
        gt.append(f"{t0 + t:.6f} {Twc[0, 3]:.6f} {Twc[1, 3]:.6f} {Twc[2, 3]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}")
        if k % 100 == 0:
            print(f"  frame {k}/{n}", flush=True)

    with open(os.path.join(a.out, "frames.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(a.out, "groundtruth.txt"), "w") as fh:
        fh.write("# timestamp tx ty tz qx qy qz qw  (camera optical frame in z-up world)\n" + "\n".join(gt) + "\n")
    for fname, folder in (("rgb.txt", "rgb"), ("depth.txt", "depth")):
        with open(os.path.join(a.out, fname), "w") as fh:
            fh.write("# synthetic\n# \n# timestamp filename\n")
            fh.writelines(f"{r['t_colour_s']} {folder}/{r['name']}.png\n" for r in rows)
    with open(os.path.join(a.out, "associations.txt"), "w") as fh:
        fh.writelines(f"{r['t_colour_s']} rgb/{r['name']}.png {r['t_colour_s']} depth/{r['name']}.png\n" for r in rows)
    for link, target in (("rgb_sync", "rgb"), ("depth_sync", "depth")):
        p = os.path.join(a.out, link)
        if not os.path.lexists(p):
            os.symlink(target, p)

    # IMU: gyro 400 Hz, accel 200 Hz, IMU axes = camera optical axes (like the D435i), with bias + noise
    def imu(rate, kind):
        ts = np.arange(0, T_total, 1.0 / rate)
        out = []
        dt = 1e-3
        for t in ts:
            Ta, Tb, Tc = pose_at(t - dt, T_total, a.speed), pose_at(t, T_total, a.speed), pose_at(t + dt, T_total, a.speed)
            if kind == "gyro":
                w = Rot.from_matrix(Ta[:3, :3].T @ Tc[:3, :3]).as_rotvec() / (2 * dt)
                v = w + np.deg2rad([0.3, -0.2, 0.25]) + rng.normal(scale=np.deg2rad(0.1), size=3)
            else:
                acc = (Tc[:3, 3] - 2 * Tb[:3, 3] + Ta[:3, 3]) / dt ** 2
                v = Tb[:3, :3].T @ (acc + np.array([0, 0, G])) + np.array([0.08, -0.05, 0.06]) \
                    + rng.normal(scale=0.02, size=3)
            out.append((t0 + t, v))
        return out
    gyro, accel = imu(400, "gyro"), imu(200, "accel")
    for fname, data in (("imu_gyro.csv", gyro), ("imu_accel.csv", accel)):
        with open(os.path.join(a.out, fname), "w") as fh:
            fh.write("t_s,frame_number,x,y,z\n")
            fh.writelines(f"{t:.6f},{i},{v[0]:.9g},{v[1]:.9g},{v[2]:.9g}\n" for i, (t, v) in enumerate(data))

    ci = {"width": W, "height": H, "fx": FX, "fy": FY, "cx": CX, "cy": CY, "model": "none", "coeffs": [0.0] * 5}
    I4 = np.eye(4).tolist()
    T_ci = np.eye(4)
    T_ci[:3, 3] = [-0.005, -0.005, -0.012]
    calib = {"colour": ci, "depth_raw": dict(ci, fx=DEPTH_FX, fy=DEPTH_FX, cx=320.0, cy=240.0),
             "aligned_depth_uses_colour_intrinsics": True, "T_colour_from_depth": I4,
             "depth_scale_m_per_unit_raw": 0.001, "depth_png_units_per_m": 5000, "fps": a.fps,
             "stereo_baseline_mm": BASELINE_M * 1000, "gyro_rate_hz": 400, "accel_rate_hz": 200,
             "T_colour_from_gyro": T_ci.tolist(), "T_colour_from_accel": T_ci.tolist()}
    with open(os.path.join(a.out, "calib.yaml"), "w") as fh:
        yaml.safe_dump(calib, fh, sort_keys=False)
    with open(os.path.join(a.out, "intrinsic.json"), "w") as fh:
        json.dump({"width": W, "height": H, "intrinsic_matrix": [FX, 0, 0, 0, FY, 0, CX, CY, 1]}, fh)
    write_rtabmap_calib(os.path.join(a.out, "rtabmap_calib.yaml"), ci)
    with open(os.path.join(a.out, "meta.json"), "w") as fh:
        json.dump({"synthetic": True, "frames": n, "seconds": T_total, "loop_length_m": L,
                   "colour_frames_written": n, "gyro_samples": len(gyro), "accel_samples": len(accel),
                   "warnings": []}, fh, indent=2)
    print(f"wrote {a.out}: {n} frames, {T_total:.1f} s, loop {L:.2f} m")


if __name__ == "__main__":
    main()
