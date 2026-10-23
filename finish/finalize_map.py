#!/usr/bin/env python3
"""Package the final map from the winning run (S8 in PLAN.md).

Input: a run folder already evaluated and fused (metrics.json, map_mesh.ply, trajectory_map.tum,
map_frame.json; made by the engine runners or eval/evaluate.py + eval/fuse.py).
Steps:
 1. optional (--colour-opt) colour refinement: Open3D colour-map optimisation (rigid) on the sharpest
    frame of every --key-every-s seconds; kept ONLY if the photometric error (vertex colour vs the
    images where the vertex is visible, measured with the run's poses) goes down. Off by default: the
    optimiser also moves the cameras, and on a synthetic walk with exact poses it made colours worse
    (0.017 -> 0.121); try it on real data and let the check decide
 2. a lighter mesh for viewing (--view-triangles)
 3. everything copied to OUT_DIR with MANIFEST.json (source run, versions, metrics vs targets,
    sha256 of every file) and README.txt
The map frame is unchanged: metres, +z up (gravity), floor at z = 0, first camera looking along +x.

Usage: python finish/finalize_map.py DATASET RUN_DIR OUT_DIR [--colour-opt] [--key-every-s 1.0]
                                     [--view-triangles 500000]
"""
import argparse
import datetime
import hashlib
import json
import os
import shutil
import sys
import time

import cv2
import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mv2lib.dataset import Dataset  # noqa: E402
from mv2lib.traj import match_to_frames, read_tum  # noqa: E402


def keyframes(ds, poses, every_s):
    """Sharpest posed frame (variance of Laplacian from frames.csv) in each time window."""
    blur = {r["name"]: float(r["blur_lapvar"] or 0) for r in ds.frames_csv() if r.get("blur_lapvar")}
    out, t0, best = [], None, None
    for i, t in enumerate(ds.stamps):
        if not np.all(np.isfinite(poses[i])):
            continue
        if t0 is None:
            t0 = t
        if t - t0 >= every_s:
            if best is not None:
                out.append(best[1])
            t0, best = t, None
        b = blur.get(ds.names[i], 0.0)
        if best is None or b > best[0]:
            best = (b, i)
    if best is not None:
        out.append(best[1])
    return out


def photometric_error(mesh, ds, poses, idx, max_vertices=200000, seed=0):
    """Mean |vertex colour - image colour| (0..1) over visible vertex/frame pairs."""
    v = np.asarray(mesh.vertices)
    c = np.asarray(mesh.vertex_colors)
    if len(v) > max_vertices:
        sel = np.random.default_rng(seed).choice(len(v), max_vertices, replace=False)
        v, c = v[sel], c[sel]
    errs = []
    for i in idx:
        Tcw = np.linalg.inv(poses[i])
        pc = v @ Tcw[:3, :3].T + Tcw[:3, 3]
        z = pc[:, 2]
        ok = z > 0.3
        uv = pc[ok, :2] / z[ok, None] * [ds.K[0, 0], ds.K[1, 1]] + [ds.K[0, 2], ds.K[1, 2]]
        u, w = np.round(uv[:, 0]).astype(int), np.round(uv[:, 1]).astype(int)
        inside = (u >= 0) & (u < ds.width) & (w >= 0) & (w < ds.height)
        d = ds.depth_m(i)
        idx_ok = np.where(ok)[0][inside]
        dz = d[w[inside], u[inside]]
        vis = (dz > 0) & (np.abs(dz - z[idx_ok]) < 0.03)
        img = cv2.cvtColor(ds.colour(i), cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        col = img[w[inside][vis], u[inside][vis]]
        errs.append(np.abs(col - c[idx_ok[vis]]).mean(1))
    e = np.concatenate(errs) if errs else np.array([np.nan])
    return float(np.mean(e)), int(e.size)


def colour_optimise(mesh, ds, poses, idx):
    intr = o3d.camera.PinholeCameraIntrinsic(ds.width, ds.height, ds.K[0, 0], ds.K[1, 1], ds.K[0, 2], ds.K[1, 2])
    rgbds, params = [], []
    for i in idx:
        rgb = cv2.cvtColor(ds.colour(i), cv2.COLOR_BGR2RGB)
        rgbds.append(o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.ascontiguousarray(rgb)), o3d.geometry.Image(np.ascontiguousarray(ds.depth_m(i))),
            depth_scale=1.0, depth_trunc=4.0, convert_rgb_to_intensity=False))
        p = o3d.camera.PinholeCameraParameters()
        p.intrinsic = intr
        p.extrinsic = np.linalg.inv(poses[i])
        params.append(p)
    traj = o3d.camera.PinholeCameraTrajectory()
    traj.parameters = params
    out, _ = o3d.pipelines.color_map.run_rigid_optimizer(
        o3d.geometry.TriangleMesh(mesh), rgbds, traj, o3d.pipelines.color_map.RigidOptimizerOption(maximum_iteration=100))
    return out


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("run_dir")
    ap.add_argument("out_dir")
    ap.add_argument("--colour-opt", action="store_true", help="try colour-map optimisation (kept only if better)")
    ap.add_argument("--key-every-s", type=float, default=1.0)
    ap.add_argument("--view-triangles", type=int, default=500000)
    a = ap.parse_args()
    t_start = time.time()
    ds = Dataset(a.dataset)
    run = os.path.abspath(a.run_dir)
    for f in ("metrics.json", "map_mesh.ply", "map_cloud.ply", "trajectory_map.tum", "map_frame.json"):
        if not os.path.isfile(os.path.join(run, f)):
            sys.exit(f"ERROR: {run}/{f} missing: run eval/evaluate.py and eval/fuse.py on this run first")
    if os.path.exists(a.out_dir) and os.listdir(a.out_dir):
        sys.exit(f"ERROR: {a.out_dir} exists and is not empty")
    os.makedirs(a.out_dir, exist_ok=True)

    mesh = o3d.io.read_triangle_mesh(os.path.join(run, "map_mesh.ply"))
    st, T = read_tum(os.path.join(run, "trajectory_map.tum"))
    poses, _ = match_to_frames(ds.stamps, st, T)
    colour = {"applied": False}
    if a.colour_opt:
        idx = keyframes(ds, poses, a.key_every_s)
        print(f"colour refinement on {len(idx)} keyframes...", flush=True)
        e0, n0 = photometric_error(mesh, ds, poses, idx)
        opt = colour_optimise(mesh, ds, poses, idx)
        e1, n1 = photometric_error(opt, ds, poses, idx)
        colour = {"keyframes": len(idx), "photometric_error_before": e0, "photometric_error_after": e1,
                  "samples": n0, "applied": bool(e1 < e0)}
        print(f"photometric error {e0:.4f} -> {e1:.4f}: {'kept' if e1 < e0 else 'NOT kept (no improvement)'}")
        if e1 < e0:
            mesh = opt
    mesh.compute_vertex_normals()
    out = os.path.abspath(a.out_dir)
    o3d.io.write_triangle_mesh(os.path.join(out, "map_mesh.ply"), mesh)
    view = mesh.simplify_quadric_decimation(a.view_triangles) if len(mesh.triangles) > a.view_triangles else mesh
    view.compute_vertex_normals()
    o3d.io.write_triangle_mesh(os.path.join(out, "map_mesh_view.ply"), view)
    for f in ("map_cloud.ply", "trajectory_map.tum", "map_frame.json", "views.png", "metrics.json", "run.json"):
        if os.path.isfile(os.path.join(run, f)):
            shutil.copy2(os.path.join(run, f), os.path.join(out, f))

    metrics = json.load(open(os.path.join(run, "metrics.json")))
    frame = json.load(open(os.path.join(run, "map_frame.json")))
    files = sorted(f for f in os.listdir(out) if f not in ("MANIFEST.json", "README.txt"))
    manifest = {
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "dataset": ds.root, "source_bag": ds.meta.get("source_bag"),
        "source_bag_sha256_first64MB_plus_size": ds.meta.get("source_sha256_first64MB_plus_size"),
        "run": run, "engine": metrics.get("run_info", {}).get("engine"),
        "variant": metrics.get("run_info", {}).get("variant"),
        "frame": {"units": "m", "up": "+z (gravity)", "floor_z": 0.0, "first_camera_looks_along": "+x",
                  "T_map_from_traj": frame.get("T_map_from_traj")},
        "summary": {"posed_fraction": metrics["coverage"]["fraction"],
                    "revisit_trans_median_m": metrics.get("revisits", {}).get("trans_median_m"),
                    "revisit_trans_p90_m": metrics.get("revisits", {}).get("trans_p90_m"),
                    "start_end_m": (metrics.get("revisits", {}).get("start_end") or {}).get("trans_m"),
                    "sharpness_ratio": metrics.get("sharpness", {}).get("ratio"),
                    "floor": frame.get("floor"),
                    "gyro_frac_le_1deg": metrics.get("gyro", {}).get("frac_le_1deg")},
        "colour_refinement": colour,
        "versions": {"open3d": o3d.__version__, "numpy": np.__version__, "opencv": cv2.__version__},
        "triangles": len(mesh.triangles), "view_triangles": len(view.triangles),
        "files": {f: sha256(os.path.join(out, f)) for f in files},
        "seconds": round(time.time() - t_start, 1),
    }
    with open(os.path.join(out, "MANIFEST.json"), "w") as fh:
        json.dump(manifest, fh, indent=2)
    vox_cm = f"{100 * float(frame.get('voxel_m', 0.01)):g} cm"
    with open(os.path.join(out, "README.txt"), "w") as fh:
        fh.write(
            "Final map (mapping_v2)\n\n"
            "Frame: metres; +z is up (gravity from the IMU); floor at z = 0; first camera looks along +x.\n\n"
            f"map_mesh.ply        full-resolution coloured mesh (TSDF, {vox_cm} voxels)\n"
            "map_mesh_view.ply   lighter mesh for viewers\n"
            f"map_cloud.ply       coloured point cloud ({vox_cm})\n"
            "trajectory_map.tum  camera path in the map frame (TUM: t x y z qx qy qz qw, colour optical frame)\n"
            "map_frame.json      how the map frame was made, floor tilt/flatness check\n"
            "metrics.json        quality metrics of the trajectory (see PLAN.md for targets)\n"
            "views.png           fixed top and oblique views\n"
            "MANIFEST.json       provenance, summary, sha256 of every file\n")
    print(f"final map: {out}  ({len(mesh.triangles)} triangles; view mesh {len(view.triangles)}; "
          f"{manifest['seconds']} s)")


if __name__ == "__main__":
    main()
