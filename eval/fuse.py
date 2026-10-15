#!/usr/bin/env python3
"""Shared map builder: fuse every frame of a dataset into a TSDF using a run's trajectory.

The same builder is used for every engine, so map differences come from the poses alone.
- depth limit from this camera's noise model: sigma_z = k z^2 <= 4 cm (k from calib.yaml)
- flying pixels at depth edges removed (3x3 depth range above max(6 sigma, 3 cm))
- map frame: gravity 'up' = +z (from RUN/metrics.json, made by eval/evaluate.py), first camera
  looking along +x, floor at z = 0, metres
- floating fragments (connected pieces under --min-piece-m2 of surface) removed from the mesh

Outputs in RUN_DIR: map_mesh.ply, map_cloud.ply, map_frame.json, trajectory_map.tum, views.png

Usage: python eval/fuse.py DATASET RUN_DIR [--voxel 0.01] [--trunc 0.04] [--depth-max auto]
                           [--every 1] [--min-piece-m2 0.05]
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
from mv2lib.traj import match_to_frames, anchor_first, read_tum, valid_mask, write_tum  # noqa: E402

SIGMA_MAX_M = 0.04


def edge_filter(z, k):
    """Invalidate depth pixels next to a depth jump larger than max(6 sigma(z), 3 cm) (noise alone spans ~3 sigma)."""
    big = np.where(z > 0, z, 0).astype(np.float32)
    small = np.where(z > 0, z, 1e6).astype(np.float32)
    kernel = np.ones((3, 3), np.uint8)
    zmax = cv2.dilate(big, kernel)
    zmin = cv2.erode(small, kernel)
    thr = np.maximum(6 * k * z ** 2, 0.03)
    out = z.copy()
    out[(zmax - zmin) > thr] = 0
    return out


def floor_height(mesh):
    """Height of the lowest large up-facing surface (the floor)."""
    mesh.compute_vertex_normals()
    v = np.asarray(mesh.vertices)
    n = np.asarray(mesh.vertex_normals)
    up = v[n[:, 2] > 0.9, 2]
    if len(up) < 100:
        return float(np.percentile(v[:, 2], 1)) if len(v) else 0.0
    lo = up[up <= np.percentile(up, 40)]
    hist, edges = np.histogram(lo, bins=np.arange(lo.min() - 0.01, lo.max() + 0.02, 0.01))
    i = int(np.argmax(hist))
    return float(0.5 * (edges[i] + edges[i + 1]))


def remove_small_pieces(mesh, min_area):
    if len(mesh.triangles) == 0:
        return mesh, 0
    labels, counts, areas = mesh.cluster_connected_triangles()
    labels = np.asarray(labels)
    areas = np.asarray(areas)
    drop = areas[labels] < min_area
    mesh.remove_triangles_by_mask(drop)
    mesh.remove_unreferenced_vertices()
    return mesh, int((areas < min_area).sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("run_dir")
    ap.add_argument("--voxel", type=float, default=0.01)
    ap.add_argument("--trunc", type=float, default=0.04)
    ap.add_argument("--depth-max", default="auto")
    ap.add_argument("--every", type=int, default=1, help="use every N-th frame (1 = all)")
    ap.add_argument("--min-piece-m2", type=float, default=0.05)
    a = ap.parse_args()
    t_start = time.time()
    ds = Dataset(a.dataset)
    run = os.path.abspath(a.run_dir)
    st, T = read_tum(os.path.join(run, "trajectory.tum"))
    poses, _ = match_to_frames(ds.stamps, st, T)
    poses = anchor_first(poses)
    m = valid_mask(poses)

    warnings = []
    R = np.eye(3)
    mpath = os.path.join(run, "metrics.json")
    if os.path.isfile(mpath):
        g = json.load(open(mpath)).get("gravity", {})
        if "R_map_from_traj" in g:
            R = np.array(g["R_map_from_traj"])
        else:
            warnings.append("no gravity estimate in metrics.json: map is NOT gravity-aligned")
    else:
        warnings.append("no metrics.json (run eval/evaluate.py first): map is NOT gravity-aligned")
    A = np.eye(4)
    A[:3, :3] = R

    k = ds.depth_noise_model()
    zmax = float(a.depth_max) if a.depth_max != "auto" else float(np.clip(np.sqrt(SIGMA_MAX_M / k), 1.5, 6.0))
    # Open3D tensor VoxelBlockGrid (the legacy ScalableTSDFVolume fuses nothing in open3d-cpu 0.20.0)
    dev = o3d.core.Device("CPU:0")
    vbg = o3d.t.geometry.VoxelBlockGrid(
        attr_names=("tsdf", "weight", "color"),
        attr_dtypes=(o3d.core.float32, o3d.core.float32, o3d.core.float32),
        attr_channels=((1), (1), (3)), voxel_size=a.voxel, block_resolution=8, block_count=20000, device=dev)
    K = o3d.core.Tensor(ds.K, o3d.core.float64)
    trunc_mult = a.trunc / a.voxel
    idx = [i for i in range(0, len(ds), a.every) if m[i]]
    print(f"Fusing {len(idx)} frames (of {len(ds)}), voxel {a.voxel} m, trunc {a.trunc} m, depth <= {zmax:.2f} m")
    for n, i in enumerate(idx):
        z = edge_filter(ds.depth_m(i), k)
        rgb = cv2.cvtColor(ds.colour(i), cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        d = o3d.t.geometry.Image(o3d.core.Tensor(np.ascontiguousarray(z)))
        c = o3d.t.geometry.Image(o3d.core.Tensor(np.ascontiguousarray(rgb)))
        E = o3d.core.Tensor(np.linalg.inv(A @ poses[i]), o3d.core.float64)
        blocks = vbg.compute_unique_block_coordinates(d, K, E, 1.0, zmax, trunc_voxel_multiplier=trunc_mult)
        vbg.integrate(blocks, d, c, K, K, E, 1.0, zmax, trunc_voxel_multiplier=trunc_mult)
        if n % 500 == 0:
            print(f"  {n}/{len(idx)}", flush=True)

    mesh = vbg.extract_triangle_mesh().to_legacy()
    floor = floor_height(mesh)
    S = np.eye(4)
    S[2, 3] = -floor
    mesh.transform(S)
    mesh, n_removed = remove_small_pieces(mesh, a.min_piece_m2)
    mesh.compute_vertex_normals()
    cloud = vbg.extract_point_cloud().to_legacy().transform(S).voxel_down_sample(a.voxel)
    o3d.io.write_triangle_mesh(os.path.join(run, "map_mesh.ply"), mesh)
    o3d.io.write_point_cloud(os.path.join(run, "map_cloud.ply"), cloud)
    T_map = S @ A
    write_tum(os.path.join(run, "trajectory_map.tum"), ds.stamps[m], np.einsum("ij,njk->nik", T_map, poses[m]),
              header="timestamp tx ty tz qx qy qz qw  (colour optical frame in the map frame)")
    info = {"T_map_from_traj": T_map.round(9).tolist(), "up_axis": "+z", "units": "m",
            "floor_at_z": 0.0, "floor_shift_m": -floor, "first_camera_looks_along": "+x (horizontal projection)",
            "voxel_m": a.voxel, "trunc_m": a.trunc, "depth_max_m": zmax, "depth_noise_k": k,
            "frames_fused": len(idx), "every": a.every, "pieces_removed": n_removed,
            "mesh_vertices": len(mesh.vertices), "mesh_triangles": len(mesh.triangles), "cloud_points": len(cloud.points),
            "seconds": round(time.time() - t_start, 1), "warnings": warnings}
    with open(os.path.join(run, "map_frame.json"), "w") as fh:
        json.dump(info, fh, indent=2)
    for wmsg in warnings:
        print("WARNING:", wmsg)
    print(f"mesh {len(mesh.triangles)} triangles, cloud {len(cloud.points)} points, floor shift {-floor:+.3f} m, "
          f"{info['seconds']} s -> {run}/map_mesh.ply, map_cloud.ply")

    from render_views import render  # noqa: E402  (same folder)
    render(run)


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    main()
