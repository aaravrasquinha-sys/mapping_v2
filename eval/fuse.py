#!/usr/bin/env python3
"""Shared map builder: fuse every frame of a dataset into a TSDF using a run's trajectory.

The same builder is used for every engine, so map differences come from the poses alone.
- depth limit from this camera's noise model: sigma_z = k z^2 <= 4 cm (k from calib.yaml)
- flying pixels at depth edges removed (3x3 depth range above max(6 sigma, 3 cm))
- map frame: gravity 'up' = +z (from RUN/metrics.json, made by eval/evaluate.py), first camera
  looking along +x, floor at z = 0, metres
- floating fragments (connected pieces under --min-piece-m2 of surface) removed from the mesh

Outputs in RUN_DIR: map_mesh.ply, map_cloud.ply, map_frame.json, trajectory_map.tum, views.png

Memory guard (--voxel auto, the default): starts at 1 cm voxels; if the map grows past the memory
budget (--mem-gb, default 90% of the RAM; the rest of a peak can go to swap) it starts again with coarser voxels
(1.5 cm, 2 cm, 3 cm, 4 cm), so a long corridor or a whole floor cannot run the machine out of memory.
The voxel size used is printed and stored in map_frame.json. Env MV2_VOXEL / MV2_FUSE_MEM_GB set the defaults.

Usage: python eval/fuse.py DATASET RUN_DIR [--voxel auto|0.01] [--trunc 4*voxel] [--depth-max auto]
                           [--mem-gb auto] [--every 1] [--min-piece-m2 0.05]
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
VOXEL_LADDER = (0.01, 0.015, 0.02, 0.03, 0.04)
# measured (Open3D 0.20 CPU, block_resolution 8, whole fuse.py incl. mesh/cloud extraction and views):
# peak RSS ~= 0.35 GB + 40 KB per voxel block (synthetic room: 17.9k blocks -> 1.05 GB, 4.2k -> 0.51 GB).
# Budget with 20% margin. A 1 cm map uses ~150 blocks per m^2 of surface seen.
BYTES_PER_BLOCK = 48 * 1024
FIXED_GB = 0.35


def mem_total_gb():
    try:
        for line in open("/proc/meminfo"):
            if line.startswith("MemTotal:"):
                return int(line.split()[1]) / 1024 ** 2
    except OSError:
        pass
    return 4.0


class OverBudget(Exception):
    pass


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


def floor_check(mesh, voxel):
    """Plane fit to the up-facing surface at floor level (map frame, floor at z=0): tilt against
    gravity and flatness. A bent map or a wrong 'up' shows up here."""
    v = np.asarray(mesh.vertices)
    n = np.asarray(mesh.vertex_normals)
    sel = (n[:, 2] > 0.9) & (np.abs(v[:, 2]) < 0.06)
    area = float(sel.sum() * voxel ** 2 / 2.0)          # ~2 vertices per voxel^2 on a surface
    if sel.sum() < 500:
        return {"error": "too little floor seen", "floor_area_m2": area}
    q = v[sel]
    c = q.mean(0)
    _, _, Vt = np.linalg.svd(q - c, full_matrices=False)
    normal = Vt[2] if Vt[2, 2] > 0 else -Vt[2]
    resid = (q - c) @ normal
    # robust: refit on inliers within 3 cm
    keep = np.abs(resid) < 0.03
    c = q[keep].mean(0)
    _, _, Vt = np.linalg.svd(q[keep] - c, full_matrices=False)
    normal = Vt[2] if Vt[2, 2] > 0 else -Vt[2]
    resid = (q[keep] - c) @ normal
    return {"tilt_deg": float(np.degrees(np.arccos(np.clip(normal[2], -1, 1)))),
            "flatness_rms_m": float(np.sqrt((resid ** 2).mean())), "floor_area_m2": area,
            "extent_m": (q[keep].max(0) - q[keep].min(0))[:2].round(2).tolist()}


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
    ap.add_argument("--voxel", default=os.environ.get("MV2_VOXEL", "auto"),
                    help="voxel size in m, or auto (1 cm, coarser only if the memory budget would be exceeded)")
    ap.add_argument("--trunc", type=float, default=None, help="TSDF truncation in m (default 4 x voxel)")
    ap.add_argument("--mem-gb", default=os.environ.get("MV2_FUSE_MEM_GB", "auto"),
                    help="memory budget for the map in GB (default: 90%% of the RAM)")
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
    K = o3d.core.Tensor(ds.K, o3d.core.float64)
    outliers = set()
    if os.path.isfile(mpath):
        outliers = set(json.load(open(mpath)).get("outlier_frames", []))
    idx = [i for i in range(0, len(ds), a.every) if m[i] and i not in outliers]
    if outliers:
        print(f"skipping {len(outliers)} outlier frames flagged by evaluate.py (single-frame jumps)")

    mem_gb = float(a.mem_gb) if a.mem_gb != "auto" else max(1.5, 0.9 * mem_total_gb())
    max_blocks = int(max(0.25, mem_gb - FIXED_GB) * 1024 ** 3 / BYTES_PER_BLOCK)
    if a.voxel == "auto":
        ladder = list(VOXEL_LADDER)
    else:
        ladder = [float(a.voxel)]
    print(f"memory budget {mem_gb:.1f} GB -> at most {max_blocks} voxel blocks; voxel sizes to try: {ladder}")

    def fuse_once(voxel, trunc, limit):
        # Open3D tensor VoxelBlockGrid (the legacy ScalableTSDFVolume fuses nothing in open3d-cpu 0.20.0)
        vbg = o3d.t.geometry.VoxelBlockGrid(
            attr_names=("tsdf", "weight", "color"),
            attr_dtypes=(o3d.core.float32, o3d.core.float32, o3d.core.float32),
            attr_channels=((1), (1), (3)), voxel_size=voxel, block_resolution=8, block_count=20000,
            device=o3d.core.Device("CPU:0"))
        trunc_mult = trunc / voxel
        print(f"Fusing {len(idx)} frames (of {len(ds)}), voxel {voxel} m, trunc {trunc} m, depth <= {zmax:.2f} m",
              flush=True)
        no_depth = 0
        for n, i in enumerate(idx):
            z = edge_filter(ds.depth_m(i), k)
            if np.count_nonzero((z > 0) & (z < zmax)) < 500:       # nothing to fuse (covered lens, blank frame)
                no_depth += 1
                continue
            rgb = cv2.cvtColor(ds.colour(i), cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            d = o3d.t.geometry.Image(o3d.core.Tensor(np.ascontiguousarray(z)))
            c = o3d.t.geometry.Image(o3d.core.Tensor(np.ascontiguousarray(rgb)))
            E = o3d.core.Tensor(np.linalg.inv(A @ poses[i]), o3d.core.float64)
            blocks = vbg.compute_unique_block_coordinates(d, K, E, 1.0, zmax, trunc_voxel_multiplier=trunc_mult)
            vbg.integrate(blocks, d, c, K, K, E, 1.0, zmax, trunc_voxel_multiplier=trunc_mult)
            if n % 50 == 0:
                nb = vbg.hashmap().size()
                if limit and nb > limit:
                    raise OverBudget(f"{nb} blocks after {n}/{len(idx)} frames")
                if n % 500 == 0:
                    print(f"  {n}/{len(idx)}  ({nb} blocks, ~{FIXED_GB + nb * BYTES_PER_BLOCK / 1024 ** 3:.1f} GB at the peak)",
                          flush=True)
        return vbg, no_depth

    vbg = None
    for j, voxel in enumerate(ladder):
        trunc = a.trunc if a.trunc is not None else round(4 * voxel, 4)
        last = j == len(ladder) - 1
        try:
            vbg, no_depth = fuse_once(voxel, trunc, None if last else max_blocks)
            break
        except OverBudget as e:
            msg = f"voxel {voxel} m needs more than the {mem_gb:.1f} GB budget ({e}); starting again coarser"
            print("NOTE:", msg, flush=True)
            warnings.append(msg)
            vbg = None
            import gc
            gc.collect()
    a.voxel, a.trunc = voxel, trunc
    n_blocks = int(vbg.hashmap().size())
    print(f"fused with voxel {voxel} m: {n_blocks} blocks (~{FIXED_GB + n_blocks * BYTES_PER_BLOCK / 1024 ** 3:.1f} GB peak estimate)")

    if no_depth:
        print(f"skipped {no_depth} frames with (almost) no valid depth")
    mesh = vbg.extract_triangle_mesh().to_legacy()
    floor_z = floor_height(mesh)
    S = np.eye(4)
    S[2, 3] = -floor_z
    mesh.transform(S)
    mesh, n_removed = remove_small_pieces(mesh, a.min_piece_m2)
    mesh.compute_vertex_normals()
    floor = floor_check(mesh, a.voxel)
    cloud = vbg.extract_point_cloud().to_legacy().transform(S).voxel_down_sample(a.voxel)
    o3d.io.write_triangle_mesh(os.path.join(run, "map_mesh.ply"), mesh)
    o3d.io.write_point_cloud(os.path.join(run, "map_cloud.ply"), cloud)
    T_map = S @ A
    write_tum(os.path.join(run, "trajectory_map.tum"), ds.stamps[m], np.einsum("ij,njk->nik", T_map, poses[m]),
              header="timestamp tx ty tz qx qy qz qw  (colour optical frame in the map frame)")
    info = {"T_map_from_traj": T_map.round(9).tolist(), "up_axis": "+z", "units": "m",
            "floor_at_z": 0.0, "floor_shift_m": -floor_z, "floor": floor, "first_camera_looks_along": "+x (horizontal projection)",
            "voxel_m": a.voxel, "trunc_m": a.trunc, "voxel_blocks": n_blocks, "mem_budget_gb": round(mem_gb, 2), "depth_max_m": zmax, "depth_noise_k": k,
            "frames_fused": len(idx), "every": a.every, "outlier_frames_skipped": len(outliers), "no_depth_frames_skipped": no_depth, "pieces_removed": n_removed,
            "mesh_vertices": len(mesh.vertices), "mesh_triangles": len(mesh.triangles), "cloud_points": len(cloud.points),
            "seconds": round(time.time() - t_start, 1), "warnings": warnings}
    with open(os.path.join(run, "map_frame.json"), "w") as fh:
        json.dump(info, fh, indent=2)
    for wmsg in warnings:
        print("WARNING:", wmsg)
    if "tilt_deg" in floor:
        print(f"floor: tilt {floor['tilt_deg']:.2f} deg vs gravity, flatness {100 * floor['flatness_rms_m']:.2f} cm RMS, "
              f"{floor['floor_area_m2']:.1f} m^2 seen")
    else:
        print("floor:", floor.get("error"))
    print(f"mesh {len(mesh.triangles)} triangles, cloud {len(cloud.points)} points, floor shift {-floor_z:+.3f} m, "
          f"{info['seconds']} s -> {run}/map_mesh.ply, map_cloud.ply")

    from render_views import render  # noqa: E402  (same folder)
    render(run)


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    main()
