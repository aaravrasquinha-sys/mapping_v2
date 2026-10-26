#!/usr/bin/env python3
"""RTAB-Map run report: tracking over time, resets, loop closures, path shape, map cleanliness.

Made for comparing RTAB-Map against another SLAM pipeline on the same recording: everything is
given against time from the first colour frame (seconds), not in absolute coordinates.

Per run (RUN_DIR/report/):
  report.md / report.json  summary: parameters, odometry sessions (resets) with times, frames without a
                           pose, loop closures (time pairs), path per leg + the turn, positions every 60 s
                           in a leg-aligned frame, the stationary spin, height wander, points below floor
  windows.csv              per 5 s window: frames, posed %, odometry inliers/features, low-inlier frames,
                           jumps, resets, loop closures
  path.png                 top-down path coloured by time; resets, jumps and loop closures marked
  corridor_views.png       views of the fused map from eye height, looking down each leg (floor visible)
With several runs and --out DIR: DIR/rtabmap_comparison.md with the key numbers side by side.

Inputs used from each run folder: trajectory.tum (+ trajectory_map.tum, map_cloud.ply, metrics.json from
eval/evaluate.py + eval/fuse.py), rtabmap.db (sessions, loop closures; read-only), rtabmap.log (one line per
frame when RTAB-Map ran without --quiet), run.json (parameters).

Usage: python eval/rtabmap_report.py DATASET RUN_DIR [RUN_DIR ...] [--out DIR] [--window-s 5]
                                     [--windows "0-20,100-130,..."] [--spin 340-387] [--turn-window 150-260]
"""
import argparse
import csv
import json
import os
import re
import sqlite3
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from mv2lib.dataset import Dataset  # noqa: E402
from mv2lib.traj import match_to_frames, read_tum, valid_mask  # noqa: E402

LINK_TYPES = {0: "neighbor", 1: "global loop closure", 2: "local proximity (space)", 3: "local proximity (time)",
              4: "user", 5: "virtual", 6: "neighbor merged", 7: "pose prior", 8: "landmark", 9: "gravity"}
LOOP_TYPES = (1, 2, 4)
JUMP_M = 0.10            # a step of >10 cm between consecutive frames (3 m/s at 30 fps) is a jump
LOW_INLIERS = 20
ITER_RE = re.compile(r"Iteration\s+(\d+)/(\d+).*?quality=(-?\d+)/(-?\d+)")
DEFAULT_WINDOWS = "0-20,25-100,100-130,180-230,300-420,340-387,470-490"


# ---------------------------------------------------------------- inputs
def read_db(path, tol=0.002):
    """Nodes (id, map_id, stamp) and links from an RTAB-Map database, read-only. Missing -> empty."""
    out = {"nodes": [], "links": [], "error": None}
    if not os.path.isfile(path):
        out["error"] = "no rtabmap.db"
        return out
    try:
        con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        cols = {r[1] for r in con.execute("PRAGMA table_info(Node)")}
        if not {"id", "map_id", "stamp"} <= cols:
            out["error"] = f"Node table lacks id/map_id/stamp (has {sorted(cols)})"
            return out
        out["nodes"] = con.execute("SELECT id, map_id, stamp FROM Node ORDER BY id").fetchall()
        lcols = {r[1] for r in con.execute("PRAGMA table_info(Link)")}
        if {"from_id", "to_id", "type"} <= lcols:
            out["links"] = con.execute("SELECT from_id, to_id, type FROM Link").fetchall()
        con.close()
    except sqlite3.Error as e:
        out["error"] = f"sqlite: {e}"
    return out


def read_log(path, n_frames):
    """Per-frame odometry (inliers, features) from rtabmap-rgbd_dataset's per-iteration lines."""
    inl = np.full(n_frames, np.nan)
    feat = np.full(n_frames, np.nan)
    if not os.path.isfile(path):
        return inl, feat, 0
    n = 0
    with open(path, errors="replace") as fh:
        for line in fh:
            m = ITER_RE.search(line)
            if not m:
                continue
            i = int(m.group(1)) - 1
            if 0 <= i < n_frames:
                inl[i], feat[i] = float(m.group(3)), float(m.group(4))
                n += 1
    return inl, feat, n


def parse_ranges(text):
    out = []
    for part in text.split(","):
        part = part.strip()
        if part:
            a, b = part.split("-")
            out.append((float(a), float(b)))
    return out


# ---------------------------------------------------------------- geometry
def horizontal_frame(P, up):
    """Two horizontal axes for positions P (posed only)."""
    up = up / np.linalg.norm(up)
    ref = np.array([1.0, 0, 0]) if abs(up[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = ref - up * (ref @ up)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    return e1, e2


def smooth(t, P, win_s=1.0):
    out = np.empty_like(P)
    j0 = j1 = 0
    s = np.zeros(P.shape[1])
    for i in range(len(t)):
        while j1 < len(t) and t[j1] <= t[i] + win_s / 2:
            s += P[j1]
            j1 += 1
        while t[j0] < t[i] - win_s / 2:
            s -= P[j0]
            j0 += 1
        out[i] = s / (j1 - j0)
    return out


def path_geometry(rel, P, up, turn_window=None):
    """Leg/turn analysis on posed positions P [N,3] at times rel [N] with gravity 'up'."""
    e1, e2 = horizontal_frame(P, up)
    H = np.stack([P @ e1, P @ e2], 1)
    Hs = smooth(rel, H, 1.0)
    Zs = smooth(rel, (P @ up)[:, None], 1.0)[:, 0]
    # resample at 0.5 s for length (no frame jitter)
    grid = np.arange(rel[0], rel[-1], 0.5)
    G = np.stack([np.interp(grid, rel, Hs[:, 0]), np.interp(grid, rel, Hs[:, 1]), np.interp(grid, rel, Zs)], 1)
    steps = np.linalg.norm(np.diff(G, axis=0), axis=1)
    cum = np.concatenate([[0], np.cumsum(steps)])
    # heading of horizontal motion over 2 s, only where moving
    k = 4
    d = G[k:, :2] - G[:-k, :2]
    tm = 0.5 * (grid[k:] + grid[:-k])
    moving = np.linalg.norm(d, axis=1) > 0.3
    res = {"path_length_m": float(cum[-1]), "height_range_m": float(Zs.max() - Zs.min()),
           "end_offset_m": float(np.linalg.norm(G[-1] - G[0]))}
    if moving.sum() < 10:
        res["turn"] = None
        return res, None
    head = np.unwrap(np.arctan2(d[moving, 1], d[moving, 0]))
    th = tm[moving]
    best, tb = 0.0, None
    for i, t in enumerate(th):
        if turn_window and not (turn_window[0] <= t <= turn_window[1]):
            continue                      # only look for the corner inside the window given by the user
        a = head[(th >= t - 20) & (th < t)]
        b = head[(th > t) & (th <= t + 20)]
        if len(a) > 3 and len(b) > 3:
            dd = abs(np.median(b) - np.median(a))
            if dd > best:
                best, tb = dd, t
    turn = None
    if tb is not None and np.degrees(best) > 45:
        h0 = np.median(head[(th >= tb - 20) & (th < tb - 5)]) if np.any((th >= tb - 20) & (th < tb - 5)) else head[0]
        h1 = np.median(head[(th > tb + 5) & (th <= tb + 20)]) if np.any((th > tb + 5) & (th <= tb + 20)) else head[-1]
        frac = (head - h0) / (h1 - h0) if abs(h1 - h0) > 1e-6 else np.zeros_like(head)
        near = (th > tb - 40) & (th < tb + 40)
        ts = th[near & (frac >= 0.1)].min() if np.any(near & (frac >= 0.1)) else tb
        te = th[near & (frac <= 0.9)].max() if np.any(near & (frac <= 0.9)) else tb
        L1 = float(np.interp(tb, grid, cum))
        turn = {"time_s": float(tb), "start_s": float(ts), "end_s": float(te), "angle_deg": float(np.degrees(h1 - h0)),
                "leg1_path_m": L1, "leg2_path_m": float(cum[-1] - L1)}
        # leg-aligned frame: origin = first position; A along leg 1, B horizontal towards leg 2
        p_turn = np.array([np.interp(tb, grid, G[:, 0]), np.interp(tb, grid, G[:, 1])])
        A = p_turn - G[0, :2]
        A /= np.linalg.norm(A)
        B = np.array([-A[1], A[0]])
        if (G[-1, :2] - p_turn) @ B < 0:
            B = -B
        turn["leg1_straight_m"] = float(np.linalg.norm(p_turn - G[0, :2]))
        turn["leg2_straight_m"] = float(np.linalg.norm(G[-1, :2] - p_turn))
        res["frame"] = {"A": A.tolist(), "B": B.tolist()}

        def to_leg(t):
            p = np.array([np.interp(t, grid, G[:, 0]), np.interp(t, grid, G[:, 1]), np.interp(t, grid, G[:, 2])])
            q = p[:2] - G[0, :2]
            return [round(float(q @ A), 2), round(float(q @ B), 2), round(float(p[2] - G[0, 2]), 2)]
        res["positions_every_60s"] = {f"{int(t)}": to_leg(t) for t in list(np.arange(0, grid[-1], 60)) + [grid[-1]]
                                      if t >= grid[0]}
    res["turn"] = turn
    return res, {"grid": grid, "G": G, "e1": e1, "e2": e2}


def yaw_change_deg(rel, T, up, t0, t1):
    """Total turning (sum of |heading change| of the camera's forward axis projected horizontally)."""
    sel = (rel >= t0) & (rel <= t1)
    if sel.sum() < 2:
        return None
    f = T[sel][:, :3, 2]
    f = f - np.outer(f @ up, up)
    e1, e2 = horizontal_frame(f, up)
    h = np.unwrap(np.arctan2(f @ e2, f @ e1))
    return float(np.degrees(np.abs(np.diff(h)).sum())), float(np.degrees(h[-1] - h[0]))


# ---------------------------------------------------------------- pictures
def path_plot(out, rel, P, up, resets_t, jump_t, loops_t):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    e1, e2 = horizontal_frame(P, up)
    x, y = P @ e1, P @ e2
    fig, ax = plt.subplots(figsize=(8, 8))
    sc = ax.scatter(x, y, c=rel, s=2, cmap="viridis")
    fig.colorbar(sc, ax=ax, label="time from start (s)", shrink=0.8)

    def at(ts):
        idx = np.clip(np.searchsorted(rel, ts), 0, len(rel) - 1)
        return x[idx], y[idx]
    if len(resets_t):
        ax.scatter(*at(np.array(resets_t)), marker="x", s=80, c="red", label=f"odometry reset ({len(resets_t)})")
    if len(jump_t):
        ax.scatter(*at(np.array(jump_t)), marker="o", s=30, facecolors="none", edgecolors="orange",
                   label=f"jump > {JUMP_M * 100:.0f} cm ({len(jump_t)})")
    for (ta, tb) in loops_t[:300]:
        (xa, ya), (xb, yb) = at(np.array([ta])), at(np.array([tb]))
        ax.plot([xa[0], xb[0]], [ya[0], yb[0]], "m-", lw=0.6, alpha=0.6)
    if loops_t:
        ax.plot([], [], "m-", label=f"loop closure ({len(loops_t)})")
    for t in range(0, int(rel[-1]) + 1, 60):
        xi, yi = at(np.array([t]))
        ax.annotate(f"{t}s", (xi[0], yi[0]), fontsize=8)
    ax.scatter([x[0]], [y[0]], c="green", s=60, label="start", zorder=5)
    ax.set_aspect("equal")
    ax.set_xlabel("m (horizontal)")
    ax.set_ylabel("m (horizontal)")
    ax.legend(loc="best", fontsize=8)
    ax.set_title(os.path.basename(os.path.dirname(os.path.dirname(os.path.abspath(out)))))
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def perspective(pts, cols, eye, fwd, up, W=900, H=600, f=450.0, max_d=40.0):
    """Pinhole view of a coloured point cloud from eye looking along fwd (z-buffered splats)."""
    fwd = fwd / np.linalg.norm(fwd)
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    q = pts - eye
    z = q @ fwd
    ok = (z > 0.2) & (z < max_d)
    u = (q[ok] @ right) / z[ok] * f + W / 2
    v = (q[ok] @ down) / z[ok] * f + H / 2
    z, c = z[ok], cols[ok]
    inside = (u >= 0) & (u < W - 2) & (v >= 0) & (v < H - 2)
    u, v, z, c = u[inside].astype(int), v[inside].astype(int), z[inside], c[inside]
    order = np.argsort(-z)
    img = np.full((H, W, 3), 255, np.uint8)
    for du in (0, 1, 2):
        for dv in (0, 1, 2):
            near = z[order] < (6.0 if (du == 2 or dv == 2) else 1e9)
            o = order[near]
            img[v[o] + dv, u[o] + du] = c[o]
    return img


def corridor_views(run, out, geo, up_map):
    """Views from 1.4 m above the floor at the start of each leg, looking along it, pitched 12 deg down."""
    import open3d as o3d
    cp = os.path.join(run, "map_cloud.ply")
    if not os.path.isfile(cp) or geo is None or "frame" not in geo[0]:
        return None
    cloud = o3d.io.read_point_cloud(cp)
    pts = np.asarray(cloud.points)
    cols = (np.asarray(cloud.colors)[:, ::-1] * 255).astype(np.uint8)
    res, aux = geo
    e1, e2, grid, G = aux["e1"], aux["e2"], aux["grid"], aux["G"]
    A, B = np.array(res["frame"]["A"]), np.array(res["frame"]["B"])
    tb = res["turn"]["time_s"]
    panels = []
    for t, d2, label in ((grid[0] + 2, A, "leg 1 from its start"), (min(tb + 15, grid[-1]), B, "leg 2 after the turn")):
        p2 = np.array([np.interp(t, grid, G[:, 0]), np.interp(t, grid, G[:, 1])])
        eye = p2[0] * e1 + p2[1] * e2 + 1.4 * up_map           # map frame: floor at z = 0
        fwd = d2[0] * e1 + d2[1] * e2
        fwd = fwd / np.linalg.norm(fwd) * np.cos(np.radians(12)) - up_map * np.sin(np.radians(12))
        img = perspective(pts, cols, eye, fwd, up_map)
        cv2.putText(img, f"{label} (t={t:.0f} s), eye 1.4 m", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        panels.append(img)
    cv2.imwrite(out, np.hstack(panels))
    return out


# ---------------------------------------------------------------- one run
def report_run(ds, run, a):
    rel_all = ds.stamps - ds.stamps[0]
    n = len(ds)
    rep = {"run": run, "dataset": ds.root, "frames": n, "duration_s": float(rel_all[-1]),
           "time_zero": "first colour frame", "notes": []}
    rj = os.path.join(run, "run.json")
    if os.path.isfile(rj):
        info = json.load(open(rj))
        rep["rtabmap_version"] = info.get("rtabmap_version")
        rep["parameters"] = info.get("parameters")
        rep["variant"] = info.get("variant")
        rep["runtime_s"] = info.get("runtime_s")
    st, T = read_tum(os.path.join(run, "trajectory.tum"))
    poses, _ = match_to_frames(ds.stamps, st, T)
    posed = valid_mask(poses)
    rep["posed_frames"] = int(posed.sum())
    rep["posed_pct"] = round(100.0 * posed.mean(), 2)

    # map-frame poses (gravity up = +z, floor z = 0) when the map was built; else trajectory frame + metrics up
    up = np.array([0, 0, 1.0])
    Pmap = None
    tmp = os.path.join(run, "trajectory_map.tum")
    if os.path.isfile(tmp):
        s2, T2 = read_tum(tmp)
        Pmap, _ = match_to_frames(ds.stamps, s2, T2)
    else:
        mp = os.path.join(run, "metrics.json")
        g = json.load(open(mp)).get("gravity", {}) if os.path.isfile(mp) else {}
        if "R_map_from_traj" in g:
            R = np.array(g["R_map_from_traj"])
            Pmap = poses.copy()
            Pmap[posed] = np.einsum("ij,njk->nik", np.pad(R, ((0, 1), (0, 1))) + np.diag([0, 0, 0, 1.0]), poses[posed])
        else:
            Pmap = poses
            up = np.array([0, -1.0, 0])                     # optical frame y is down; first camera roughly level
            rep["notes"].append("no gravity estimate: 'up' taken as the first camera's -y (tilted)")
    pm = valid_mask(Pmap)

    # jumps (consecutive posed frames)
    P = Pmap[:, :3, 3]
    jump = np.zeros(n, bool)
    both = pm[1:] & pm[:-1]
    stepd = np.full(n - 1, np.nan)
    stepd[both] = np.linalg.norm(P[1:][both] - P[:-1][both], axis=1)
    jump[1:] = np.nan_to_num(stepd, nan=0) > JUMP_M
    rep["jumps"] = int(jump.sum())
    rep["largest_step_m"] = round(float(np.nanmax(stepd)), 3) if np.isfinite(stepd).any() else None

    # database: sessions (odometry resets) and loop closures
    db = read_db(os.path.join(run, "rtabmap.db"))
    if db["error"]:
        rep["notes"].append(f"database: {db['error']}")
    node_t = {nid: float(s) for nid, _, s in db["nodes"]}
    sessions = []
    for nid, mid, s in db["nodes"]:
        if not sessions or sessions[-1]["map_id"] != mid:
            sessions.append({"map_id": int(mid), "start_s": round(float(s) - ds.stamps[0], 2), "nodes": 0})
        sessions[-1]["nodes"] += 1
        sessions[-1]["end_s"] = round(float(s) - ds.stamps[0], 2)
    rep["odometry_sessions"] = sessions
    resets_t = [x["start_s"] for x in sessions[1:]]
    rep["odometry_resets"] = len(resets_t)
    seen = set()
    loops = []
    type_counts = {}
    for f_, t_, ty in db["links"]:
        type_counts[LINK_TYPES.get(ty, str(ty))] = type_counts.get(LINK_TYPES.get(ty, str(ty)), 0) + 1
        if ty not in LOOP_TYPES or f_ not in node_t or t_ not in node_t:
            continue
        key = (min(f_, t_), max(f_, t_), ty)
        if key in seen:
            continue
        seen.add(key)
        ta, tb = sorted((node_t[f_] - ds.stamps[0], node_t[t_] - ds.stamps[0]))
        loops.append({"t_a_s": round(ta, 2), "t_b_s": round(tb, 2), "type": LINK_TYPES.get(ty, str(ty))})
    loops.sort(key=lambda d: (d["t_b_s"], d["t_a_s"]))
    rep["link_counts"] = type_counts
    rep["loop_closures"] = loops
    if node_t:
        ns = np.array(sorted(node_t.values()))
        j = np.clip(np.searchsorted(ns, ds.stamps), 1, len(ns) - 1)
        has_node = np.minimum(np.abs(ns[j] - ds.stamps), np.abs(ns[j - 1] - ds.stamps)) < 0.002
        rep["frames_without_node"] = int((~has_node).sum())
    else:
        has_node = None

    # log: odometry inliers per frame
    lp_ = os.path.join(run, "rtabmap.log")
    if not os.path.isfile(lp_):                       # R2 keeps a copy of its R1's odometry log
        lp_ = os.path.join(run, "rtabmap_r1.log")
    inl, feat, nlog = read_log(lp_, n)
    rep["odometry_log_frames"] = nlog
    if nlog == 0:
        rep["notes"].append("rtabmap.log has no per-frame lines (run with --quiet?): odometry inliers n/a")
    else:
        rep["odometry_inliers_median"] = float(np.nanmedian(inl))
        rep["frames_low_inliers"] = int(np.sum(inl < LOW_INLIERS))
        rep["frames_zero_inliers"] = int(np.sum(inl <= 0))

    # windows
    W = a.window_s
    rows = []
    for w0 in np.arange(0, rel_all[-1] + 1e-9, W):
        sel = (rel_all >= w0) & (rel_all < w0 + W)
        if not sel.any():
            continue
        r = {"t0_s": round(float(w0), 1), "t1_s": round(float(w0 + W), 1), "frames": int(sel.sum()),
             "posed_pct": round(100.0 * posed[sel].mean(), 1),
             "lost_pct": round(100.0 * (1 - posed[sel].mean()), 1),
             "jumps": int(jump[sel].sum()),
             "resets": int(sum(w0 <= t < w0 + W for t in resets_t)),
             "loop_closures": int(sum((w0 <= lp["t_a_s"] < w0 + W) or (w0 <= lp["t_b_s"] < w0 + W) for lp in loops))}
        if nlog:
            r["inliers_median"] = float(np.nanmedian(inl[sel])) if np.isfinite(inl[sel]).any() else None
            r["inliers_min"] = float(np.nanmin(inl[sel])) if np.isfinite(inl[sel]).any() else None
            r["features_median"] = float(np.nanmedian(feat[sel])) if np.isfinite(feat[sel]).any() else None
            r["low_inlier_pct"] = round(100.0 * float(np.mean(inl[sel] < LOW_INLIERS)), 1)
        rows.append(r)
    rep["windows_s"] = W
    named = []
    for (t0, t1) in parse_ranges(a.windows):
        sel = (rel_all >= t0) & (rel_all < t1)
        if not sel.any():
            continue
        d = {"window": f"{t0:g}-{t1:g} s", "frames": int(sel.sum()), "lost_pct": round(100.0 * (1 - posed[sel].mean()), 1),
             "jumps": int(jump[sel].sum()), "resets": int(sum(t0 <= t < t1 for t in resets_t))}
        if nlog:
            d["low_inlier_pct"] = round(100.0 * float(np.mean(inl[sel] < LOW_INLIERS)), 1)
            d["inliers_median"] = float(np.nanmedian(inl[sel])) if np.isfinite(inl[sel]).any() else None
        named.append(d)
    rep["named_windows"] = named

    # path geometry
    rel_p = rel_all[pm]
    geo = None
    sel = None
    if pm.sum() > 50:
        g, aux = path_geometry(rel_p, P[pm], up, parse_ranges(a.turn_window)[0] if a.turn_window else None)
        rep["path"] = g
        geo = (g, aux) if aux is not None else None
        s0, s1 = parse_ranges(a.spin)[0]
        yc = yaw_change_deg(rel_all[pm], Pmap[pm], up, s0, s1)
        sel = (rel_all >= s0) & (rel_all <= s1)
        if not sel.any():
            sel = None
    if pm.sum() > 50 and sel is not None:
        spin_loops = [lp for lp in loops if (s0 - 5 <= lp["t_a_s"] <= s0 + 15) and (s1 - 15 <= lp["t_b_s"] <= s1 + 5)]
        rep["spin"] = {"window": f"{s0:g}-{s1:g} s", "turned_total_deg": None if yc is None else round(yc[0]),
                       "net_heading_change_deg": None if yc is None else round(yc[1]),
                       "lost_pct": round(100.0 * (1 - posed[sel].mean()), 1), "jumps": int(jump[sel].sum()),
                       "resets": int(sum(s0 <= t <= s1 for t in resets_t)),
                       "loop_closures_start_to_end": spin_loops}
    # map cleanliness: points below the floor (reflection 'ghosts')
    cp = os.path.join(run, "map_cloud.ply")
    mf = os.path.join(run, "map_frame.json")
    if os.path.isfile(cp) and os.path.isfile(mf):
        import open3d as o3d
        z = np.asarray(o3d.io.read_point_cloud(cp).points)[:, 2]
        fr = json.load(open(mf))
        rep["map"] = {"voxel_m": fr.get("voxel_m"), "depth_max_m": fr.get("depth_max_m"),
                      "points": int(len(z)), "below_floor_10cm_pct": round(100.0 * float(np.mean(z < -0.10)), 2),
                      "below_floor_30cm_pct": round(100.0 * float(np.mean(z < -0.30)), 2),
                      "floor": fr.get("floor")}

    od = os.path.join(run, "report")
    os.makedirs(od, exist_ok=True)
    with open(os.path.join(od, "windows.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    jump_t = list(rel_all[jump])
    try:
        path_plot(os.path.join(od, "path.png"), rel_p, P[pm], up, resets_t, jump_t,
                  [(lp["t_a_s"], lp["t_b_s"]) for lp in loops])
    except Exception as e:                                   # a picture must never stop the report
        rep["notes"].append(f"path plot failed: {e}")
    try:
        if geo is not None and os.path.isfile(tmp):
            corridor_views(run, os.path.join(od, "corridor_views.png"), geo, np.array([0, 0, 1.0]))
    except Exception as e:
        rep["notes"].append(f"corridor views failed: {e}")
    json.dump(rep, open(os.path.join(od, "report.json"), "w"), indent=2)
    write_md(os.path.join(od, "report.md"), rep, rows)
    return rep


def fmt(v, nd=2):
    if v is None:
        return "n/a"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def write_md(path, rep, rows):
    L = [f"# RTAB-Map report: {os.path.basename(rep['run'].rstrip('/'))}", ""]
    L += [f"- dataset: `{rep['dataset']}`, {rep['frames']} frames, {rep['duration_s']:.1f} s (t = 0 at the first colour frame)",
          f"- RTAB-Map: {rep.get('rtabmap_version')}; variant {rep.get('variant')}; runtime {rep.get('runtime_s')} s",
          f"- parameters: `{' '.join(rep.get('parameters') or [])}`",
          f"- posed frames: {rep['posed_frames']} ({rep['posed_pct']} %); frames without a node in the database: "
          f"{rep.get('frames_without_node', 'n/a')}",
          f"- odometry sessions: {len(rep['odometry_sessions'])} (resets: {rep['odometry_resets']})"
          + (f" at {', '.join(f'{t:.1f}' for t in [s['start_s'] for s in rep['odometry_sessions'][1:]][:40])} s" if rep['odometry_resets'] else ""),
          f"- jumps > {JUMP_M * 100:.0f} cm between consecutive frames: {rep['jumps']} (largest step {fmt(rep.get('largest_step_m'))} m)"]
    if rep.get("odometry_log_frames"):
        L.append(f"- odometry inliers: median {fmt(rep['odometry_inliers_median'], 0)}; frames under {LOW_INLIERS} inliers: "
                 f"{rep['frames_low_inliers']}; with 0: {rep['frames_zero_inliers']}")
    L.append(f"- links in the database: {rep.get('link_counts')}")
    for nt in rep["notes"]:
        L.append(f"- NOTE: {nt}")
    L += ["", "## Named windows", "", "| window | frames | lost % | low-inlier % | median inliers | jumps | resets |",
          "|---|---|---|---|---|---|---|"]
    for d in rep["named_windows"]:
        L.append(f"| {d['window']} | {d['frames']} | {d['lost_pct']} | {fmt(d.get('low_inlier_pct'), 1)} | "
                 f"{fmt(d.get('inliers_median'), 0)} | {d['jumps']} | {d['resets']} |")
    sp = rep.get("spin")
    if sp:
        L += ["", "## Stationary spin", "",
              f"{sp['window']}: turned {fmt(sp['turned_total_deg'])} deg in total (net {fmt(sp['net_heading_change_deg'])} deg), "
              f"lost {sp['lost_pct']} %, jumps {sp['jumps']}, resets {sp['resets']}, "
              f"loop closures between its start and its end: {len(sp['loop_closures_start_to_end'])}"
              + (" (" + ", ".join(f"{x['t_a_s']}<->{x['t_b_s']} s" for x in sp['loop_closures_start_to_end'][:10]) + ")"
                 if sp['loop_closures_start_to_end'] else "")]
    pa = rep.get("path")
    if pa:
        L += ["", "## Path", "",
              f"- total path {pa['path_length_m']:.1f} m (1 s smoothing, 0.5 s steps); start-to-end {pa['end_offset_m']:.1f} m; "
              f"height range {pa['height_range_m']:.2f} m"]
        tu = pa.get("turn")
        if tu:
            L += [f"- turn: centre {tu['time_s']:.0f} s (from {tu['start_s']:.0f} to {tu['end_s']:.0f} s), {tu['angle_deg']:.0f} deg",
                  f"- leg 1: {tu['leg1_path_m']:.1f} m along the path, {tu['leg1_straight_m']:.1f} m straight; "
                  f"leg 2: {tu['leg2_path_m']:.1f} m along the path, {tu['leg2_straight_m']:.1f} m straight",
                  "", "Positions (m) in a leg-aligned frame: A = along leg 1 (their z), B = horizontal towards leg 2 (their x), "
                  "H = height change; origin = first pose.", "", "| t (s) | A | B | H |", "|---|---|---|---|"]
            for t, (pA, pB, pH) in pa.get("positions_every_60s", {}).items():
                L.append(f"| {t} | {pA} | {pB} | {pH} |")
        else:
            L.append("- no clear turn found")
    mp = rep.get("map")
    if mp:
        L += ["", "## Map", "",
              f"- voxel {mp['voxel_m']} m, depth used up to {fmt(mp['depth_max_m'])} m (deeper depth is not fused), {mp['points']} points",
              f"- points more than 10 cm below the floor (reflection ghosts): {mp['below_floor_10cm_pct']} %; more than 30 cm: "
              f"{mp['below_floor_30cm_pct']} %",
              f"- floor check: {mp.get('floor')}"]
    L += ["", "## Loop closures (time pairs, s)", ""]
    lc = rep["loop_closures"]
    if not lc:
        L.append("none")
    for x in lc[:200]:
        L.append(f"- {x['t_a_s']} <-> {x['t_b_s']}  ({x['type']})")
    if len(lc) > 200:
        L.append(f"- ... {len(lc) - 200} more in report.json")
    L += ["", f"## Per {rep['windows_s']:g} s window", "", "| " + " | ".join(rows[0].keys()) + " |",
          "|" + "---|" * len(rows[0])]
    for r in rows:
        L.append("| " + " | ".join(fmt(v, 1) for v in r.values()) + " |")
    L += ["", "Pictures: path.png (top-down, coloured by time), corridor_views.png (fused map from eye height)."]
    open(path, "w").write("\n".join(L) + "\n")


def extra_params(r):
    p = list(r.get("parameters") or [])
    if "--Odom/ResetCountdown" in p:
        p = p[p.index("--Odom/ResetCountdown") + 2:]
    return " ".join(p) or "-"


def compare_md(path, reps):
    keys = [("variant", lambda r: r.get("variant")),
            ("parameters beyond R1", extra_params),
            ("posed %", lambda r: r["posed_pct"]),
            ("odometry resets", lambda r: r["odometry_resets"]),
            ("jumps > 10 cm", lambda r: r["jumps"]),
            ("largest step (m)", lambda r: r.get("largest_step_m")),
            ("frames < 20 inliers", lambda r: r.get("frames_low_inliers")),
            ("loop closures", lambda r: len(r["loop_closures"])),
            ("spin: loop closure start<->end", lambda r: len((r.get("spin") or {}).get("loop_closures_start_to_end", [])) if r.get("spin") else None),
            ("path length (m)", lambda r: round(r["path"]["path_length_m"], 1) if r.get("path") else None),
            ("turn at (s)", lambda r: round(r["path"]["turn"]["time_s"]) if r.get("path", {}).get("turn") else None),
            ("leg 1 / leg 2 straight (m)", lambda r: f"{r['path']['turn']['leg1_straight_m']:.1f} / {r['path']['turn']['leg2_straight_m']:.1f}"
             if r.get("path", {}).get("turn") else None),
            ("height range (m)", lambda r: round(r["path"]["height_range_m"], 2) if r.get("path") else None),
            ("below floor > 10 cm (%)", lambda r: (r.get("map") or {}).get("below_floor_10cm_pct")),
            ("runtime (s)", lambda r: r.get("runtime_s"))]
    names = [os.path.basename(r["run"].rstrip("/")) for r in reps]
    L = ["# RTAB-Map runs compared", "", "| | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for label, fn in keys:
        vals = []
        for r in reps:
            try:
                vals.append(fmt(fn(r), 2))
            except Exception:
                vals.append("n/a")
        L.append(f"| {label} | " + " | ".join(vals) + " |")
    L += ["", "Per-run details: each run's report/report.md, windows.csv, path.png, corridor_views.png."]
    open(path, "w").write("\n".join(L) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out")
    ap.add_argument("--window-s", type=float, default=5.0)
    ap.add_argument("--windows", default=DEFAULT_WINDOWS)
    ap.add_argument("--spin", default="340-387")
    ap.add_argument("--turn-window", default=None,
                    help='seconds, e.g. "150-260": look for the corner of the path only inside this window '
                         "(default: the sharpest turn of the whole recording, which can be a different turn)")
    a = ap.parse_args()
    ds = Dataset(a.dataset)
    reps = []
    for run in a.runs:
        run = os.path.abspath(run)
        if not os.path.isfile(os.path.join(run, "trajectory.tum")):
            print(f"skip {run}: no trajectory.tum")
            continue
        r = report_run(ds, run, a)
        reps.append(r)
        print(f"report: {run}/report/report.md  (posed {r['posed_pct']} %, resets {r['odometry_resets']}, "
              f"jumps {r['jumps']}, loop closures {len(r['loop_closures'])})")
    if a.out and reps:
        os.makedirs(a.out, exist_ok=True)
        compare_md(os.path.join(a.out, "rtabmap_comparison.md"), reps)
        print(f"comparison: {a.out}/rtabmap_comparison.md")
    return 0 if reps else 1


if __name__ == "__main__":
    sys.exit(main())
