#!/usr/bin/env python3
"""Pack RTAB-Map runs into one folder to hand to someone who compares them with another pipeline.

Per run (OUT/<run name>/):
  trajectory_optical.tum   TUM: timestamp x y z qx qy qz qw. One pose per colour frame that has one. Body frame =
                           colour camera OPTICAL frame (x right, y down, z forward); world frame = first posed
                           frame (identity). Timestamps = the colour frame's device time in seconds, the same
                           clock as the frames in the .db3 (frames.csv has the SDK frame number of each).
  trajectory_map.tum       same poses in the map frame: metres, +z up (gravity from the accelerometer), floor at
                           z = 0, first camera looking along +x
  per_frame.csv            every colour frame: index, SDK frame number, time, time since first frame, pose yes/no,
                           odometry inliers and features (from RTAB-Map's own per-frame output), step to previous
                           posed frame, jump flag
  odometry_resets.csv      every odometry loss: time of the first frame of the new session
  loop_closures.csv        every accepted link between non-consecutive frames, with times and type, and whether it
                           crosses the spin window
  windows_5s.csv           per 5 s: lost %, inliers, jumps, resets, loop closures
  parameters_passed.txt    exactly what we passed to RTAB-Map on top of its defaults (+ post-processing steps)
  parameters_db.txt        what rtabmap.db stores (rtabmap-info; --diff = only parameters different from default)
  map_cloud.ply            our fused map cloud (see README: Open3D TSDF built from these poses, NOT RTAB-Map's own)
  map_mesh.ply             (--with-mesh) same map as a mesh
  metrics.json map_frame.json run.json report.md
Top level: README.md (facts + how to read the files), frames.csv, calib.yaml, rtabmap_calib.yaml, meta.json,
rtabmap_comparison.md (if present).

Usage: python eval/rtabmap_pack.py DATASET RUN_DIR [RUN_DIR ...] --out DIR [--spin 340-387] [--with-mesh]
"""
import argparse
import csv
import json
import os
import shutil
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from mv2lib.dataset import Dataset  # noqa: E402
from mv2lib.traj import match_to_frames, read_tum, valid_mask  # noqa: E402
import rtabmap_report as rr  # noqa: E402


def find_tool(name):
    for d in (os.environ.get("RTABMAP_BIN"), os.path.join(os.environ.get("MV2_TOOLS", os.path.expanduser("~/mv2_tools")), "rtabmap", "bin")):
        if d and os.access(os.path.join(d, name), os.X_OK):
            return os.path.join(d, name)
    return shutil.which(name)


def db_parameters(db, out_path):
    """What the database stores: rtabmap-info (--diff first), else any 'parameters' column found by sqlite."""
    lines = []
    tool = find_tool("rtabmap-info")
    if tool:
        for flag, title in (("--diff", "rtabmap-info --diff (parameters that differ from RTAB-Map's defaults)"), (None, "rtabmap-info (full)")):
            try:
                r = subprocess.run([tool] + ([flag] if flag else []) + [db], capture_output=True, text=True, timeout=600)
                lines += [f"##### {title}  [exit {r.returncode}]", r.stdout.rstrip(), r.stderr.rstrip()[-1500:], ""]
            except Exception as e:
                lines += [f"##### {title}: failed: {e}", ""]
    else:
        lines.append("##### rtabmap-info not found (set RTABMAP_BIN)")
    try:
        import sqlite3
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        for (tbl,) in con.execute("SELECT name FROM sqlite_master WHERE type='table'"):
            cols = [r[1] for r in con.execute(f"PRAGMA table_info({tbl})")]
            for c in cols:
                if "param" in c.lower():
                    row = con.execute(f"SELECT {c} FROM {tbl} LIMIT 1").fetchone()
                    if row and row[0]:
                        v = row[0].decode(errors="replace") if isinstance(row[0], bytes) else str(row[0])
                        lines += [f"##### sqlite {tbl}.{c}", v.replace(";", "\n"), ""]
        con.close()
    except Exception as e:
        lines.append(f"##### sqlite read failed: {e}")
    open(out_path, "w").write("\n".join(lines) + "\n")


def pack_run(ds, run, out_root, a):
    name = os.path.basename(run.rstrip("/"))
    od = os.path.join(out_root, name)
    os.makedirs(od, exist_ok=True)
    n = len(ds)
    t0 = ds.stamps[0]
    rel = ds.stamps - t0
    fr = ds.frames_csv()
    fnum = {}
    for r in fr:                                   # frames.csv rows are the colour frames in the dataset
        fnum[r.get("name")] = r.get("colour_frame_number")
    nums = [fnum.get(nm, "") for nm in ds.names]

    # trajectories (copied as they are: one file per frame that has a pose)
    for src, dst in (("trajectory.tum", "trajectory_optical.tum"), ("trajectory_map.tum", "trajectory_map.tum")):
        if os.path.isfile(os.path.join(run, src)):
            shutil.copy2(os.path.join(run, src), os.path.join(od, dst))
    st, T = read_tum(os.path.join(run, "trajectory.tum"))
    poses, _ = match_to_frames(ds.stamps, st, T)
    posed = valid_mask(poses)
    P = poses[:, :3, 3]
    step = np.full(n, np.nan)
    both = posed[1:] & posed[:-1]
    step[1:][both] = np.linalg.norm(P[1:][both] - P[:-1][both], axis=1)

    lp = os.path.join(run, "rtabmap.log")
    if not os.path.isfile(lp):
        lp = os.path.join(run, "rtabmap_r1.log")
    inl, feat, nlog = rr.read_log(lp, n)
    with open(os.path.join(od, "per_frame.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["frame_index", "colour_frame_number", "t_s", "t_since_first_frame_s", "posed", "odom_inliers",
                    "odom_features", "step_to_previous_posed_m", "jump_over_10cm"])
        for i in range(n):
            w.writerow([i, nums[i], f"{ds.stamps[i]:.6f}", f"{rel[i]:.3f}", int(posed[i]),
                        "" if np.isnan(inl[i]) else int(inl[i]), "" if np.isnan(feat[i]) else int(feat[i]),
                        "" if np.isnan(step[i]) else f"{step[i]:.4f}", int(np.nan_to_num(step[i], nan=0) > rr.JUMP_M)])

    # database: resets and loop closures
    db = rr.read_db(os.path.join(run, "rtabmap.db"))
    s0, s1 = rr.parse_ranges(a.spin)[0]
    node_t = {nid: float(s) for nid, _, s in db["nodes"]}
    sessions = []
    for nid, mid, s in db["nodes"]:
        if not sessions or sessions[-1][0] != mid:
            sessions.append((mid, float(s)))
    with open(os.path.join(od, "odometry_resets.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["reset_number", "new_session_map_id", "t_s", "t_since_first_frame_s", "in_spin_window"])
        for k, (mid, s) in enumerate(sessions[1:], 1):
            w.writerow([k, mid, f"{s:.6f}", f"{s - t0:.3f}", int(s0 <= s - t0 <= s1)])
    seen, rows = set(), []
    for f_, t_, ty in db["links"]:
        if ty not in rr.LOOP_TYPES or f_ not in node_t or t_ not in node_t:
            continue
        key = (min(f_, t_), max(f_, t_), ty)
        if key in seen:
            continue
        seen.add(key)
        ta, tb = sorted((node_t[f_] - t0, node_t[t_] - t0))
        rows.append((ta, tb, rr.LINK_TYPES.get(ty, str(ty))))
    rows.sort(key=lambda r: (r[1], r[0]))
    cross = 0
    with open(os.path.join(od, "loop_closures.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_a_since_first_frame_s", "t_b_since_first_frame_s", "type", "crosses_spin_window"])
        for ta, tb, ty in rows:
            c = int(ta <= s0 + 5 and tb >= s1 - 5)
            cross += c
            w.writerow([f"{ta:.2f}", f"{tb:.2f}", ty, c])
    kf = inter = None
    try:
        import sqlite3
        con = sqlite3.connect(f"file:{os.path.join(run, 'rtabmap.db')}?mode=ro", uri=True)
        kf = con.execute("SELECT COUNT(*) FROM Node WHERE weight >= 0").fetchone()[0]
        inter = con.execute("SELECT COUNT(*) FROM Node WHERE weight < 0").fetchone()[0]
        con.close()
    except Exception:
        pass

    for f in ("report/windows.csv",):
        if os.path.isfile(os.path.join(run, f)):
            shutil.copy2(os.path.join(run, f), os.path.join(od, "windows_5s.csv"))
    for f in ("metrics.json", "map_frame.json", "run.json", "map_cloud.ply", "rtabmap_info.txt", "views.png"):
        if os.path.isfile(os.path.join(run, f)):
            shutil.copy2(os.path.join(run, f), os.path.join(od, f))
    for f in ("path.png", "corridor_views.png", "report.md", "report.json"):
        if os.path.isfile(os.path.join(run, "report", f)):
            shutil.copy2(os.path.join(run, "report", f), os.path.join(od, f))
    if a.with_mesh and os.path.isfile(os.path.join(run, "map_mesh.ply")):
        shutil.copy2(os.path.join(run, "map_mesh.ply"), os.path.join(od, "map_mesh.ply"))
    rj = json.load(open(os.path.join(run, "run.json"))) if os.path.isfile(os.path.join(run, "run.json")) else {}
    with open(os.path.join(od, "parameters_passed.txt"), "w") as fh:
        fh.write("Everything else is RTAB-Map 0.24.2's default (see parameters_db.txt for the stored values).\n\n")
        p = rj.get("parameters") or []
        i = 0
        while i < len(p):
            if p[i].startswith("--") and i + 1 < len(p) and not p[i + 1].startswith(("--", "+")):
                fh.write(f"{p[i][2:]} {p[i + 1]}\n")
                i += 2
            else:
                fh.write(f"{p[i]}\n")
                i += 1
    if os.path.isfile(os.path.join(run, "rtabmap.db")):
        db_parameters(os.path.join(run, "rtabmap.db"), os.path.join(od, "parameters_db.txt"))
    return {"name": name, "posed": int(posed.sum()), "frames": n, "keyframes": kf, "intermediate": inter,
            "resets": len(sessions) - 1, "loops": len(rows), "loops_crossing_spin": cross, "nlog": nlog}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--spin", default="340-387")
    ap.add_argument("--with-mesh", action="store_true")
    a = ap.parse_args()
    ds = Dataset(a.dataset)
    os.makedirs(a.out, exist_ok=True)
    info = []
    for run in a.runs:
        run = os.path.abspath(run)
        if not os.path.isfile(os.path.join(run, "trajectory.tum")):
            print(f"skip {run}: no trajectory.tum")
            continue
        r = pack_run(ds, run, a.out, a)
        info.append(r)
        print(f"packed {r['name']}: {r['posed']}/{r['frames']} posed, {r['resets']} resets, {r['loops']} loop closures "
              f"({r['loops_crossing_spin']} cross the spin window), nodes: {r['keyframes']} keyframes + {r['intermediate']} intermediate")
    for f in ("frames.csv", "calib.yaml", "rtabmap_calib.yaml", "meta.json"):
        if os.path.isfile(os.path.join(ds.root, f)):
            shutil.copy2(os.path.join(ds.root, f), os.path.join(a.out, f))
    json.dump(info, open(os.path.join(a.out, "pack_info.json"), "w"), indent=2)
    return 0 if info else 1


if __name__ == "__main__":
    sys.exit(main())
