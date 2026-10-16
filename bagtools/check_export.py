#!/usr/bin/env python3
"""Check an exported dataset against its source .db3, independently of the exporter.

The .db3 is re-read as SQLite (not through the SDK) and compared with the export:
 - every colour frame in the file appears in frames.csv (same device timestamps)
 - every written frame has rgb + depth PNGs of the right type and size
 - each pairing decision matches an independent nearest-in-time search
 - gyro/accel sample counts equal the IMU messages in the file
 - timestamps strictly increase, names are unique, symlinks resolve
Also writes a contact sheet (colour | depth for ~24 frames spread over the walk)
and export_report.json. Exit code 0 = PASS, 1 = FAIL.

Usage:
  python bagtools/check_export.py BAG.db3 DATASET_DIR [--lock]
    --lock   on PASS, make the dataset read-only (chmod -R a-w)
"""
import argparse
import bisect
import csv
import json
import os
import sqlite3
import stat
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from inspect_db3 import STREAM_RE, decode, kv  # noqa: E402


def stream_stamps(db, kind):
    """Device timestamps (ms) of one stream kind ('Color', 'Depth', 'Gyro', 'Accel') from metadata."""
    out = []
    for tid, name, typ in db.execute("select id, name, type from topics"):
        m = STREAM_RE.match(name)
        if not m or m.group(3) != kind or m.group(5) not in ("image/metadata", "imu/metadata"):
            continue
        for (blob,) in db.execute("select data from messages where topic_id=? order by timestamp", (tid,)):
            d = kv(decode(blob, typ).data)
            out.append(float(d["timestamp"]))
        break
    return sorted(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("dataset")
    ap.add_argument("--lock", action="store_true")
    a = ap.parse_args()
    ds = a.dataset
    results = []

    def check(name, ok, detail=""):
        results.append({"check": name, "ok": bool(ok), "detail": detail})
        print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))

    meta = json.load(open(os.path.join(ds, "meta.json")))
    rows = list(csv.DictReader(open(os.path.join(ds, "frames.csv"))))
    limited = meta.get("max_frames") is not None

    db = sqlite3.connect(f"file:{os.path.abspath(a.bag)}?mode=ro", uri=True)
    c_ms = stream_stamps(db, "Color")
    d_ms = stream_stamps(db, "Depth")
    g_ms = stream_stamps(db, "Gyro")
    acc_ms = stream_stamps(db, "Accel")

    # 1. every colour frame accounted for
    exp_c = [round(float(r["t_colour_s"]) * 1000.0, 3) for r in rows]
    src_c = [round(t, 3) for t in c_ms]
    if limited:
        src_c = src_c[: meta["max_frames"]]
    # frames.csv stores seconds with 6 decimals (0.001 ms), so compare to that resolution, not bit for bit
    same_n = len(exp_c) == len(src_c)
    max_diff = float(np.max(np.abs(np.array(exp_c) - np.array(src_c)))) if same_n and exp_c else float("nan")
    check("colour frames: export == file", same_n and max_diff <= 0.0015,
          f"file {len(src_c)}, frames.csv {len(exp_c)}, largest time difference {max_diff:.4f} ms"
          + (" (limited run)" if limited else ""))

    # 2. timestamps / names
    ts = [float(r["t_colour_s"]) for r in rows]
    check("colour timestamps strictly increasing", all(b > c for b, c in zip(ts[1:], ts[:-1])))
    names = [r["name"] for r in rows]
    check("frame names unique", len(names) == len(set(names)))

    # 3. pairing re-done independently
    half = meta["pairing_threshold_ms"]
    mism = 0
    for r in rows:
        tc = float(r["t_colour_s"]) * 1000.0
        i = bisect.bisect_left(d_ms, tc)
        cands = [d_ms[j] for j in (i - 1, i) if 0 <= j < len(d_ms)]
        best = min(cands, key=lambda t: abs(t - tc)) if cands else None
        should_pair = best is not None and abs(best - tc) <= half
        if should_pair != (r["flag"] != "no_depth"):
            mism += 1
        elif should_pair and abs(float(r["t_depth_s"]) * 1000.0 - best) > 0.002:
            mism += 1
    n_paired = sum(1 for r in rows if r["flag"] != "no_depth")
    check("pairing matches independent nearest-time search", mism == 0,
          f"{mism} mismatches; paired {n_paired}, no_depth {len(rows) - n_paired}, threshold {half:.2f} ms")

    # 4. files
    missing, bad = 0, 0
    w, h = None, None
    sample_every = max(1, n_paired // 200)
    for k, r in enumerate(x for x in rows if x["flag"] != "no_depth"):
        pr = os.path.join(ds, "rgb", r["name"] + ".png")
        pd = os.path.join(ds, "depth", r["name"] + ".png")
        if not (os.path.isfile(pr) and os.path.isfile(pd)) or os.path.getsize(pr) == 0 or os.path.getsize(pd) == 0:
            missing += 1
            continue
        if k % sample_every == 0:
            ci = cv2.imread(pr, cv2.IMREAD_UNCHANGED)
            di = cv2.imread(pd, cv2.IMREAD_UNCHANGED)
            if w is None and ci is not None:
                h, w = ci.shape[:2]
            if ci is None or di is None or ci.dtype != np.uint8 or ci.ndim != 3 or di.dtype != np.uint16 \
                    or di.shape != ci.shape[:2] or (w and ci.shape[:2] != (h, w)):
                bad += 1
    n_rgb = len([f for f in os.listdir(os.path.join(ds, "rgb")) if f.endswith(".png")])
    n_dep = len([f for f in os.listdir(os.path.join(ds, "depth")) if f.endswith(".png")])
    check("rgb/ and depth/ hold exactly the paired frames", n_rgb == n_dep == n_paired and missing == 0,
          f"rgb {n_rgb}, depth {n_dep}, paired {n_paired}, missing {missing}")
    check("sampled PNGs have the right type and size", bad == 0, f"{bad} bad of ~{n_paired // sample_every} read")
    lists = {f: sum(1 for line in open(os.path.join(ds, f)) if not line.startswith("#"))
             for f in ("rgb.txt", "depth.txt", "associations.txt")}
    check("TUM lists complete", all(v == n_paired for v in lists.values()), str(lists))
    check("rgb_sync/depth_sync symlinks resolve",
          all(os.path.isdir(os.path.join(ds, s)) for s in ("rgb_sync", "depth_sync")))

    # 5. IMU
    if not limited:
        margin = float(meta.get("imu_margin_ms", 30000.0))
        lo, hi = src_c[0] - margin, src_c[-1] + margin
        for nm, st, key in (("gyro", g_ms, "gyro_samples"), ("accel", acc_ms, "accel_samples")):
            expect = sum(1 for t in st if lo <= t <= hi)
            check(f"{nm} samples: export == file", meta[key] == expect,
                  f"file {len(st)}, inside the colour time range {expect}, export {meta[key]}, "
                  f"dropped {len(st) - meta[key]}")
    for fname in ("imu_gyro.csv", "imu_accel.csv"):
        t = np.loadtxt(os.path.join(ds, fname), delimiter=",", skiprows=1, usecols=0, ndmin=1)
        check(f"{fname} times non-decreasing", t.size == 0 or bool(np.all(np.diff(t) >= 0)), f"{t.size} rows")

    # 6. quality summary (reported, not pass/fail)
    def col(k):
        return np.array([float(r[k]) for r in rows if r[k] != ""])
    blur, bright, valid, dt = col("blur_lapvar"), col("brightness"), col("valid_depth_frac"), col("dt_depth_minus_colour_ms")
    quality = {
        "blur_lapvar_p5_p50_p95": np.percentile(blur, [5, 50, 95]).round(1).tolist() if blur.size else None,
        "frames_blur_below_quarter_median": int((blur < 0.25 * np.median(blur)).sum()) if blur.size else None,
        "brightness_p5_p50_p95": np.percentile(bright, [5, 50, 95]).round(1).tolist() if bright.size else None,
        "valid_depth_p5_p50_p95": np.percentile(valid, [5, 50, 95]).round(3).tolist() if valid.size else None,
        "dt_depth_colour_ms_p5_p50_p95": np.percentile(dt, [5, 50, 95]).round(2).tolist() if dt.size else None,
        "colour_frames_lost_while_recording": int(round((c_ms[-1] - c_ms[0]) * meta.get("fps", 30) / 1000.0)) + 1 - len(c_ms)
        if len(c_ms) > 1 and not limited else None,
    }
    print("QUALITY", json.dumps(quality))

    # 7. contact sheet
    paired = [r for r in rows if r["flag"] != "no_depth"]
    if paired:
        n = min(24, len(paired))
        idx = np.linspace(0, len(paired) - 1, n).round().astype(int)
        tiles = []
        t0 = float(rows[0]["t_colour_s"])
        for i in idx:
            r = paired[i]
            ci = cv2.imread(os.path.join(ds, "rgb", r["name"] + ".png"))
            di = cv2.imread(os.path.join(ds, "depth", r["name"] + ".png"), cv2.IMREAD_UNCHANGED)
            ci = cv2.resize(ci, (320, 240))
            dm = di.astype(np.float32) / 5000.0  # metres
            dv = np.clip(dm / 5.0 * 255.0, 0, 255).astype(np.uint8)
            dc = cv2.applyColorMap(dv, cv2.COLORMAP_TURBO)
            dc[di == 0] = 0
            dc = cv2.resize(dc, (320, 240), interpolation=cv2.INTER_NEAREST)
            tile = np.hstack([ci, dc])
            label = f"#{rows.index(r)}  t+{float(r['t_colour_s']) - t0:.1f}s  blur {r['blur_lapvar']}"
            cv2.putText(tile, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(tile, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            tiles.append(tile)
        while len(tiles) % 3:
            tiles.append(np.zeros_like(tiles[0]))
        grid = np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)])
        cv2.imwrite(os.path.join(ds, "contact_sheet.jpg"), grid, [cv2.IMWRITE_JPEG_QUALITY, 85])
        print(f"Contact sheet: {os.path.join(ds, 'contact_sheet.jpg')}  (left colour, right depth 0-5 m)")

    ok = all(r["ok"] for r in results)
    report = {"pass": ok, "checks": results, "quality": quality, "warnings": meta.get("warnings", [])}
    with open(os.path.join(ds, "export_report.json"), "w") as fh:
        json.dump(report, fh, indent=2)
    for wmsg in meta.get("warnings", []):
        print("WARNING (from export):", wmsg)
    print("RESULT:", "PASS" if ok else "FAIL")

    if ok and a.lock:
        for root, dirs, files in os.walk(ds):
            for f in files:
                p = os.path.join(root, f)
                if not os.path.islink(p):
                    os.chmod(p, os.stat(p).st_mode & ~(stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH))
        for root, dirs, files in os.walk(ds, topdown=False):
            for d in dirs:
                p = os.path.join(root, d)
                if not os.path.islink(p):
                    os.chmod(p, os.stat(p).st_mode & ~(stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH))
        os.chmod(ds, os.stat(ds).st_mode & ~(stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH))
        print("Dataset is now read-only. (Undo with: chmod -R u+w", ds + ")")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
