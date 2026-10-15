#!/usr/bin/env python3
"""Inspect a librealsense .db3 recording without playing it back.

Reads the file as SQLite (read-only) and reports, per stream:
message count, rate, first/last device timestamp, timestamp domain,
frame-number gaps (frames lost while recording), image size/encoding,
plus intrinsics, extrinsics, depth units, compression and device info.
Ends with plain verdict lines (e.g. whether the IMU is full rate).

Needs only: python stdlib, zstandard, rosbags.

Usage:
  python bagtools/inspect_db3.py BAG.db3 [--out inspect.json]
"""
import argparse
import json
import os
import re
import sqlite3
import statistics
import sys
import time

import zstandard
from rosbags.typesys import Stores, get_typestore

ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"
STREAM_RE = re.compile(r"^/device_(\d+)/sensor_(\d+)/([A-Za-z]+)_(\d+)/(.+)$")
SENSOR_RE = re.compile(r"^/device_(\d+)/sensor_(\d+)/(info|option/.+)$")

TS = get_typestore(Stores.ROS2_HUMBLE)
DZ = zstandard.ZstdDecompressor()


def decode(blob, msgtype):
    raw = DZ.decompress(blob, max_output_size=1 << 30) if blob[:4] == ZSTD_MAGIC else blob
    return TS.deserialize_cdr(raw, msgtype)


def kv(text):
    """Parse 'a=1;b=2;' strings used by librealsense for metadata/info."""
    out = {}
    for part in text.split(";"):
        if "=" in part:
            k, v = part.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def percentile(sorted_vals, q):
    if not sorted_vals:
        return None
    i = min(len(sorted_vals) - 1, max(0, int(round(q * (len(sorted_vals) - 1)))))
    return sorted_vals[i]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--out", help="write JSON report here (default: <bag>.inspect.json next to the bag "
                                   "if writable, else ./<name>.inspect.json)")
    a = ap.parse_args()

    if not os.path.isfile(a.bag):
        sys.exit(f"ERROR: no such file: {a.bag}")
    t_start = time.time()
    db = sqlite3.connect(f"file:{os.path.abspath(a.bag)}?mode=ro", uri=True)
    tables = {r[0] for r in db.execute("select name from sqlite_master where type='table'")}
    if not {"topics", "messages"} <= tables:
        sys.exit(f"ERROR: not a rosbag2 sqlite file (tables: {sorted(tables)})")

    topics = {tid: (name, typ) for tid, name, typ in db.execute("select id, name, type from topics")}
    counts = dict(db.execute("select topic_id, count(*) from messages group by topic_id"))

    # compression check on a few messages
    sample = db.execute("select data from messages limit 50").fetchall()
    n_zstd = sum(1 for (b,) in sample if b[:4] == ZSTD_MAGIC)
    compression = "zstd (per message)" if n_zstd == len(sample) else ("none" if n_zstd == 0 else "mixed")

    def all_msgs(tid):
        return db.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid,))

    def first_string(name):
        for tid, (n, typ) in topics.items():
            if n == name and typ == "std_msgs/msg/String":
                row = db.execute("select data from messages where topic_id=? order by timestamp limit 1", (tid,)).fetchone()
                return decode(row[0], typ).data if row else None
        return None

    report = {"file": os.path.abspath(a.bag), "size_bytes": os.path.getsize(a.bag),
              "compression": compression, "streams": {}, "sensors": {}, "topics": {}}

    # file version / device info
    for tid, (name, typ) in topics.items():
        report["topics"][name] = {"type": typ, "count": counts.get(tid, 0)}
        if name == "/file_version" and counts.get(tid):
            row = db.execute("select data from messages where topic_id=?", (tid,)).fetchone()
            report["file_version"] = int(decode(row[0], typ).data)
    dev_info = first_string("/device_0/info")
    report["device_info"] = dev_info

    # sensors: names and options
    for tid, (name, typ) in topics.items():
        m = SENSOR_RE.match(name)
        if not m or typ != "std_msgs/msg/String":
            continue
        sensor = f"sensor_{m.group(2)}"
        entry = report["sensors"].setdefault(sensor, {"options": {}})
        if m.group(3) == "info":
            entry["info"] = first_string(name)
        elif m.group(3).endswith("/value"):
            opt = m.group(3)[len("option/"):-len("/value")]
            entry["options"][opt] = first_string(name)

    # streams
    stream_topics = {}
    for tid, (name, typ) in topics.items():
        m = STREAM_RE.match(name)
        if m:
            key = f"sensor_{m.group(2)}/{m.group(3)}_{m.group(4)}"
            stream_topics.setdefault(key, {})[m.group(5)] = (tid, typ)

    for key, parts in sorted(stream_topics.items()):
        s = {"kind": key.split("/")[1].rsplit("_", 1)[0]}
        for sub in ("info", "camera_info", "imu_intrinsic"):
            if sub in parts:
                s[sub] = first_string(f"/device_0/{key}/{sub}")
        s["tf"] = {sub: first_string(f"/device_0/{key}/{sub}") for sub in parts if sub.startswith("tf/")}
        data = parts.get("image/data") or parts.get("imu/data")
        meta = parts.get("image/metadata") or parts.get("imu/metadata")
        if data:
            s["data_messages"] = counts.get(data[0], 0)
            s["data_type"] = data[1]
        if data and data[1] == "sensor_msgs/msg/Image":
            row = db.execute("select data from messages where topic_id=? order by timestamp limit 1", (data[0],)).fetchone()
            if row:
                im = decode(row[0], data[1])
                s["image"] = {"width": im.width, "height": im.height, "encoding": im.encoding, "step": im.step}
        if meta:
            stamps, fnums, domains, sys_t = [], [], set(), []
            for _, blob in all_msgs(meta[0]):
                d = kv(decode(blob, meta[1]).data)
                if "timestamp" in d:
                    stamps.append(float(d["timestamp"]))
                if "Frame number" in d:
                    fnums.append(int(float(d["Frame number"])))
                if "timestamp_domain" in d:
                    domains.add(d["timestamp_domain"])
            s["metadata_messages"] = len(stamps)
            s["timestamp_domain"] = sorted(domains)
            if stamps:
                st = sorted(stamps)
                dts = sorted(b - c for b, c in zip(st[1:], st[:-1]))
                dur = (st[-1] - st[0]) / 1000.0
                s["device_time_ms"] = {"first": st[0], "last": st[-1]}
                s["duration_s"] = round(dur, 3)
                s["rate_hz"] = round((len(st) - 1) / dur, 2) if dur > 0 else None
                med = statistics.median(dts) if dts else None
                s["dt_ms"] = {"median": med, "p99": percentile(dts, 0.99), "max": dts[-1] if dts else None}
                s["non_increasing_timestamps"] = sum(1 for b, c in zip(stamps[1:], stamps[:-1]) if b <= c)
                s["duplicate_timestamps"] = len(stamps) - len(set(stamps))
                if med:
                    s["time_gaps_over_1.5x_median"] = sum(1 for d in dts if d > 1.5 * med)
            if fnums:
                fs = sorted(set(fnums))
                missing = (fs[-1] - fs[0] + 1) - len(fs)
                gaps = [(p, n) for p, n in zip(fs[:-1], fs[1:]) if n - p > 1]
                s["frame_numbers"] = {"first": fs[0], "last": fs[-1], "unique": len(fs),
                                      "missing": missing, "duplicates": len(fnums) - len(fs),
                                      "largest_gaps": sorted(gaps, key=lambda g: g[0] - g[1])[:10]}
        report["streams"][key] = s

    # verdicts
    v = []
    by_kind = {}
    for key, s in report["streams"].items():
        by_kind.setdefault(s["kind"], []).append((key, s))
    colour = by_kind.get("Color", [])
    depth = by_kind.get("Depth", [])
    gyro = by_kind.get("Gyro", [])
    accel = by_kind.get("Accel", [])
    cfps = colour[0][1].get("rate_hz") if colour else None
    for label, lst in (("colour", colour), ("depth", depth)):
        for key, s in lst:
            fn = s.get("frame_numbers", {})
            v.append(f"{label} ({key}): {s.get('data_messages', 0)} frames, {s.get('rate_hz')} Hz, "
                     f"{s.get('duration_s')} s, lost while recording: {fn.get('missing', '?')}, "
                     f"image {s.get('image', {}).get('width')}x{s.get('image', {}).get('height')} "
                     f"{s.get('image', {}).get('encoding')}")
    for label, lst in (("gyro", gyro), ("accel", accel)):
        for key, s in lst:
            r = s.get("rate_hz") or 0
            per_frame = (r / cfps) if cfps else None
            full = per_frame is not None and per_frame > 2.5
            v.append(f"{label} ({key}): {s.get('data_messages', 0)} samples, {r} Hz"
                     + (f", {per_frame:.1f} per colour frame -> {'FULL RATE' if full else 'THINNED (about 1 per frame)'}"
                        if per_frame else ""))
    if not gyro or not accel:
        v.append("IMU: MISSING (no gyro and/or accel stream in file)")
    else:
        full = all((s.get("rate_hz") or 0) / (cfps or 1) > 2.5 for _, s in gyro + accel)
        v.append(f"IMU full rate: {'YES' if full else 'NO'}")
    if colour and depth:
        c, d = colour[0][1], depth[0][1]
        if c.get("device_time_ms") and d.get("device_time_ms"):
            v.append("colour/depth start offset: "
                     f"{d['device_time_ms']['first'] - c['device_time_ms']['first']:.1f} ms")
        doms = set(c.get("timestamp_domain", [])) | set(d.get("timestamp_domain", []))
        v.append(f"timestamp domain(s): {sorted(doms)}")
    for sensor, e in report["sensors"].items():
        if "Depth_Units" in e.get("options", {}):
            v.append(f"depth units ({sensor}): {e['options']['Depth_Units']} m")
    v.append(f"compression: {compression}; file version: {report.get('file_version')}; "
             f"size: {report['size_bytes'] / 1e9:.2f} GB")
    report["verdicts"] = v
    report["inspect_seconds"] = round(time.time() - t_start, 1)

    out = a.out
    if not out:
        cand = os.path.splitext(os.path.abspath(a.bag))[0] + ".inspect.json"
        out = cand if os.access(os.path.dirname(cand), os.W_OK) else os.path.basename(cand)
    with open(out, "w") as fh:
        json.dump(report, fh, indent=2)

    print("=" * 70)
    print(f"File: {report['file']}")
    if dev_info:
        print(f"Device: {dev_info[:200]}")
    for key, s in report["streams"].items():
        print(f"- {key}: info[{s.get('info')}]")
        if s.get("camera_info"):
            print(f"    camera_info: {s['camera_info']}")
        for sub, txt in s.get("tf", {}).items():
            print(f"    {sub}: {txt}")
        if s.get("imu_intrinsic"):
            print(f"    imu_intrinsic: {s['imu_intrinsic'][:120]}...")
        if s.get("dt_ms"):
            print(f"    dt ms: median {s['dt_ms']['median']}, p99 {s['dt_ms']['p99']}, max {s['dt_ms']['max']}; "
                  f"non-increasing {s.get('non_increasing_timestamps')}, duplicates {s.get('duplicate_timestamps')}")
    print("-" * 70)
    for line in v:
        print("VERDICT", line)
    print(f"Report: {out}  ({report['inspect_seconds']} s)")


if __name__ == "__main__":
    main()
