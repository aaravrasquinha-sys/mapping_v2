#!/usr/bin/env python3
"""Export a librealsense .db3 recording to a plain RGB-D dataset folder.

How it reads the file (each point was tested on files made with this SDK):
- Sensor-level playback in NON-real-time mode: every frame and every IMU sample
  is delivered, even when this script is slow. The pipeline/syncer is NOT used:
  it repeats frames, mispairs colour/depth and thins the IMU to ~1 sample/frame.
- Each colour frame is paired with the depth frame nearest in device time.
  If none is within half a frame period, the colour frame is flagged
  ("no_depth") in frames.csv and not written to rgb/ (so rgb/ and depth/ match).
- Depth is aligned to colour by the SDK's own rs.align on that pair.

Output folder:
  rgb/<t>.png      colour, 8-bit (t = colour device time in seconds)
  depth/<t>.png    uint16, 5000 units per metre (TUM convention), aligned to colour
  rgb_sync, depth_sync   symlinks to rgb/, depth/ (for rtabmap-rgbd_dataset)
  rgb.txt depth.txt associations.txt   TUM lists
  frames.csv       per colour frame: pairing, time gap, blur, brightness, valid depth
  imu_gyro.csv imu_accel.csv   every sample (device time, seconds)
  imu.csv          gyro times + linearly interpolated accel (EuRoC-style, ns)
  calib.yaml       intrinsics, extrinsics, depth scale, read from the file
  intrinsic.json   Open3D camera intrinsic
  rtabmap_calib.yaml   RTAB-Map calibration (incl. optical local_transform)
  meta.json        versions, counts, pairing statistics, warnings

Usage:
  python bagtools/export_db3.py BAG.db3 OUT_DIR [--max-frames N] [--force]
"""
import argparse
import csv
import datetime
import hashlib
import json
import os
import platform
import queue
import shutil
import sys
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml

TUM_UNITS_PER_M = 5000
TUM_MAX_RAW = 65535


# ----------------------------------------------------------------------------- helpers

def intr_dict(i):
    return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy, "cx": i.ppx, "cy": i.ppy,
            "model": str(i.model).split(".")[-1], "coeffs": [float(c) for c in i.coeffs]}


def extr_matrix(e):
    """librealsense extrinsics (column-major rotation) -> 4x4 list, p_to = T @ p_from."""
    T = np.eye(4)
    T[:3, :3] = np.array(e.rotation, dtype=float).reshape(3, 3).T
    T[:3, 3] = np.array(e.translation, dtype=float)
    return T.round(9).tolist()


def stamp_name(t_ms):
    return f"{t_ms / 1000.0:017.6f}"


def colour_to_bgr(frame):
    fmt = frame.get_profile().format()
    img = np.asanyarray(frame.get_data())
    if fmt == rs.format.rgb8:
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if fmt == rs.format.bgr8:
        return img.copy()
    if fmt == rs.format.rgba8:
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if fmt == rs.format.bgra8:
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if fmt == rs.format.yuyv:
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
    raise RuntimeError(f"unsupported colour format {fmt}")


def depth_to_tum(z16, depth_scale):
    z = z16.astype(np.uint32)
    if abs(depth_scale - 0.001) < 1e-12:
        out = z * 5                                    # exact: 1 mm -> 5 units
    else:
        out = np.round(z.astype(np.float64) * depth_scale * TUM_UNITS_PER_M).astype(np.uint32)
    out[out > TUM_MAX_RAW] = 0                          # beyond 13.1 m: invalid
    return out.astype(np.uint16)


def sha256_head(path, n=64 << 20):
    """sha256 of the first 64 MB + size: cheap identity of the source file."""
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        h.update(fh.read(n))
    h.update(str(os.path.getsize(path)).encode())
    return h.hexdigest()


# ----------------------------------------------------------------------------- exporter

class Exporter:
    def __init__(self, bag, out, max_frames=None):
        self.bag, self.out, self.max_frames = bag, out, max_frames
        self.lock = threading.Lock()
        self.pending_colour = []      # kept colour frames waiting for a depth decision
        self.depth_buf = []           # kept depth frames, increasing time
        self.latest_depth_ms = -1e18
        self.depth_done = False
        self.n_colour_in = 0
        self.n_depth_in = 0
        self.gyro, self.accel = [], []
        self.work = queue.Queue(maxsize=8)
        self.rows = []
        self.errors = []
        self.half_period_ms = None
        self.depth_scale = None
        self.last_event = time.time()
        self.limit_reached = False

    # -- pairing (called with self.lock held) ---------------------------------------
    def _resolve(self, final=False):
        ready = []
        while self.pending_colour:
            c = self.pending_colour[0]
            tc = c.get_timestamp()
            if not (final or self.depth_done or self.latest_depth_ms >= tc):
                break
            self.pending_colour.pop(0)
            best, best_dt = None, None
            for d in self.depth_buf:
                dt = d.get_timestamp() - tc
                if best is None or abs(dt) < abs(best_dt):
                    best, best_dt = d, dt
            if best is not None and abs(best_dt) <= self.half_period_ms:
                ready.append((c, best, best_dt))
            else:
                ready.append((c, None, best_dt))
            # drop depth frames that can no longer be within half a period of a later colour frame
            keep_from = tc - 2 * self.half_period_ms
            self.depth_buf = [d for d in self.depth_buf if d.get_timestamp() >= keep_from]
        return ready

    def _push(self, items):
        for it in items:
            self.work.put(it)          # blocks when the writer is behind -> back-pressure

    # -- callbacks -------------------------------------------------------------------
    def on_colour(self, f):
        self.last_event = time.time()
        with self.lock:
            if self.max_frames and self.n_colour_in >= self.max_frames:
                self.limit_reached = True
                return
            self.n_colour_in += 1
            f.keep()
            self.pending_colour.append(f)
            ready = self._resolve()
        self._push(ready)

    def on_depth(self, f):
        self.last_event = time.time()
        with self.lock:
            self.n_depth_in += 1
            f.keep()
            self.depth_buf.append(f)
            self.latest_depth_ms = max(self.latest_depth_ms, f.get_timestamp())
            ready = self._resolve()
        self._push(ready)

    def on_motion(self, f):
        self.last_event = time.time()
        m = f.as_motion_frame().get_motion_data()
        rec = (f.get_timestamp(), f.get_frame_number(), m.x, m.y, m.z)
        if f.get_profile().stream_type() == rs.stream.gyro:
            self.gyro.append(rec)
        else:
            self.accel.append(rec)

    # -- writer thread -----------------------------------------------------------------
    def writer(self):
        align = rs.align(rs.stream.color)
        q = rs.frame_queue(4, keep_frames=True)
        pair = {}

        def make_composite(f, src):
            src.frame_ready(src.allocate_composite_frame([pair["c"], pair["d"]]))

        blk = rs.processing_block(make_composite)
        blk.start(q)
        while True:
            item = self.work.get()
            if item is None:
                break
            c, d, dt = item
            try:
                self._write_one(c, d, dt, align, blk, q, pair)
            except Exception as e:  # keep going, report at the end
                self.errors.append(f"frame {c.get_frame_number()}: {e!r}")

    def _write_one(self, c, d, dt, align, blk, q, pair):
        tc = c.get_timestamp()
        name = stamp_name(tc)
        row = {"name": name, "t_colour_s": f"{tc / 1000.0:.6f}", "colour_frame_number": c.get_frame_number(),
               "t_depth_s": "", "depth_frame_number": "", "dt_depth_minus_colour_ms": "",
               "flag": "", "blur_lapvar": "", "brightness": "", "valid_depth_frac": ""}
        bgr = colour_to_bgr(c)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        row["blur_lapvar"] = f"{cv2.Laplacian(gray, cv2.CV_64F).var():.1f}"
        row["brightness"] = f"{gray.mean():.1f}"
        if d is None:
            row["flag"] = "no_depth"
            if dt is not None:
                row["dt_depth_minus_colour_ms"] = f"{dt:.3f}"
            self.rows.append(row)
            return
        pair["c"], pair["d"] = c, d
        blk.invoke(c)
        fs = q.wait_for_frame(5000).as_frameset()
        aligned = align.process(fs)
        ad = aligned.get_depth_frame()
        ac = aligned.get_color_frame()
        if not ad or not ac or ac.get_frame_number() != c.get_frame_number():
            raise RuntimeError("align returned an unexpected frameset")
        z = np.asanyarray(ad.get_data())
        if z.shape != gray.shape:
            raise RuntimeError(f"aligned depth shape {z.shape} != colour {gray.shape}")
        tum = depth_to_tum(z, self.depth_scale)
        ok1 = cv2.imwrite(os.path.join(self.out, "rgb", name + ".png"), bgr, [cv2.IMWRITE_PNG_COMPRESSION, 3])
        ok2 = cv2.imwrite(os.path.join(self.out, "depth", name + ".png"), tum, [cv2.IMWRITE_PNG_COMPRESSION, 3])
        if not (ok1 and ok2):
            raise RuntimeError("PNG write failed (disk full?)")
        row.update({"t_depth_s": f"{d.get_timestamp() / 1000.0:.6f}", "depth_frame_number": d.get_frame_number(),
                    "dt_depth_minus_colour_ms": f"{dt:.3f}", "valid_depth_frac": f"{(z > 0).mean():.4f}"})
        self.rows.append(row)

    # -- main --------------------------------------------------------------------------
    def run(self):
        t_start = time.time()
        os.makedirs(os.path.join(self.out, "rgb"), exist_ok=True)
        os.makedirs(os.path.join(self.out, "depth"), exist_ok=True)

        ctx = rs.context()
        dev = ctx.load_device(self.bag)
        pb = dev.as_playback()
        pb.set_real_time(False)

        prof = {"colour": None, "depth": None, "gyro": None, "accel": None}
        owner = {}
        for s in pb.query_sensors():
            for p in s.get_stream_profiles():
                st = p.stream_type()
                key = {rs.stream.color: "colour", rs.stream.depth: "depth",
                       rs.stream.gyro: "gyro", rs.stream.accel: "accel"}.get(st)
                if key and prof[key] is None:
                    prof[key] = p
                    owner[key] = s
        for k in ("colour", "depth"):
            if prof[k] is None:
                sys.exit(f"ERROR: no {k} stream in {self.bag}")
        cprof = prof["colour"].as_video_stream_profile()
        dprof = prof["depth"].as_video_stream_profile()
        self.half_period_ms = 0.5 * 1000.0 / cprof.fps()

        depth_sensor = owner["depth"]
        try:
            self.depth_scale = depth_sensor.as_depth_sensor().get_depth_scale()
        except Exception:
            self.depth_scale = depth_sensor.get_option(rs.option.depth_units)
        if not self.depth_scale or self.depth_scale <= 0:
            sys.exit("ERROR: could not read the depth scale from the file")

        calib = {
            "colour": intr_dict(cprof.get_intrinsics()),
            "depth_raw": intr_dict(dprof.get_intrinsics()),
            "aligned_depth_uses_colour_intrinsics": True,
            "T_colour_from_depth": extr_matrix(dprof.get_extrinsics_to(cprof)),
            "depth_scale_m_per_unit_raw": float(self.depth_scale),
            "depth_png_units_per_m": TUM_UNITS_PER_M,
            "fps": cprof.fps(),
        }
        for opt, key in ((rs.option.stereo_baseline, "stereo_baseline_mm"),):
            try:
                if depth_sensor.supports(opt):
                    calib[key] = float(depth_sensor.get_option(opt))
            except Exception:
                pass
        for k in ("gyro", "accel"):
            if prof[k] is not None:
                mp = prof[k].as_motion_stream_profile()
                calib[f"{k}_rate_hz"] = mp.fps()
                try:
                    calib[f"T_colour_from_{k}"] = extr_matrix(prof[k].get_extrinsics_to(cprof))
                except Exception as e:
                    calib[f"T_colour_from_{k}"] = None
                    self.errors.append(f"no {k}->colour extrinsics in file: {e!r}")
                try:
                    mi = mp.get_motion_intrinsics()
                    calib[f"{k}_intrinsics"] = {"data": np.array(mi.data).tolist(),
                                                "noise_variances": list(mi.noise_variances),
                                                "bias_variances": list(mi.bias_variances)}
                except Exception:
                    pass
        try:
            calib["device"] = {"name": dev.get_info(rs.camera_info.name),
                               "serial": dev.get_info(rs.camera_info.serial_number),
                               "firmware": dev.get_info(rs.camera_info.firmware_version)}
        except Exception:
            pass

        writer = threading.Thread(target=self.writer, daemon=True)
        writer.start()

        # open everything first, then start all, then resume: no sensor misses the start
        by_sensor = {}
        for k, p in prof.items():
            if p is not None:
                by_sensor.setdefault(owner[k], []).append(p)
        status = []
        pb.set_status_changed_callback(lambda st: status.append(st))
        pb.pause()
        for s, plist in by_sensor.items():
            s.open(plist)
        for s, plist in by_sensor.items():
            kinds = {p.stream_type() for p in plist}
            if rs.stream.gyro in kinds or rs.stream.accel in kinds:
                if kinds & {rs.stream.color, rs.stream.depth}:
                    sys.exit("ERROR: motion and video streams on one sensor: not supported")
                s.start(self.on_motion)
            elif rs.stream.color in kinds and rs.stream.depth in kinds:
                s.start(lambda f: self.on_colour(f) if f.get_profile().stream_type() == rs.stream.color
                        else self.on_depth(f))
            elif rs.stream.color in kinds:
                s.start(self.on_colour)
            else:
                s.start(self.on_depth)
        pb.resume()

        duration_s = pb.get_duration().total_seconds() if hasattr(pb.get_duration(), "total_seconds") else None
        last_print = 0
        while True:
            time.sleep(0.2)
            stopped = pb.current_status() == rs.playback_status.stopped
            quiet = time.time() - self.last_event > 2.0
            if self.limit_reached and self.work.empty():
                break
            if stopped and quiet:
                break
            if time.time() - last_print > 10:
                last_print = time.time()
                pos = pb.get_position() / 1e9
                print(f"  colour in {self.n_colour_in}, depth in {self.n_depth_in}, written {len(self.rows)}, "
                      f"gyro {len(self.gyro)}, accel {len(self.accel)}, file position {pos:.0f}"
                      + (f"/{duration_s:.0f} s" if duration_s else " s"), flush=True)

        with self.lock:
            self.depth_done = True
            ready = self._resolve(final=True)
        self._push(ready)
        self.work.put(None)
        writer.join()
        for s in by_sensor:
            try:
                s.stop()
            except Exception:
                pass
            try:
                s.close()
            except Exception:
                pass

        self.finish(calib, t_start)

    # -- outputs ---------------------------------------------------------------------
    def finish(self, calib, t_start):
        out = self.out
        rows = sorted(self.rows, key=lambda r: float(r["t_colour_s"]))
        paired = [r for r in rows if r["flag"] != "no_depth"]
        names = [r["name"] for r in rows]
        dup_names = len(names) - len(set(names))
        depth_use = {}
        for r in paired:
            depth_use[r["depth_frame_number"]] = depth_use.get(r["depth_frame_number"], 0) + 1
        for r in paired:
            if depth_use[r["depth_frame_number"]] > 1:
                r["flag"] = "depth_reused"

        fields = ["name", "t_colour_s", "colour_frame_number", "t_depth_s", "depth_frame_number",
                  "dt_depth_minus_colour_ms", "flag", "blur_lapvar", "brightness", "valid_depth_frac"]
        with open(os.path.join(out, "frames.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)

        hdr = "# exported by mapping_v2 bagtools/export_db3.py from {}\n".format(os.path.basename(self.bag))
        with open(os.path.join(out, "rgb.txt"), "w") as fh:
            fh.write("# color images\n" + hdr + "# timestamp filename\n")
            for r in paired:
                fh.write(f"{r['t_colour_s']} rgb/{r['name']}.png\n")
        with open(os.path.join(out, "depth.txt"), "w") as fh:
            fh.write("# depth maps (aligned to colour, 5000 units per metre)\n" + hdr + "# timestamp filename\n")
            for r in paired:
                fh.write(f"{r['t_depth_s']} depth/{r['name']}.png\n")
        with open(os.path.join(out, "associations.txt"), "w") as fh:
            for r in paired:
                fh.write(f"{r['t_colour_s']} rgb/{r['name']}.png {r['t_depth_s']} depth/{r['name']}.png\n")

        for link, target in (("rgb_sync", "rgb"), ("depth_sync", "depth")):
            p = os.path.join(out, link)
            if os.path.lexists(p):
                os.remove(p)
            os.symlink(target, p)

        # IMU
        gyro = sorted(self.gyro)
        accel = sorted(self.accel)
        for fname, data in (("imu_gyro.csv", gyro), ("imu_accel.csv", accel)):
            with open(os.path.join(out, fname), "w") as fh:
                fh.write("t_s,frame_number,x,y,z\n")
                for t, fn, x, y, z in data:
                    fh.write(f"{t / 1000.0:.6f},{fn},{x:.9g},{y:.9g},{z:.9g}\n")
        n_combined = 0
        if gyro and accel:
            ta = np.array([a[0] for a in accel])
            av = np.array([a[2:] for a in accel])
            with open(os.path.join(out, "imu.csv"), "w") as fh:
                fh.write("#timestamp [ns],w_x [rad s^-1],w_y [rad s^-1],w_z [rad s^-1],"
                         "a_x [m s^-2],a_y [m s^-2],a_z [m s^-2]\n")
                for t, fn, x, y, z in gyro:
                    if t < ta[0] or t > ta[-1]:
                        continue
                    a = [np.interp(t, ta, av[:, i]) for i in range(3)]
                    fh.write(f"{int(round(t * 1e6))},{x:.9g},{y:.9g},{z:.9g},{a[0]:.9g},{a[1]:.9g},{a[2]:.9g}\n")
                    n_combined += 1

        # calibration files
        with open(os.path.join(out, "calib.yaml"), "w") as fh:
            yaml.safe_dump(calib, fh, sort_keys=False)
        ci = calib["colour"]
        with open(os.path.join(out, "intrinsic.json"), "w") as fh:
            json.dump({"width": ci["width"], "height": ci["height"],
                       "intrinsic_matrix": [ci["fx"], 0, 0, 0, ci["fy"], 0, ci["cx"], ci["cy"], 1]}, fh, indent=1)
        write_rtabmap_calib(os.path.join(out, "rtabmap_calib.yaml"), ci)

        warnings = list(self.errors)
        if any(abs(c) > 1e-3 for c in ci["coeffs"]):
            warnings.append(f"colour distortion is non-zero ({ci['model']} {ci['coeffs']}); "
                            "engine calibration files use zero distortion")
        if dup_names:
            warnings.append(f"{dup_names} duplicate colour timestamps")

        dts = np.array([float(r["dt_depth_minus_colour_ms"]) for r in paired]) if paired else np.array([0.0])
        meta = {
            "source_bag": os.path.abspath(self.bag),
            "source_size_bytes": os.path.getsize(self.bag),
            "source_sha256_first64MB_plus_size": sha256_head(self.bag),
            "exported_at": datetime.datetime.now().isoformat(timespec="seconds"),
            "export_seconds": round(time.time() - t_start, 1),
            "max_frames": self.max_frames,
            "versions": {"pyrealsense2": rs.__version__, "numpy": np.__version__, "opencv": cv2.__version__,
                         "python": platform.python_version(), "machine": platform.machine()},
            "colour_frames_received": self.n_colour_in,
            "depth_frames_received": self.n_depth_in,
            "colour_frames_written": len(paired),
            "colour_frames_no_depth": sum(1 for r in rows if r["flag"] == "no_depth"),
            "colour_frames_depth_reused": sum(1 for r in rows if r["flag"] == "depth_reused"),
            "dt_depth_minus_colour_ms": {"median": float(np.median(dts)), "max_abs": float(np.abs(dts).max())},
            "pairing_threshold_ms": self.half_period_ms,
            "gyro_samples": len(gyro), "accel_samples": len(accel), "imu_csv_rows": n_combined,
            "warnings": warnings,
        }
        with open(os.path.join(out, "meta.json"), "w") as fh:
            json.dump(meta, fh, indent=2)

        print("=" * 70)
        for k in ("colour_frames_received", "depth_frames_received", "colour_frames_written",
                  "colour_frames_no_depth", "colour_frames_depth_reused", "gyro_samples", "accel_samples",
                  "export_seconds"):
            print(f"{k}: {meta[k]}")
        print(f"dt depth-colour ms: median {meta['dt_depth_minus_colour_ms']['median']:.2f}, "
              f"max |dt| {meta['dt_depth_minus_colour_ms']['max_abs']:.2f}")
        for wmsg in warnings:
            print("WARNING:", wmsg)
        print(f"Output: {out}")
        print("Next: python bagtools/check_export.py", self.bag, out)


def write_rtabmap_calib(path, ci, name="rtabmap_calib"):
    """RTAB-Map CameraModel yaml (same layout RTAB-Map itself writes)."""
    fx, fy, cx, cy = ci["fx"], ci["fy"], ci["cx"], ci["cy"]

    def mat(rows, cols, data):
        return (f"   rows: {rows}\n   cols: {cols}\n   data: [ "
                + ", ".join(repr(float(v)) for v in data) + " ]\n")

    txt = ("%YAML:1.0\n---\n"
           f"camera_name: {name}\n"
           f"image_width: {ci['width']}\nimage_height: {ci['height']}\n"
           "camera_matrix:\n" + mat(3, 3, [fx, 0, cx, 0, fy, cy, 0, 0, 1]) +
           "distortion_coefficients:\n" + mat(1, 5, [0, 0, 0, 0, 0]) +
           "distortion_model: plumb_bob\n"
           "rectification_matrix:\n" + mat(3, 3, [1, 0, 0, 0, 1, 0, 0, 0, 1]) +
           "projection_matrix:\n" + mat(3, 4, [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]) +
           # camera optical frame (z forward) in RTAB-Map's base frame (x forward, z up)
           "local_transform:\n" + mat(3, 4, [0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0]))
    with open(path, "w") as fh:
        fh.write(txt)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--max-frames", type=int, default=None, help="stop after N colour frames (smoke test)")
    ap.add_argument("--force", action="store_true", help="delete OUT if it already exists")
    a = ap.parse_args()
    if not os.path.isfile(a.bag):
        sys.exit(f"ERROR: no such file: {a.bag}")
    if os.path.exists(a.out) and os.listdir(a.out):
        if not a.force:
            sys.exit(f"ERROR: {a.out} exists and is not empty (use --force to replace it)")
        shutil.rmtree(a.out)
    os.makedirs(a.out, exist_ok=True)
    print(f"Exporting {a.bag} -> {a.out}" + (f" (first {a.max_frames} colour frames)" if a.max_frames else ""))
    Exporter(a.bag, a.out, a.max_frames).run()


if __name__ == "__main__":
    main()
