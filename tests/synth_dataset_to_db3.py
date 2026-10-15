#!/usr/bin/env python3
"""Turn a synthetic room walk (tests/make_synth_rgbd.py output) into a RealSense .db3 recorded with
the real SDK (software_device + recorder), so the WHOLE pipeline - export, check, engines,
evaluation, final map - can be tested from a bag with exact ground truth.

Depth is stored as z16 (1 mm units) with the colour intrinsics and an identity depth->colour
extrinsic (the synthetic depth is already in the colour camera), 2 ms after colour; IMU samples
are stored at their own rates with the dataset's gyro->colour extrinsic.
groundtruth.txt is copied next to the bag as <name>.groundtruth.txt.

Usage: python tests/synth_dataset_to_db3.py SYNTH_DATASET OUT.db3 [--max-frames N]
"""
import argparse
import csv
import os
import shutil
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml

DEPTH_OFFSET_MS = 2.0


def intr(c):
    i = rs.intrinsics()
    i.width, i.height = c["width"], c["height"]
    i.fx, i.fy, i.ppx, i.ppy = c["fx"], c["fy"], c["cx"], c["cy"]
    i.model = rs.distortion.none
    i.coeffs = [0.0] * 5
    return i


def read_imu(path):
    with open(path) as fh:
        return [(float(r["t_s"]), float(r["x"]), float(r["y"]), float(r["z"])) for r in csv.DictReader(fh)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("out")
    ap.add_argument("--max-frames", type=int)
    a = ap.parse_args()
    ds = os.path.abspath(a.dataset)
    cal = yaml.safe_load(open(os.path.join(ds, "calib.yaml")))
    c = cal["colour"]
    W, H = c["width"], c["height"]
    fps = int(round(cal["fps"]))
    names = sorted(os.listdir(os.path.join(ds, "rgb")))[: a.max_frames]
    gyro = read_imu(os.path.join(ds, "imu_gyro.csv"))
    accel = read_imu(os.path.join(ds, "imu_accel.csv"))
    if os.path.exists(a.out):
        os.remove(a.out)

    dev = rs.software_device()
    s_depth = dev.add_sensor("Stereo Module")
    s_color = dev.add_sensor("RGB Camera")
    s_mot = dev.add_sensor("Motion Module")
    vs = rs.video_stream()
    vs.type, vs.index, vs.uid = rs.stream.depth, 0, 0
    vs.width, vs.height, vs.fps, vs.bpp, vs.fmt = W, H, fps, 2, rs.format.z16
    vs.intrinsics = intr(c)
    dp = s_depth.add_video_stream(vs)
    cs = rs.video_stream()
    cs.type, cs.index, cs.uid = rs.stream.color, 0, 1
    cs.width, cs.height, cs.fps, cs.bpp, cs.fmt = W, H, fps, 3, rs.format.rgb8
    cs.intrinsics = intr(c)
    cp = s_color.add_video_stream(cs)
    gs = rs.motion_stream()
    gs.type, gs.index, gs.uid, gs.fps, gs.fmt = rs.stream.gyro, 0, 2, int(cal["gyro_rate_hz"]), rs.format.motion_xyz32f
    gs.intrinsics = rs.motion_device_intrinsic()
    gp = s_mot.add_motion_stream(gs)
    as_ = rs.motion_stream()
    as_.type, as_.index, as_.uid, as_.fps, as_.fmt = rs.stream.accel, 0, 3, int(cal["accel_rate_hz"]), rs.format.motion_xyz32f
    as_.intrinsics = rs.motion_device_intrinsic()
    ap_ = s_mot.add_motion_stream(as_)

    ext = rs.extrinsics()
    ext.rotation = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    ext.translation = [0.0, 0.0, 0.0]
    dp.register_extrinsics_to(cp, ext)
    T = np.array(cal["T_colour_from_gyro"], float)
    ext_imu = rs.extrinsics()
    ext_imu.rotation = T[:3, :3].T.flatten().tolist()       # SDK stores column-major
    ext_imu.translation = T[:3, 3].tolist()
    gp.register_extrinsics_to(cp, ext_imu)
    ap_.register_extrinsics_to(cp, ext_imu)
    s_depth.add_read_only_option(rs.option.depth_units, 0.001)

    rec = rs.recorder(a.out, dev)
    sensors = rec.query_sensors()
    for s in sensors:
        s.open(s.get_stream_profiles())
    for s in sensors:
        s.start(lambda f: None)

    def video(sw, prof, buf, bpp, t_ms, fn):
        f = rs.software_video_frame()
        f.pixels = buf
        f.stride = W * bpp
        f.bpp = bpp
        f.timestamp = t_ms
        f.domain = rs.timestamp_domain.global_time
        f.frame_number = fn
        f.profile = prof.as_video_stream_profile()
        sw.on_video_frame(f)

    def motion(prof, t_ms, fn, xyz):
        m = rs.software_motion_frame()
        v = rs.vector()
        v.x, v.y, v.z = xyz
        m.data = v
        m.timestamp = t_ms
        m.domain = rs.timestamp_domain.global_time
        m.frame_number = fn
        m.profile = prof.as_motion_stream_profile()
        s_mot.on_motion_frame(m)

    gi = ai = 0
    t_end = float(os.path.splitext(names[-1])[0]) + 1.0 / fps
    for k, n in enumerate(names):
        tc = float(os.path.splitext(n)[0])
        t_next = tc + 1.0 / fps if k + 1 < len(names) else t_end
        while gi < len(gyro) and gyro[gi][0] < t_next:
            motion(gp, gyro[gi][0] * 1000.0, gi, gyro[gi][1:])
            gi += 1
        while ai < len(accel) and accel[ai][0] < t_next:
            motion(ap_, accel[ai][0] * 1000.0, ai, accel[ai][1:])
            ai += 1
        rgb = cv2.cvtColor(cv2.imread(os.path.join(ds, "rgb", n)), cv2.COLOR_BGR2RGB)
        d = cv2.imread(os.path.join(ds, "depth", n), cv2.IMREAD_UNCHANGED).astype(np.float64)
        d_mm = np.round(d * 1000.0 / cal["depth_png_units_per_m"]).astype(np.uint16)
        video(s_color, cp, np.ascontiguousarray(rgb), 3, tc * 1000.0, k)
        video(s_depth, dp, np.ascontiguousarray(d_mm), 2, tc * 1000.0 + DEPTH_OFFSET_MS, k)
        time.sleep(0.004)
        if k % 200 == 0:
            print(f"  {k}/{len(names)}", flush=True)
    time.sleep(2.0)
    for s in sensors:
        s.stop()
        s.close()
    del rec
    gt = os.path.join(ds, "groundtruth.txt")
    if os.path.isfile(gt):
        shutil.copy(gt, os.path.splitext(a.out)[0] + ".groundtruth.txt")
    print(f"wrote {a.out}: {len(names)} frames, gyro {gi}, accel {ai}, {os.path.getsize(a.out) / 1e6:.0f} MB")


if __name__ == "__main__":
    main()
