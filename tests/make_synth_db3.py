#!/usr/bin/env python3
"""Make a synthetic RealSense .db3 with the real SDK (software_device + recorder).

Used to test bagtools without a camera. The file mimics a D435i recording:
- colour 640x480 rgb8 and depth 640x480 z16, 30 fps, different intrinsics
- depth->colour extrinsic with a 15 mm baseline offset
- depth timestamps 3 ms after colour (unsynchronised sensors)
- some depth frames and one colour frame missing (lost during "recording")
- gyro 400 Hz and accel 200 Hz, every sample stored

Ground truth for the test is written next to the .db3 as <name>.truth.json.

Usage: python tests/make_synth_db3.py OUT.db3 [--frames N]
"""
import argparse
import json
import os
import time

import numpy as np
import pyrealsense2 as rs

W, H, FPS = 640, 480, 30
DEPTH_MM = 1500            # flat wall at 1.5 m (in front of the depth camera)
DEPTH_OFFSET_MS = 3.0      # depth timestamp minus colour timestamp
GYRO_HZ, ACCEL_HZ = 400, 200
MISSING_DEPTH = (20, 21)   # frame indices with no depth frame
MISSING_COLOR = (50,)      # frame index with no colour frame
BASELINE_X_M = 0.015       # depth->colour translation along x


def make_intr(fx, fy, cx, cy):
    i = rs.intrinsics()
    i.width, i.height = W, H
    i.fx, i.fy, i.ppx, i.ppy = fx, fy, cx, cy
    i.model = rs.distortion.none
    i.coeffs = [0.0] * 5
    return i


def colour_image(k):
    """Textured image that changes with k (so blur/brightness are non-trivial)."""
    y, x = np.mgrid[0:H, 0:W]
    checker = (((x + 3 * k) // 32 + y // 32) % 2) * 120
    r = (checker + x * 100 // W).astype(np.uint8)
    g = (checker + y * 100 // H).astype(np.uint8)
    b = np.full((H, W), (40 + 2 * k) % 255, np.uint8)
    img = np.dstack([r, g, b])
    img[10:20, 10:20] = (255, 0, 0)  # pure red patch top-left: catches RGB/BGR swaps
    return np.ascontiguousarray(img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--frames", type=int, default=90)
    ap.add_argument("--step", action="store_true",
                    help="depth = 1 m left half, 2 m right half (checks alignment direction)")
    ap.add_argument("--bad-imu-stamp", action="store_true",
                    help="add one accel sample stamped near 0 ms (seen in a real recording; the exporter must drop it)")
    a = ap.parse_args()
    if os.path.exists(a.out):
        os.remove(a.out)

    dev = rs.software_device()
    s_depth = dev.add_sensor("Stereo Module")
    s_color = dev.add_sensor("RGB Camera")
    s_mot = dev.add_sensor("Motion Module")

    vs = rs.video_stream()
    vs.type, vs.index, vs.uid = rs.stream.depth, 0, 0
    vs.width, vs.height, vs.fps, vs.bpp, vs.fmt = W, H, FPS, 2, rs.format.z16
    vs.intrinsics = make_intr(385.0, 385.0, 320.0, 240.0)
    dp = s_depth.add_video_stream(vs)

    cs = rs.video_stream()
    cs.type, cs.index, cs.uid = rs.stream.color, 0, 1
    cs.width, cs.height, cs.fps, cs.bpp, cs.fmt = W, H, FPS, 3, rs.format.rgb8
    cs.intrinsics = make_intr(606.75, 606.57, 320.19, 237.06)
    cp = s_color.add_video_stream(cs)

    gs = rs.motion_stream()
    gs.type, gs.index, gs.uid, gs.fps, gs.fmt = rs.stream.gyro, 0, 2, GYRO_HZ, rs.format.motion_xyz32f
    gs.intrinsics = rs.motion_device_intrinsic()
    gp = s_mot.add_motion_stream(gs)
    as_ = rs.motion_stream()
    as_.type, as_.index, as_.uid, as_.fps, as_.fmt = rs.stream.accel, 0, 3, ACCEL_HZ, rs.format.motion_xyz32f
    as_.intrinsics = rs.motion_device_intrinsic()
    ap_ = s_mot.add_motion_stream(as_)

    ext = rs.extrinsics()
    ext.rotation = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    ext.translation = [BASELINE_X_M, 0.0, 0.0]
    dp.register_extrinsics_to(cp, ext)
    ext_imu = rs.extrinsics()
    ext_imu.rotation = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    ext_imu.translation = [-0.005, -0.005, -0.012]
    gp.register_extrinsics_to(cp, ext_imu)
    ap_.register_extrinsics_to(cp, ext_imu)

    s_depth.add_read_only_option(rs.option.depth_units, 0.001)

    rec = rs.recorder(a.out, dev)
    sensors = rec.query_sensors()
    for s in sensors:
        s.open(s.get_stream_profiles())
    for s in sensors:
        s.start(lambda f: None)

    t0 = 1792054584000.3655  # ms, epoch-like (global time); 4 decimals like the real SDK stamps
    period = 1000.0 / FPS
    depth_buf = np.full((H, W), DEPTH_MM, np.uint16)
    if a.step:
        depth_buf[:, : W // 2] = 1000
        depth_buf[:, W // 2:] = 2000
    n_gyro = n_accel = 0
    truth = {"colour_t_ms": [], "depth_t_ms": [], "gyro_t_ms": [], "accel_t_ms": []}

    def video(sw, prof, buf, bpp, t, fn):
        f = rs.software_video_frame()
        f.pixels = buf
        f.stride = W * bpp
        f.bpp = bpp
        f.timestamp = t
        f.domain = rs.timestamp_domain.global_time
        f.frame_number = fn
        f.profile = prof.as_video_stream_profile()
        sw.on_video_frame(f)

    def motion(prof, t, fn, xyz):
        m = rs.software_motion_frame()
        v = rs.vector()
        v.x, v.y, v.z = xyz
        m.data = v
        m.timestamp = t
        m.domain = rs.timestamp_domain.global_time
        m.frame_number = fn
        m.profile = prof.as_motion_stream_profile()
        s_mot.on_motion_frame(m)

    if a.bad_imu_stamp:
        motion(ap_, 5.0, 999999, (0.1, -9.81, 0.2))
    for k in range(a.frames):
        tc = round(t0 + k * period, 4)       # real stamps carry 4 decimals; round-half cases must not break the check
        # IMU samples covering [tc, tc + period)
        while t0 + n_gyro * 1000.0 / GYRO_HZ < tc + period:
            tg = t0 + n_gyro * 1000.0 / GYRO_HZ
            motion(gp, tg, n_gyro, (0.01, 0.02 * np.sin(n_gyro / 50), 0.03))
            truth["gyro_t_ms"].append(tg)
            n_gyro += 1
        while t0 + n_accel * 1000.0 / ACCEL_HZ < tc + period:
            ta = t0 + n_accel * 1000.0 / ACCEL_HZ
            motion(ap_, ta, n_accel, (0.1, -9.81, 0.2))
            truth["accel_t_ms"].append(ta)
            n_accel += 1
        if k not in MISSING_COLOR:
            video(s_color, cp, colour_image(k), 3, tc, k)
            truth["colour_t_ms"].append(tc)
        if k not in MISSING_DEPTH:
            video(s_depth, dp, depth_buf, 2, tc + DEPTH_OFFSET_MS, k)
            truth["depth_t_ms"].append(tc + DEPTH_OFFSET_MS)
        time.sleep(0.003)
    time.sleep(1.0)
    for s in sensors:
        s.stop()
        s.close()
    del rec

    truth.update({
        "frames": a.frames, "missing_depth": list(MISSING_DEPTH), "missing_color": list(MISSING_COLOR),
        "depth_offset_ms": DEPTH_OFFSET_MS, "depth_mm": DEPTH_MM, "baseline_x_m": BASELINE_X_M,
        "n_gyro": n_gyro, "n_accel": n_accel, "step": a.step,
    })
    with open(os.path.splitext(a.out)[0] + ".truth.json", "w") as fh:
        json.dump(truth, fh)
    print(f"wrote {a.out}: {a.frames} frames, gyro {n_gyro}, accel {n_accel}")


if __name__ == "__main__":
    main()
