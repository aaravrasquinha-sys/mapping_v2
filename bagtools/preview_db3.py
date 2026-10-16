#!/usr/bin/env python3
"""Make a viewable video from a RealSense .db3 recording: colour on the left, depth on the right.

For LOOKING at a recording (is it sharp, is the depth sensible, where does it shake?). The mapping
pipeline does not use this: it exports every frame exactly (bagtools/export_db3.py).
Plays the file with the SDK in non-real-time mode (nothing is dropped because the machine is busy),
keeps every STEP-th colour frame, and writes an mp4 that plays at SPEED x the real speed.
Depth colours: blue = near, red = far (0..--max-depth m), black = no depth. Overlay: time in seconds.

Usage: python bagtools/preview_db3.py BAG.db3 [OUT.mp4] [--step 3] [--speed 1] [--scale 0.6]
                                      [--max-depth 4] [--start-s 0] [--max-seconds 0]
Default OUT: next to the bag, BAG.preview.mp4. Needs the project venv (pyrealsense2, opencv, numpy).
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("out", nargs="?")
    ap.add_argument("--step", type=int, default=3, help="keep every N-th colour frame (default 3: 30 fps -> 10 fps)")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed of the video (default 1 = real time)")
    ap.add_argument("--scale", type=float, default=0.6, help="size factor of each panel (default 0.6)")
    ap.add_argument("--max-depth", type=float, default=4.0, help="depth shown up to this many metres")
    ap.add_argument("--start-s", type=float, default=0.0, help="skip the first N seconds of the recording")
    ap.add_argument("--max-seconds", type=float, default=0.0, help="stop after N seconds of recording (0 = all)")
    a = ap.parse_args()
    out = a.out or os.path.splitext(a.bag)[0] + ".preview.mp4"

    cfg = rs.config()
    cfg.enable_device_from_file(os.path.abspath(a.bag), repeat_playback=False)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    pb = prof.get_device().as_playback()
    pb.set_real_time(False)
    fps = 30.0
    for s in prof.get_streams():
        if s.stream_type() == rs.stream.color:
            fps = float(s.fps())
    out_fps = max(1.0, fps / a.step * a.speed)
    print(f"{a.bag}: colour {fps:g} fps; every {a.step}th frame, video at {out_fps:g} fps ({a.speed:g}x) -> {out}", flush=True)

    writer = None
    last_fn, kept, seen, t0, t_start = -1, 0, 0, None, time.time()
    while True:
        try:
            fr = pipe.wait_for_frames(5000)
        except RuntimeError:                       # end of file
            break
        c, d = fr.get_color_frame(), fr.get_depth_frame()
        if not c or c.get_frame_number() == last_fn:
            continue
        last_fn = c.get_frame_number()
        t = c.get_timestamp() / 1000.0
        if t0 is None:
            t0 = t
        if t - t0 < a.start_s:
            continue
        if a.max_seconds and t - t0 > a.start_s + a.max_seconds:
            break
        seen += 1
        if (seen - 1) % a.step:
            continue
        img = cv2.cvtColor(np.asanyarray(c.get_data()), cv2.COLOR_RGB2BGR)
        if d:
            z = np.asanyarray(d.get_data()).astype(np.float32) * d.get_units()
            vis = np.clip(z / a.max_depth * 255, 0, 255).astype(np.uint8)
            vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
            vis[z <= 0] = 0
            vis = cv2.resize(vis, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        else:
            vis = np.zeros_like(img)
        frame = np.hstack([img, vis])
        if a.scale != 1.0:
            frame = cv2.resize(frame, None, fx=a.scale, fy=a.scale, interpolation=cv2.INTER_AREA)
        label = f"t = {t - t0:6.1f} s   frame {c.get_frame_number()}"
        cv2.putText(frame, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 3)
        cv2.putText(frame, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        if writer is None:
            writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), out_fps, (frame.shape[1], frame.shape[0]))
            if not writer.isOpened():
                sys.exit("ERROR: could not open the video writer for " + out)
        writer.write(frame)
        kept += 1
        if kept % 200 == 0:
            print(f"  t = {t - t0:.0f} s, {kept} video frames", flush=True)
    pipe.stop()
    if writer is None:
        sys.exit("ERROR: no colour frames found in the file")
    writer.release()
    print(f"done: {kept} video frames, {kept / out_fps:.0f} s long, {os.path.getsize(out) / 1e6:.0f} MB, "
          f"{time.time() - t_start:.0f} s -> {out}")


if __name__ == "__main__":
    main()
