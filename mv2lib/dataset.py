"""Read an exported dataset folder (see README: 'Exported dataset format')."""
import csv
import json
import os

import cv2
import numpy as np
import yaml

DEPTH_UNITS_PER_M = 5000.0


class Dataset:
    def __init__(self, root):
        self.root = os.path.abspath(root)
        if not os.path.isfile(os.path.join(self.root, "associations.txt")):
            raise FileNotFoundError(f"{self.root} is not an exported dataset (no associations.txt)")
        self.calib = yaml.safe_load(open(os.path.join(self.root, "calib.yaml")))
        self.meta = json.load(open(os.path.join(self.root, "meta.json")))
        self.stamps, self.rgb, self.depth = [], [], []
        for line in open(os.path.join(self.root, "associations.txt")):
            if line.startswith("#") or not line.strip():
                continue
            t, rgb, _, dep = line.split()[:4]
            self.stamps.append(float(t))
            self.rgb.append(rgb)
            self.depth.append(dep)
        self.stamps = np.array(self.stamps)
        self.names = [os.path.splitext(os.path.basename(p))[0] for p in self.rgb]
        c = self.calib["colour"]
        self.width, self.height = int(c["width"]), int(c["height"])
        self.K = np.array([[c["fx"], 0, c["cx"]], [0, c["fy"], c["cy"]], [0, 0, 1.0]])
        self.depth_units_per_m = float(self.calib.get("depth_png_units_per_m", DEPTH_UNITS_PER_M))
        self.name = os.path.basename(self.root.rstrip("/"))

    def __len__(self):
        return len(self.stamps)

    def colour(self, i):
        """BGR uint8"""
        return cv2.imread(os.path.join(self.root, self.rgb[i]), cv2.IMREAD_COLOR)

    def depth_m(self, i):
        d = cv2.imread(os.path.join(self.root, self.depth[i]), cv2.IMREAD_UNCHANGED)
        return d.astype(np.float32) / self.depth_units_per_m

    def frames_csv(self):
        p = os.path.join(self.root, "frames.csv")
        return list(csv.DictReader(open(p))) if os.path.isfile(p) else []

    def imu(self, kind):
        """kind 'gyro' or 'accel' -> (t_s [N], xyz [N,3]) or (None, None) if absent."""
        p = os.path.join(self.root, f"imu_{kind}.csv")
        if not os.path.isfile(p):
            return None, None
        a = np.loadtxt(p, delimiter=",", skiprows=1, ndmin=2)
        if a.size == 0:
            return None, None
        return a[:, 0], a[:, 2:5]

    def T_colour_from_imu(self, kind="gyro"):
        T = self.calib.get(f"T_colour_from_{kind}")
        return None if T is None else np.array(T, dtype=float)

    def depth_noise_model(self):
        """sigma_z(z) = k * z^2 for this stereo camera (subpixel 0.08 px, Intel's typical value)."""
        f = float(self.calib.get("depth_raw", {}).get("fx", 385.0))
        b = float(self.calib.get("stereo_baseline_mm", 50.0)) / 1000.0
        subpixel = 0.08
        return subpixel / (f * b)

    def groundtruth_path(self):
        p = os.path.join(self.root, "groundtruth.txt")
        return p if os.path.isfile(p) else None
