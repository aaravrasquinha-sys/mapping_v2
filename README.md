# mapping_v2

Offline indoor 3D mapping from Intel RealSense D435i recordings (`.db3`).
The Jetson records; the HP Z440 builds the map. Proven engines (RTAB-Map, ORB-SLAM3, Open3D) do the
mapping; this repo holds the exact exporter, an engine-independent evaluator, one shared map builder,
the comparison, and the final packaging. See [PLAN.md](PLAN.md) for the plan, targets and findings.

Nothing here imports or modifies `spatial_tracking_system_v1.0` or `pyslam`.

**Deploying on the Z440 from scratch: follow [RUNBOOK.md](RUNBOOK.md)** (bag copy from the Jetson, install, smoke
test, real run, what to send back). The sections below are the reference.

## Quick start on the Z440 (Ubuntu 22.04)

Each block is copy-paste. Paths assume the repo is cloned to `~/mv2`.

### 1. Code, machine check
```bash
git clone https://github.com/aaravrasquinha-sys/mapping_v2 ~/mv2      # later: cd ~/mv2 && git pull
cd ~/mv2
bash setup/check_machine.sh
```
Paste the whole output.

### 2. Engines, built natively (once; 30-90 min; asks for your sudo password for apt)
```bash
cd ~/mv2
bash setup/install_engines.sh          # RTAB-Map + g2o, Pangolin + ORB-SLAM3, Open3D 0.19.0 env
```
Must end with `RTAB-Map OK`, `ORB-SLAM3 OK` and `install_engines: done`. Re-running skips what is done.
Everything goes into `~/mv2_tools` (and `~/venvs/o3d019`); no Docker needed.
If the ORB-SLAM3 compiler gets killed (not enough RAM), add swap and re-run:
`sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile`

### 3. Python environment and self-test (about 5 min, ~2 GB; afterwards `bash setup/verify_install.sh` checks everything)
```bash
bash setup/make_venv.sh                # creates ~/venvs/mv2
source ~/venvs/mv2/bin/activate
bash tests/run_bagtools_tests.sh       # must end with: ALL TESTS PASSED
```

### 4. Copy the recording and protect it (needs ~9 GB)
```bash
mkdir -p ~/mv2_data/bags
scp drdo@192.168.1.101:bags/walk1.db3 ~/mv2_data/bags/
sha256sum ~/mv2_data/bags/walk1.db3
ssh drdo@192.168.1.101 sha256sum bags/walk1.db3     # must print the same hash
chmod a-w ~/mv2_data/bags/walk1.db3
```

### 5. Smoke test: the whole pipeline on the first 300 frames (about 5 min)
```bash
cd ~/mv2 && source ~/venvs/mv2/bin/activate
bash run_all.sh ~/mv2_data/bags/walk1.db3 --smoke 300 --engines "rtabmap orbslam3"
```
It ends by printing a summary (`~/mv2_data/runs/SUMMARY_walk1_smoke300.txt`). Paste it.
Ten seconds of walking cannot have revisits, so `n/a` there is expected.

### 6. The real run (estimated 2-5 h; Open3D is the slow part)
```bash
bash run_all.sh ~/mv2_data/bags/walk1.db3
```
What it does, in order, skipping anything already finished (so after a failure or Ctrl-C, run the same
command again):

1. inspect the bag; export every frame and IMU sample exactly; check the export independently against
   the bag (a failed check stops everything); lock the dataset read-only
2. find revisits by appearance and measure them directly (the engine-independent drift yardstick)
3. RTAB-Map R1 and R2, ORB-SLAM3 S1; each trajectory is evaluated (coverage, jumps, rotation vs gyro,
   revisits, sharpness, floor tilt/flatness) and fused by the same map builder
4. comparison against the fixed targets, a recommended run, and a **provisional final map**
   in `~/mv2_data/final/walk1/` with a replay video
5. Open3D O1 (1-3 h), then the comparison again; the final map is rebuilt only if the pick changes

`--slac` adds Open3D O2 (another 2-6 h). `--engines "rtabmap orbslam3"` skips Open3D.
Measured on a 240 s synthetic walk with only 2 cores: revisit pairs 23 min, RTAB-Map R1+R2 64 min, ORB-SLAM3
28 min (each including evaluation and map building); the Z440 should be faster. The provisional final map
after step 4 is usually what you want to look at first.

Send back: the printed summary (also in `~/mv2_data/runs/SUMMARY_walk1.txt`),
`~/mv2_data/runs/compare_walk1.png` and `~/mv2_data/final/walk1/views.png`.

### 7. Look at the map
`~/mv2_data/final/walk1/` holds `map_mesh.ply` (full), `map_mesh_view.ply` (lighter), `map_cloud.ply`,
`trajectory_map.tum`, `map_frame.json`, `metrics.json`, `views.png`, `replay.mp4`, `MANIFEST.json`
(provenance + sha256) and `README.txt`. Frame: metres, +z up (gravity), floor at z = 0.
```bash
sudo apt install -y meshlab && meshlab ~/mv2_data/final/walk1/map_mesh_view.ply
# or: python -c "import open3d as o3d, sys; o3d.visualization.draw_geometries([o3d.io.read_triangle_mesh(sys.argv[1])])" ~/mv2_data/final/walk1/map_mesh_view.ply
```
The recommendation comes from the numbers; we look at the views together before calling it final.
To package a different run instead:
`python finish/finalize_map.py ~/mv2_data/datasets/walk1 ~/mv2_data/runs/<run> ~/mv2_data/final/walk1_<run>`
(add `--colour-opt` to try colour refinement; it is kept only if it measurably improves colours).

## Layout
| Folder | What |
|---|---|
| `run_all.sh` | the whole pipeline, resumable |
| `setup/` | machine check, engine builds (`install_engines.sh`), Python environment (pinned versions) |
| `bagtools/` | `.db3` inspector, exporter, export checker |
| `engines/` | runners: `rtabmap/`, `orbslam3/`, `open3d_recon/` (native by default; Docker optional) |
| `eval/` | trajectory converter, revisit pairs, evaluator, shared map builder, views, comparison |
| `finish/` | final-map packaging, replay video |
| `mv2lib/` | shared Python helpers |
| `tests/` | synthetic recordings made with the real SDK, self-tests |

Data lives outside git, in `~/mv2_data/` (`bags/`, `datasets/`, `runs/`, `final/`, `logs/`).

## Single steps (what `run_all.sh` calls)
```bash
python bagtools/inspect_db3.py BAG --out NAME.inspect.json
python bagtools/export_db3.py BAG ~/mv2_data/datasets/NAME [--max-frames N]
python bagtools/check_export.py BAG ~/mv2_data/datasets/NAME --lock
python eval/revisit_pairs.py ~/mv2_data/datasets/NAME ~/mv2_data/runs/_cache/NAME/revisit_pairs.json
bash engines/rtabmap/run_rtabmap.sh   DATASET ~/mv2_data/runs --r2 [--max-frames N] [--docker]
bash engines/orbslam3/run_orbslam3.sh DATASET ~/mv2_data/runs      [--max-frames N] [--docker]
bash engines/open3d_recon/run_open3d.sh DATASET ~/mv2_data/runs [--slac] [--max-frames N]
python eval/evaluate.py DATASET RUN_DIR          # metrics.json (+ results.csv)
python eval/fuse.py DATASET RUN_DIR              # shared map builder -> map_mesh.ply, map_cloud.ply, views.png
python eval/compare_runs.py ~/mv2_data/runs --dataset NAME --pick
python finish/finalize_map.py DATASET RUN_DIR OUT_DIR [--colour-opt]
python finish/render_video.py DATASET RUN_DIR [--step 2]
```
Each run folder (`~/mv2_data/runs/<date>_<engine>_<variant>`) holds the engine's own output and log,
`run.json`, `trajectory.tum`, `metrics.json`, `map_mesh.ply`, `map_cloud.ply`, `map_frame.json`, `views.png`.

Docker instead of native builds (optional): `docker pull introlab3it/rtabmap:noble` and
`docker build -t mv2/orbslam3:4452a3c engines/orbslam3`, then pass `--docker` to the runners.
The native builds are the tested path.

## Exported dataset format
| File | Content |
|---|---|
| `rgb/<t>.png` | colour, 8-bit; `<t>` = colour device time in seconds |
| `depth/<t>.png` | uint16, **5000 units per metre** (TUM convention, lossless from 1 mm), aligned to colour, same name as its colour frame |
| `rgb_sync`, `depth_sync` | symlinks to `rgb/`, `depth/` (names `rtabmap-rgbd_dataset` expects) |
| `rgb.txt`, `depth.txt`, `associations.txt` | TUM RGB-D lists |
| `frames.csv` | per colour frame: pairing, colour–depth time gap, blur, brightness, valid-depth fraction, flags |
| `imu_gyro.csv`, `imu_accel.csv` | every IMU sample, device time in seconds |
| `imu.csv` | gyro times + interpolated accel, EuRoC style (ns) |
| `calib.yaml` | intrinsics, extrinsics, depth scale, IMU info — read from the file |
| `intrinsic.json`, `rtabmap_calib.yaml` | Open3D and RTAB-Map calibration files |
| `meta.json`, `export_report.json`, `contact_sheet.jpg` | provenance, check results, visual check |

Colour frames with no depth frame within half a frame period are listed in `frames.csv` with
flag `no_depth` and are not written to `rgb/`, so `rgb/` and `depth/` always match one-to-one.

## Pinned versions
Python 3.10 · pyrealsense2 2.58.4.10922 · Open3D 0.20.0 (0.19.0 for the Open3D engine) · numpy 1.26.4 ·
OpenCV 4.11.0.86 · zstandard 0.25.0 · rosbags 0.11.7 · evo 1.38.0 · RTAB-Map af1ced2 + g2o 20230806 ·
ORB-SLAM3 4452a3c + Pangolin 0.9.2.
