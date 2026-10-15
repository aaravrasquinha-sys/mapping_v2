# mapping_v2

Offline indoor 3D mapping from Intel RealSense D435i recordings (`.db3`).
The Jetson records; the HP Z440 builds the map. Proven engines (RTAB-Map, Open3D, ORB-SLAM3 if needed)
do the mapping; this repo holds the exact exporter, the evaluator, a shared map builder and glue.
See [PLAN.md](PLAN.md) for the full plan and current status.

Nothing here imports or modifies `spatial_tracking_system_v1.0` or `pyslam`.

## Layout
| Folder | What |
|---|---|
| `setup/` | machine check, Python environment (pinned versions) |
| `bagtools/` | `.db3` inspector, exporter, export checker |
| `tests/` | synthetic recordings made with the real SDK, self-tests |
| `engines/rtabmap/` | RTAB-Map runner (Docker, or native) |
| `eval/` | trajectory converter, evaluator, shared map builder (TSDF), fixed-view renders |
| `mv2lib/` | shared Python helpers |
| `finish/`, other engines | coming in the next batches |

Data lives outside git, in `~/mv2_data/` (`bags/`, `datasets/`, `runs/`).

## Steps on the Z440 (batch 1 + 2)

Run each block, then paste the lines asked for. Paths assume the repo is cloned to `~/mv2`.

### 1. Get the code and check the machine
```bash
git clone https://github.com/aaravrasquinha-sys/mapping_v2 ~/mv2
cd ~/mv2
bash setup/check_machine.sh
```
Paste the whole output.

### 2. Python environment and self-test (about 5 min, ~2 GB)
```bash
cd ~/mv2
bash setup/make_venv.sh            # creates ~/venvs/mv2
source ~/venvs/mv2/bin/activate
bash tests/run_bagtools_tests.sh   # must end with: ALL TESTS PASSED
```
If `make_venv.sh` says it cannot create a venv: `sudo apt install python3.10-venv`, then run it again.

### 3. Copy the recording and protect it (needs ~9 GB free)
```bash
mkdir -p ~/mv2_data/bags
scp drdo@192.168.1.101:bags/walk1.db3 ~/mv2_data/bags/
sha256sum ~/mv2_data/bags/walk1.db3
ssh drdo@192.168.1.101 sha256sum bags/walk1.db3     # must print the same hash
chmod a-w ~/mv2_data/bags/walk1.db3
# old run A results, only to sanity-check the evaluator later (a copy; the Jetson repo is untouched)
scp -r drdo@192.168.1.101:spatial_tracking_system_v1.0/runs/run_1792054584_bag ~/mv2_data/old_runA
```

### 4. Inspect (a minute or two)
```bash
cd ~/mv2 && source ~/venvs/mv2/bin/activate
python bagtools/inspect_db3.py ~/mv2_data/bags/walk1.db3 --out ~/mv2_data/walk1.inspect.json
```
Paste everything from the first `- sensor_` line to the end (the stream details and the `VERDICT` lines).

### 5. Smoke-test export: first 300 frames (under a minute)
```bash
python bagtools/export_db3.py ~/mv2_data/bags/walk1.db3 ~/mv2_data/datasets/walk1_smoke --max-frames 300
python bagtools/check_export.py ~/mv2_data/bags/walk1.db3 ~/mv2_data/datasets/walk1_smoke
```
Paste the summary lines and look at `~/mv2_data/datasets/walk1_smoke/contact_sheet.jpg`
(left: colour, right: depth 0–5 m, blue = near, red = far, black = no depth).

### 6. Full export (estimated 5–15 min, ~7 GB)
```bash
python bagtools/export_db3.py ~/mv2_data/bags/walk1.db3 ~/mv2_data/datasets/walk1
python bagtools/check_export.py ~/mv2_data/bags/walk1.db3 ~/mv2_data/datasets/walk1 --lock
rm -rf ~/mv2_data/datasets/walk1_smoke
```
Paste the exporter summary, all `PASS`/`FAIL` lines, the `QUALITY` line and `RESULT`,
and send the contact sheet. `--lock` makes the dataset read-only once it passes.

## Steps on the Z440 (batch 3): RTAB-Map baseline

### 7. Docker (once; skip if `docker run hello-world` already works without sudo)
```bash
sudo apt install -y docker.io
sudo usermod -aG docker $USER      # then log out and back in (or run: newgrp docker)
docker run --rm hello-world
docker pull introlab3it/rtabmap:noble          # ~1.4 GB download, ~5 GB on disk
docker image inspect --format '{{index .RepoDigests 0}}' introlab3it/rtabmap:noble
```
Paste the digest line (it pins the exact image; every run also records it in `run.json`).

### 8. Smoke test: first 300 frames (a few minutes)
```bash
cd ~/mv2 && git pull && source ~/venvs/mv2/bin/activate
bash engines/rtabmap/run_rtabmap.sh ~/mv2_data/datasets/walk1 ~/mv2_data/runs --max-frames 300 --no-fuse
```
Expected in the log: `ERROR: Dataset walk1 is not supported` and `Cannot save calibration ... because it is empty`.
Both are normal: RTAB-Map then loads our `rtabmap_calib.yaml`. Paste the lines from `coverage:` to `gravity:`.
(Coverage will be about 300/7185 frames in a smoke test; that is expected.)

### 9. Full run, R1 and R2 (estimated 20–60 min)
```bash
bash engines/rtabmap/run_rtabmap.sh ~/mv2_data/datasets/walk1 ~/mv2_data/runs --r2
```
Each run folder (`~/mv2_data/runs/<date>_rtabmap_R1`, `..._R2`) gets: RTAB-Map database and log,
`trajectory.tum`, `metrics.json`, `map_mesh.ply`, `map_cloud.ply`, `map_frame.json`, `views.png`.
Paste the summary blocks (from `coverage:` to `gravity:`) and `cat ~/mv2_data/runs/results.csv`, and send both
`views.png`. Open the maps in MeshLab or CloudCompare (`sudo apt install meshlab`), or:
```bash
python -c "import open3d as o3d, sys; o3d.visualization.draw_geometries([o3d.io.read_triangle_mesh(sys.argv[1])])" ~/mv2_data/runs/<run>/map_mesh.ply
```

## Steps on the Z440 (batch 4): revisit checks, Open3D, comparison

### 10. Revisit pairs: once per dataset (estimated 10–20 min)
```bash
cd ~/mv2 && git pull && source ~/venvs/mv2/bin/activate
python eval/revisit_pairs.py ~/mv2_data/datasets/walk1 ~/mv2_data/runs/_cache/walk1/revisit_pairs.json
```
Finds places the walk sees again ≥ 20 s later (by appearance, not by any trajectory) and measures
their relative pose directly (features + colour-assisted ICP). On a synthetic walk with exact ground truth
the accepted pairs were within 5 mm / 0.05°. Paste the last line (number of pairs, start-end pairs).

Then re-score the RTAB-Map runs so they include revisits and sharpness:
```bash
for r in ~/mv2_data/runs/*_rtabmap_R*; do case "$r" in *smoke*) continue;; esac; python eval/evaluate.py ~/mv2_data/datasets/walk1 "$r"; done
```

### 11. Open3D reconstruction system (setup once, then O1 and O2; each estimated 1–3 h on CPU)
```bash
bash engines/open3d_recon/setup_open3d.sh        # isolated env with Open3D 0.19.0 (0.20.0 has a TSDF bug)
bash engines/open3d_recon/run_open3d.sh ~/mv2_data/datasets/walk1 ~/mv2_data/runs --max-frames 300 --no-fuse   # smoke test
bash engines/open3d_recon/run_open3d.sh ~/mv2_data/datasets/walk1 ~/mv2_data/runs            # O1
bash engines/open3d_recon/run_open3d.sh ~/mv2_data/datasets/walk1 ~/mv2_data/runs --slac     # O2
```

### 12. Compare all runs
```bash
python eval/compare_runs.py ~/mv2_data/runs --dataset walk1
```
Paste the table it prints and send `~/mv2_data/runs/compare.png`.

### 12b. ORB-SLAM3 (only if RTAB-Map and Open3D disagree or neither is clean)
```bash
docker build -t mv2/orbslam3:4452a3c engines/orbslam3          # once, 20-60 min, ~4 GB
bash engines/orbslam3/run_orbslam3.sh ~/mv2_data/datasets/walk1 ~/mv2_data/runs --max-frames 300 --no-fuse   # smoke test
bash engines/orbslam3/run_orbslam3.sh ~/mv2_data/datasets/walk1 ~/mv2_data/runs
```
ORB-SLAM3 is GPLv3: fine as a reference; it matters only if it ships inside a product.

## Steps on the Z440 (batch 5): final map and replay video

### 13. Final map from the winning run (after we pick it together)
```bash
python finish/finalize_map.py ~/mv2_data/datasets/walk1 ~/mv2_data/runs/<winning run> ~/mv2_data/final/walk1
# optional, kept only if it measurably improves colours:
#   add --colour-opt
```
`~/mv2_data/final/walk1/` then holds `map_mesh.ply` (full), `map_mesh_view.ply` (lighter), `map_cloud.ply`,
`trajectory_map.tum`, `map_frame.json`, `metrics.json`, `views.png`, `MANIFEST.json` (provenance + sha256), `README.txt`.
Frame: metres, +z up (gravity), floor at z = 0.

### 14. Replay video (a few minutes)
```bash
python finish/render_video.py ~/mv2_data/datasets/walk1 ~/mv2_data/runs/<run>
```
Writes `<run>/replay.mp4`: camera image with features (red LOST banner when the engine had no pose) next to the
map growing as it is seen, with the path and the current camera. `--step 2` halves the length.

Single steps, if needed: `eval/convert_traj.py` (engine trajectory → project format), `eval/evaluate.py`
(metrics), `eval/fuse.py` (shared map builder), `eval/render_views.py` (fixed views).

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
Python 3.10 · pyrealsense2 2.58.4.10922 · Open3D 0.20.0 · numpy 1.26.4 · OpenCV 4.11.0.86 ·
zstandard 0.25.0 · rosbags 0.11.7 · evo 1.38.0 (all have x86_64 and aarch64 wheels).
