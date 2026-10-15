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
| `engines/`, `eval/`, `finish/` | coming in the next batches |

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
