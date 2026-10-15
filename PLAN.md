# Plan (agreed 15 Oct 2026)

Goal: beautiful, clean, metric, gravity-aligned 3D maps of indoor facilities from recorded `.db3` bags.
Real time does not matter (20–30+ min on the Jetson is fine). Use whatever proven engine gives the best map;
write our own code only where proven tools fall short. Benchmark: `walk1.db3` (≈240 s, 7185 frames).

## Status
| Step | State |
|---|---|
| S0 machine facts | waiting for `setup/check_machine.sh` output (assumed: standard Ubuntu 22.04, Docker installable) |
| S1 copy + protect bag | ready (README step 3) |
| S2 inspect | code ready, tested on synthetic files |
| S3 export + check | code ready, tested on synthetic files |
| S4 RTAB-Map R1/R2 | runner ready (README steps 7–9); tested natively with RTAB-Map 0.24.2 on a synthetic walk with ground truth |
| S5 evaluator + shared builder | ready: coverage, speed, gyro windows (+ time offset, bias, convention check), gravity, revisit pairs (incl. start-end), sharpness, fuse, views, run comparison (17 regression checks) |
| S6 Open3D O1/O2 | runner ready (README step 11), O1 verified end to end on the synthetic walk (38 min on 2 cores); engine isolated on Open3D 0.19.0 |
| S8 final map + replay video | ready: `finish/finalize_map.py` (packaging, MANIFEST, optional guarded colour refinement), `finish/render_video.py`; floor tilt/flatness added to the fuser and comparison |
| S7, S9 | after the walk1 results |

## Findings on the synthetic walk (979 frames, exact ground truth)
- RTAB-Map R1: ATE 9–10 mm (matches RTAB-Map's own figure), 100 % posed, map visually clean.
  Runs differ slightly (9.0 vs 10.2 mm): RTAB-Map is not bit-deterministic -> the run-twice rule is needed.
- R2 (exhaustive loop search + global BA) was NOT better locally: gyro windows max 4.6 deg vs 1.3 deg for R1.
- Open3D 0.20.0 (`open3d-cpu`): legacy ScalableTSDFVolume fuses nothing; the builder uses the tensor VoxelBlockGrid.
- Open3D 0.20.0 legacy ScalableTSDFVolume fuses nothing in BOTH open3d and open3d-cpu (0.19.0 works): the Open3D
  reconstruction engine therefore runs in its own env on 0.19.0 (scripts from tag v0.19.0).
- Revisit pairs: point-to-plane ICP slid along walls (errors up to 45 mm); colour-assisted ICP from the PnP pose
  gives <= 5 mm / 0.05 deg vs ground truth. Overlaps that are a single plane or crease are rejected.
- R2 vs R1: tighter loop consistency (revisit rotation 0.07 vs 0.47 deg) but worse local smoothness (gyro max 4.6 deg).
- Open3D O1 on the synthetic walk: ATE 6.9 cm (RTAB-Map ~1 cm); rotation jumps up to 11 deg exactly where its
  100-frame fragments join (frames 400, 500, 600) - caught by the gyro check and revisits without ground truth.
- Floor check on a floor-viewing synthetic walk: tilt 0.35 deg (= the gravity estimate's error), flatness 3.3 mm.
- Colour-map optimisation moved the cameras and made colours worse with exact poses (0.017 -> 0.121): opt-in only.
- A constant slow yaw drift looks like gyro bias to the gyro check; it is flagged as an implausible bias (> 1 deg/s)
  and measured properly by the revisit checks (next batch).

## Facts checked before writing code (on files made with the same SDK, 2.58.4)
- `.db3` = plain SQLite; every IMU sample is its own message; messages are zstd-compressed by default;
  decodable without ROS (`zstandard` + `rosbags`).
- Pipeline playback (syncer) is **not exact**: it thins IMU to one sample per frameset, repeats frames,
  and with colour+depth only it dropped the first frames and mispaired colour/depth.
- Sensor-level playback in non-real-time mode delivers every frame and sample, even with a slow consumer.
  `rs.align` works on colour/depth pairs we build ourselves. Real-time mode drops IMU samples.
- `rtabmap-rgbd_dataset` loads `<dataset>/rtabmap_calib.yaml` for unknown dataset names (no patch needed);
  its depth factor is fixed to TUM units, hence our 5000/m depth PNGs.
- `pip install pyrealsense2==2.58.4` fails; use `==2.58.4.10922` (wheels for x86_64 and aarch64).

## Decisions
- Engines: RTAB-Map first, Open3D reconstruction system second, ORB-SLAM3 only if neither is clean or they disagree.
- Our code: exporter, evaluator (no ground truth), one shared map builder (TSDF + gravity), glue.
  Own tracking/loop code only if a measured gap remains after engine settings and engine offline tools.
- No memory management / Bayes filter in our code. RTAB-Map's time/memory limits are off by default
  (thresholds 0, confirmed from each run's stored parameters); its loop detector stays as is, plus its
  exhaustive offline loop search.
- Jetson records, Z440 computes; same repo and pinned versions on both. Nothing touches STS v1 / pyslam.
- Numbers decide; targets below were fixed before any result; a second recording is the final test.
- Engines use threads/RANSAC: each finalist runs twice; a difference counts only if larger than the run-to-run spread.

## Steps
- **S0** machine facts (`setup/check_machine.sh`).
- **S1** copy bag to `~/mv2_data/bags/`, sha256 on both machines, read-only; Jetson original untouched.
- **S2** inspect (`bagtools/inspect_db3.py`): counts, rates, device times, lost frames, compression,
  calibration; verdicts incl. "IMU full rate".
- **S3** export (`bagtools/export_db3.py`, `check_export.py`): sensor-level non-real-time playback; nearest-time
  pairing within half a frame period (else flagged); SDK alignment; full IMU; pass = counts match the file exactly.
- **S4** RTAB-Map baseline (Docker `introlab3it/rtabmap:noble`, digest pinned): `rtabmap-rgbd_dataset` with the
  tool's documented example settings (DetectionRate 2, LinearUpdate 0, AngularUpdate 0, STMSize 30) + intermediate
  nodes (a pose for every frame); then `rtabmap-detectMoreLoopClosures` + `rtabmap-globalBundleAdjustment` on a copy.
  300-frame smoke test first. Quick look together in `rtabmap-databaseViewer`. GUI run can be screen-recorded
  for the "watch it work" video; numbers always come from the headless run.
- **S5** evaluator + shared map builder: one trajectory format (TUM, every frame, camera optical frame) with an
  automatic convention check; TSDF 1 cm voxels, 4 cm truncation, depth ≤ ≈3 m (from the noise model of this camera);
  gravity from accelerometer rotated into the map and averaged over the walk; fixed-view renders; metrics below;
  old run A must score clearly bad.
- **S6** Open3D reconstruction system v0.20.0: O1 defaults (our intrinsics, depth_scale 5000), O2 = O1 + SLAC.
- **S7** review together; pick the winner (coverage, plausibility, revisit consistency, sharpness; ties → simpler).
  If none is clean: ORB-SLAM3 as a third opinion and fix the recording procedure before writing mapping code.
- **S8** final map from the winner: shared builder vs engine's own export (keep the better), floater removal,
  optional Open3D colour-map optimisation; outputs `map_mesh.ply`, `map_cloud.ply`, `trajectory.txt`,
  `map_frame.json` (up axis, units, transform, source run).
- **S9** held-out recording in a different space (still 3 s at start and end on a marked spot), same pipeline,
  zero parameter changes.
- Later: run on the Jetson, 848×480 depth test, STS integration, anchor (M2).

## Engine runs
| Run | What |
|---|---|
| R1 | RTAB-Map, documented dataset settings + all-frame poses |
| R2 | R1 + exhaustive loop search + global bundle adjustment |
| O1 / O2 | Open3D reconstruction defaults / + SLAC |
| S1 / S2 | ORB-SLAM3 RGB-D / RGB-D-Inertial (only if needed; S2 only with full-rate IMU) |

## Metrics and "clean" targets (fixed now)
| Metric | Catches | Clean |
|---|---|---|
| frames with a pose, one connected map | tracking loss, split maps | 100 %, one map |
| speed between frames | jumps | none > 1.5 m/s |
| rotation over 0.5 s windows vs gyro | wrong rotations | 99 % ≤ 1°, max ≤ 3° |
| revisit pairs (found by appearance, measured directly, compared with trajectory) | drift, bad loops | median ≤ 2 cm / 0.5°, p90 ≤ 5 cm / 1.5° |
| first vs last view (if they overlap) | total drift | ≤ 5 cm |
| plane spread multi-frame ÷ single-frame | doubled walls, smear | ≤ 1.5 |
| floor tilt / flatness after gravity alignment | bent map, wrong up | ≤ 1° / ≤ 1 cm |
| agreement between engines (evo), runtime | — | reported |

Basis: D435 depth noise ≈0.5 cm at 1 m, ≈2 cm at 2 m.

## Pinned versions
Python 3.10 · pyrealsense2 2.58.4.10922 · Open3D 0.20.0 (`open3d-cpu` on x86_64) · RTAB-Map Docker `noble`
(digest pinned on first pull) · ORB-SLAM3 commit 4452a3c (only if needed, GPLv3) · evo 1.38.0 · rosbags 0.11.7 ·
zstandard 0.25.0.

## Working agreement
Code arrives by git push; the user only pulls. Every batch is tested here first on synthetic data made with the
same SDK; anything touching hardware gets a short real-hardware check first. The user pastes outputs back.

## Known risks
- Accelerometer bias can tilt "up" by up to ~1°; a full loop cancels most of it; the floor check reports the rest.
- If start and end views don't overlap, revisit pairs carry the drift measurement.
- Disk ≈35 GB of the 40 GB budget; the Z440 bag copy can be deleted after the export passes.
