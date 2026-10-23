# RUNBOOK: deploy mapping_v2 on the HP Z440 and map walk1 (from scratch)

Machine: HP Z440 (`slam@slam`), Ubuntu 22.04.3. The recording (`walk1.db3`, 11.2 GB, 240 s) is still
on the Jetson (`drdo@192.168.1.101`, `~/bags/walk1.db3`). This runbook gets from "empty Z440" to
"map + comparison + video" and tells you exactly what to send back.

Rules that hold throughout
- The Jetson only records and is only read from here. **Do not delete or move the bag on the Jetson** until
  the final map has been accepted.
- Nothing here touches `spatial_tracking_system_v1.0` or pyslam.
- Everything long runs inside `tmux`, so a dropped SSH session or a closed terminal does not kill it.
- Every step is safe to repeat. If anything stops, run the same command again.

---

## 0. Before you start (5 min)

| Need | Check |
|---|---|
| About **45 GB free** on the home disk (35 GB is the minimum) | `df -h ~` |
| Internet from the Z440 to github.com, ubuntu archives, pypi.org | `curl -sI https://github.com \| head -1` |
| `sudo` password for user `slam` | `sudo -v` |
| The Z440 can reach the Jetson, which is powered on and idle | `ping -c2 192.168.1.101` |
| The Z440 will not go to sleep during the run | see below |

```bash
# stop the Z440 from sleeping during a multi-hour run (undo later: sudo systemctl unmask sleep.target suspend.target hibernate.target)
sudo systemctl mask sleep.target suspend.target hibernate.target
sudo apt update && sudo apt install -y git tmux rsync
export JETSON=drdo@192.168.1.101        # change if the Jetson's address is different
[ -f ~/.ssh/id_ed25519 ] || ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519    # only if you have no ssh key yet (no questions asked)
ssh-copy-id $JETSON                     # one-time: no more password prompts for ssh/rsync
ssh $JETSON 'ls -l ~/bags/walk1.db3; df -h ~ | tail -1'    # must show the bag, 11177476096 bytes
tmux new -s mv2                         # everything below runs in here
#   detach: Ctrl-b then d      come back: tmux attach -t mv2
```

If the Jetson check fails: wrong address (`ip a` on the Jetson), Jetson off, or the bag has another
path (`ls ~/bags`). Fix that first. If the two machines cannot be networked, use the USB fallback in step 3.

## 1. Get the code (1 min)

```bash
git clone https://github.com/aaravrasquinha-sys/mapping_v2 ~/mv2
cd ~/mv2 && git log --oneline | head -1
bash setup/check_machine.sh        # paste this output to me
```

Later updates: `cd ~/mv2 && git pull` (never edit files in `~/mv2` by hand; if I send a patch, `git pull` applies it).

## 2a. Add swap first (2 min; this Z440 has only 3.7 GB of RAM, the compilers need more)

The machine check showed 3.7 GB RAM. Swap is slow memory on the disk; it lets the compilers and the engines finish
instead of being killed. The install script refuses to start without it. Do it once:
```bash
sudo fallocate -l 16G /swapfile_mv2 && sudo chmod 600 /swapfile_mv2 && sudo mkswap /swapfile_mv2 && sudo swapon /swapfile_mv2
echo '/swapfile_mv2 none swap sw 0 0' | sudo tee -a /etc/fstab      # keep it after a reboot
free -h                                                            # Swap: should show 16Gi
```
Close the browser and other desktop programs before the long steps (they were using 2.6 GB). If you can add RAM to
16 GB or more (the Z440 takes DDR4 ECC modules), everything gets faster and the swap is only a safety net; it is not
required.

## 2. Install the engines (1.5-3 h on this machine: one compiler at a time; start it first, copy the bag meanwhile)

Open a second tmux window (`Ctrl-b c`) for step 3 so both run together.

```bash
cd ~/mv2
bash setup/install_engines.sh
```
- Asks for the sudo password once at the start (apt packages), then builds g2o, RTAB-Map, Pangolin, ORB-SLAM3 and the
  Open3D 0.19.0 environment. All of it lands in `~/mv2_tools` and `~/venvs/o3d019`; nothing is installed system-wide
  except apt packages.
- **Done when** it prints `RTAB-Map OK`, `ORB-SLAM3 OK` and `install_engines: done`.
- ORB-SLAM3 needs about 6 GB of RAM per compiler job; the script picks the job count from free RAM. If it says
  `Killed` or `ERROR: ORB-SLAM3 build failed`, add swap and run the same command again:
  `sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile`

Then the Python environment and self-test (about 5 min):
```bash
bash setup/make_venv.sh                   # creates ~/venvs/mv2 (Python 3.10, pinned versions)
source ~/venvs/mv2/bin/activate
bash tests/run_bagtools_tests.sh          # must end with: ALL TESTS PASSED
```
If `make_venv.sh` says it cannot create a venv: `sudo apt install -y python3.10-venv`, then run it again.

## 3. Copy the bag from the Jetson (11.2 GB; 3-20 min on a wired LAN, longer over Wi-Fi)

Do this in the second tmux window while step 2 compiles.

```bash
export JETSON=drdo@192.168.1.101
mkdir -p ~/mv2_data/bags && cd ~/mv2_data/bags
# 1. fingerprint made ON the Jetson (runs in the background while the copy goes)
ssh $JETSON 'sha256sum ~/bags/walk1.db3' > walk1.jetson.sha256 &
# 2. copy; if it is interrupted, run this same line again and it continues
rsync -ah --partial --inplace --info=progress2 $JETSON:bags/walk1.db3 .
wait
# 3. compare
cat walk1.jetson.sha256
sha256sum walk1.db3
```
The two hashes **must be identical**. Then lock the copy:
```bash
chmod a-w ~/mv2_data/bags/walk1.db3
```
- Hash differs: copy again (`rsync ... --checksum`), compare again. Never process an unverified copy.
- `rsync: command not found` on the Jetson: `scp $JETSON:bags/walk1.db3 .` instead.
- **No network between the machines:** on the Jetson `cp ~/bags/walk1.db3 /media/<usb>/` and
  `sha256sum ~/bags/walk1.db3 > /media/<usb>/walk1.jetson.sha256`; on the Z440 copy both files into
  `~/mv2_data/bags/` and run the same compare.

### 3b. Optional: watch the recording (needs step 2 finished and `bash setup/make_venv.sh` done)
```bash
cd ~/mv2 && source ~/venvs/mv2/bin/activate
python bagtools/preview_db3.py ~/mv2_data/bags/walk1.db3 ~/mv2_data/walk1.preview.mp4     # a few minutes
xdg-open ~/mv2_data/walk1.preview.mp4          # on the Z440's own screen; or copy the mp4 to your laptop
```
Colour on the left, depth on the right (blue near, red far, black no depth), time in the corner. Look for: blur when
turning, shaky stretches, blank or glaring areas, the person or objects moving, depth holes. Options: `--max-seconds 30`
for a quick look, `--speed 4` to watch it fast, `--step 1` for every frame.

## 4. Verify everything in one go (1 min)

```bash
cd ~/mv2 && source ~/venvs/mv2/bin/activate
bash setup/verify_install.sh ~/mv2_data/bags/walk1.db3
```
**Must end with `VERIFY: ALL OK`** (a `WARN` on the OS or disk is fine; any `FAIL` tells you the command that fixes it).
Paste the output to me.

## 5. Smoke test: the whole pipeline on the first 300 frames (about 5-10 min)

```bash
bash run_all.sh ~/mv2_data/bags/walk1.db3 --smoke 300 --engines "rtabmap orbslam3"
cat ~/mv2_data/runs/SUMMARY_walk1_smoke300.txt
```
What good looks like
- Every line in the summary starts with `OK` (or `skip`); no `FAILED`.
- `export check` printed `RESULT: PASS`.
- The first 10 seconds contain no revisits, so the `revisit ...` rows and `start-end` showing `n/a` is expected.
  `ATE (GT)` is `n/a` for every real recording (there is no ground truth).
- Look at `~/mv2_data/datasets/walk1_smoke300/contact_sheet.jpg`: left colour, right depth (blue near, red far,
  black no depth). Colour should look normal, depth should match the scene.

Paste the summary and send me the contact sheet. **Stop here if anything is `FAILED`** and send me the
log named on that line (`~/mv2_data/logs/walk1_smoke300/...`).

Remove the smoke data afterwards (small, and the real comparison ignores it, but it clutters the folders):
```bash
cd ~/mv2_data/runs && for d in */; do grep -q '"dataset": ".*walk1_smoke300' "$d/metrics.json" 2>/dev/null && rm -rf "$d"; done
chmod -R u+w ~/mv2_data/datasets/walk1_smoke300
rm -rf ~/mv2_data/datasets/walk1_smoke300 ~/mv2_data/final/walk1_smoke300 ~/mv2_data/logs/walk1_smoke300 \
       ~/mv2_data/runs/_cache/walk1_smoke300 ~/mv2_data/runs/*walk1_smoke300*
```

## 6. The real run

Two stages, so you get a first map and the numbers early. Both are the same command; the second one only adds
what is missing.

**Stage A: export, checks, RTAB-Map R1+R2, ORB-SLAM3 (about 1-2.5 h)**
```bash
cd ~/mv2 && source ~/venvs/mv2/bin/activate
bash run_all.sh ~/mv2_data/bags/walk1.db3 --engines "rtabmap orbslam3"
```
What happens: inspect the bag, export all 7000+ frames and every IMU sample, check the export against the bag
(a failed check stops everything), find revisits, run the engines, compare, pick, build the final map and a replay
video. Progress: `tail -f ~/mv2_data/logs/walk1/*.log` in another tmux window.

You should now have:
- `~/mv2_data/runs/SUMMARY_walk1.txt`  (the table; paste it to me)
- `~/mv2_data/runs/compare_walk1.png`  (every run's map, stacked)
- `~/mv2_data/final/walk1/`  (provisional final map, `views.png`, `replay.mp4`)

**Stage B: add Open3D (slow; start it in the evening or overnight, it is optional)**
```bash
bash run_all.sh ~/mv2_data/bags/walk1.db3
```
Open3D's reconstruction measured 72 min for 979 frames on 2 CPU threads, so for walk1 expect roughly
`9 h / (cores / 2)`: about 2-3 h on 8 cores, 4-5 h on 4 cores. It runs last; the final map is rebuilt only if it
becomes the recommended run. Add `--slac` for its non-rigid variant (roughly double). Skip it entirely if Stage A
already gives a clean map.

## 7. What to send me

```bash
cat ~/mv2_data/runs/SUMMARY_walk1.txt
```
plus the files `~/mv2_data/runs/compare_walk1.png`, `~/mv2_data/final/walk1/views.png`, and
`~/mv2_data/final/walk1/replay.mp4` (if it is large, `views.png` and the table are enough at first).

and the bag inspection (stream rates, IMU rate verdict, frame gaps):
`tail -25 ~/mv2_data/logs/walk1/1_inspect.log`

How to read the table: every row has a fixed target (PLAN.md) and a tick or cross. A run that misses nothing is
"clean". The recommendation is only a proposal from the numbers; we look at the views together before calling a
map final.

## 8. Look at the map yourself

```bash
sudo apt install -y meshlab
meshlab ~/mv2_data/final/walk1/map_mesh_view.ply
```
Frame: metres, +z up (gravity), floor at z = 0. Full-resolution mesh is `map_mesh.ply`, points are `map_cloud.ply`,
camera path is `trajectory_map.tum`, provenance and checksums are `MANIFEST.json`.

---

## If something goes wrong

| Symptom | Cause / fix |
|---|---|
| `install_engines.sh`: network error, `Could not resolve host` | The Z440 cannot reach github.com / ubuntu / pypi. Tell me which host; do not work around it by hand. |
| ORB-SLAM3 build `Killed` | Out of RAM: add swap (step 2) and run the command again. |
| `rsync` stops halfway | Run the same line again; it continues. Always finish with the hash comparison. |
| Hashes differ | Copy again with `--checksum`; never run on an unverified copy. |
| `export check ... RESULT: FAIL` | The export differs from the bag. Do not continue. Send me `~/mv2_data/logs/walk1/2_check.log`. To redo: `chmod -R u+w ~/mv2_data/datasets/walk1 && rm -rf ~/mv2_data/datasets/walk1`, then run again. |
| `FAILED RTAB-Map ...` or another engine | The other engines still ran. Send me the log named on the line. Delete that run's folder in `~/mv2_data/runs/` if it exists, then run the same command again. |
| Z440 rebooted or the terminal was closed (no tmux) | Nothing finished is lost. Open a new `tmux`, `cd ~/mv2 && source ~/venvs/mv2/bin/activate`, run the same `run_all.sh` command; it skips what is done. |
| Something says `Killed` or the machine freezes | Out of memory: check `free -h` (Swap must show 16Gi), close other programs, run the same command again. |
| Disk full | `df -h ~`. The bag copy (11 GB) can be deleted only after the export passed AND the Jetson copy is still there; the dataset is ~10 GB. |
| Everything has `n/a` in the revisit rows | The walk has no place seen twice 20 s apart. Say so and we use the other checks. |
| Window closed / SSH dropped | `tmux attach -t mv2`. Jobs kept running. |

Start completely over (keeps the bag): `chmod -R u+w ~/mv2_data/datasets; rm -rf ~/mv2_data/{datasets,runs,final,logs}`.
Re-do one engine only: delete its folder in `~/mv2_data/runs/` and run the same `run_all.sh` command.

## Expected total effort (measured on 2 CPU threads with 7 GB RAM; this Z440 has 4 cores but only 3.7 GB RAM, so expect about the same, up to 1.5x slower)

| Phase | Time |
|---|---|
| add swap | 2 min (once) |
| install engines + venv | 1.5-3 h with 4 GB RAM (once; one compiler at a time) |
| copy bag | 2-15 min |
| smoke test | 5-10 min |
| Stage A (export, revisits 23 min, RTAB-Map 64 min, ORB-SLAM3 28 min, map, video) | about 2.5 h |
| Stage B (Open3D O1) | 4-6 h on 4 cores (optional; its memory use at full length is untested, with 4 GB RAM consider skipping it) |

Disk: bag 11 GB + dataset about 10 GB + engines/tools about 5 GB + runs about 5 GB + final map about 1 GB + swap 16 GB
= about 48 GB; this machine has 395 GB free, so no problem.

## After the map is accepted

Keep: `~/mv2_data/final/walk1/` and the bag. Safe to delete to save space: `~/mv2_data/datasets/walk1` (can be
recreated from the bag with one command), `~/mv2_tools/src`. Only then consider freeing the Jetson copy, and only
after a second copy of the bag exists somewhere else. STS integration and the map anchor are deliberately not
started until the map is accepted.

## 9. Several bags unattended (e.g. over a weekend): `queue.sh`

```bash
# once, before leaving (needs sudo): no sleep, no systemd-oomd, swap on
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
sudo systemctl disable --now systemd-oomd
swapon --show                                   # must list the 19 GB swap
cd ~/mv2 && git pull && source ~/venvs/mv2/bin/activate
chmod a-w ~/mv2_data/bags/*.db3
setsid nohup bash queue.sh ~/mv2_data/bags/corridor.db3 ~/mv2_data/bags/floor1.db3 ~/mv2_data/bags/floor2.db3 \
  > ~/queue.log 2>&1 < /dev/null &
tail -f ~/queue.log                             # first lines: RAM/swap/disk checks; Ctrl-C only stops watching
```

- Default is RTAB-Map R1 only (`--engines rtabmap --no-r2 --no-revisits --no-video`); other run_all options go
  after `--`. Each bag gets up to 3 attempts; a failed bag does not stop the next.
- It refuses to start with less than 4 GB swap or too little disk (each bag's export needs about the bag's size).
- Memory: the map builder starts at 1 cm voxels and switches to 1.5 / 2 / 3 / 4 cm by itself if the map would not
  fit in RAM (measured: about 0.35 GB + 40 KB per 8 cm voxel block; 1 cm needs about 150 blocks per m^2 of
  surface). On this 3.7 GB machine about 64k blocks fit at 1 cm, i.e. one large room or a ~50 m corridor;
  a whole floor will come out at 2-3 cm. The voxel used is in `map_frame.json` and the final README.
- Every step in `SUMMARY_<name>.txt` shows its peak memory. If a map step died, the next attempt rebuilds only the
  map (no second RTAB-Map run).
- Results: `~/mv2_data/runs/QUEUE_<date>.txt` (one line per bag + each bag's summary), maps in `~/mv2_data/final/`.

## 10. RTAB-Map study of one recording (R1 + R2 + variants + comparison report): `rtabmap_study.sh`

```bash
setsid nohup bash rtabmap_study.sh ~/mv2_data/bags/corridor1_slow.db3 > ~/study.log 2>&1 < /dev/null &
```
Runs queue.sh (export, R1 + R2, final map; retries), then R1 variants (default `d7` = `Vis/MaxDepth 7`; RTAB-Map's
own default is 0 = no depth limit), then `eval/rtabmap_report.py` on every run. Results in
`~/mv2_data/final/<name>/rtabmap_report/`: `rtabmap_comparison.md`, and per run `report.md` (resets with times,
frames without pose, loop-closure time pairs, legs/turn, positions every 60 s, the spin, points below the floor),
`windows.csv` (per 5 s), `path.png`, `corridor_views.png`, `views.png`, `run.json` (exact parameters).
R2 survives a failed bundle adjustment (exported with --opt 0, marked FAILED in run.json).
