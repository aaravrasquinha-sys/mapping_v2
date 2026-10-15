#!/usr/bin/env bash
# Run Open3D's reconstruction system (v0.19.0) on an exported dataset, then convert, evaluate, fuse.
#
#   O1 = make fragments, register, refine, integrate (defaults; our intrinsics; depth scale 5000)
#   O2 = O1 + SLAC (non-rigid correction). Its rigid poses are evaluated like any engine;
#        its own deformed point cloud is kept as native_slac_pointcloud.ply.
#
# Usage:
#   bash engines/open3d_recon/run_open3d.sh DATASET RUNS_ROOT [--slac] [--max-frames N] [--no-fuse]
# Needs: setup_open3d.sh done once; the project venv active (for convert/evaluate/fuse).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
VENV="${O3D_VENV:-$HOME/venvs/o3d019}"
TOOLS="${O3D_TOOLS:-$HOME/mv2_data/tools/open3d-v0.19.0}"
RS="$TOOLS/examples/python/reconstruction_system"

DS="" ROOT="" SLAC=0 MAXF=0 FUSE=1
while [ $# -gt 0 ]; do
  case "$1" in
    --slac) SLAC=1 ;;
    --max-frames) MAXF="$2"; shift ;;
    --no-fuse) FUSE=0 ;;
    -h|--help) sed -n 2,11p "$0"; exit 0 ;;
    *) if [ -z "$DS" ]; then DS="$1"; elif [ -z "$ROOT" ]; then ROOT="$1"; else echo "unknown argument $1"; exit 1; fi ;;
  esac
  shift
done
[ -n "$DS" ] && [ -n "$ROOT" ] || { sed -n 2,11p "$0"; exit 1; }
[ -f "$RS/run_system.py" ] && [ -x "$VENV/bin/python" ] || { echo "ERROR: run engines/open3d_recon/setup_open3d.sh first"; exit 1; }
python -c "import open3d, pyrealsense2" 2>/dev/null || { echo "ERROR: activate the project venv first: source ~/venvs/mv2/bin/activate"; exit 1; }
DS="$(cd "$DS" && pwd)"
mkdir -p "$ROOT"; ROOT="$(cd "$ROOT" && pwd)"

STAMP="$(date +%Y%m%d-%H%M)"
TAG="$( [ "$MAXF" -gt 0 ] && echo "_smoke$MAXF" || true )"
VARIANT="$( [ "$SLAC" -eq 1 ] && echo O2 || echo O1 )"
RUN="$ROOT/${STAMP}_open3d_${VARIANT}${TAG}"
WORK="$RUN/work"
mkdir -p "$WORK/rgb" "$WORK/depth"

# work folder: links to the (read-only) dataset; Open3D writes fragments/ and scene/ here
mapfile -t FILES < <(cd "$DS/rgb" && printf '%s\n' *.png | sort)
[ "$MAXF" -gt 0 ] && FILES=("${FILES[@]:0:$MAXF}")
for f in "${FILES[@]}"; do ln -s "$DS/rgb/$f" "$WORK/rgb/$f"; ln -s "$DS/depth/$f" "$WORK/depth/$f"; done
echo "Open3D reconstruction ($VARIANT) on ${#FILES[@]} frames -> $RUN"

python - "$RUN/config.json" "$WORK" "$DS/intrinsic.json" <<'EOF'
import json, sys
out, work, intr = sys.argv[1:]
cfg = {"name": "mapping_v2", "path_dataset": work + "/", "path_intrinsic": intr,
       "depth_scale": 5000.0,          # our depth PNGs: 5000 units per metre
       "depth_max": 3.0,               # Open3D default; D435 noise reaches ~4 cm there
       "voxel_size": 0.05, "depth_diff_max": 0.07,
       "preference_loop_closure_odometry": 0.1, "preference_loop_closure_registration": 5.0,
       "tsdf_cubic_size": 3.0, "icp_method": "color", "global_registration": "ransac",
       "python_multi_threading": True}
json.dump(cfg, open(out, "w"), indent=2)
EOF

STEPS=(--make --register --refine --integrate)
[ "$SLAC" -eq 1 ] && STEPS+=(--slac --slac_integrate)
T0=$(date +%s)
( cd "$RS" && "$VENV/bin/python" run_system.py --config "$RUN/config.json" "${STEPS[@]}" ) > "$RUN/open3d.log" 2>&1 \
  || { echo "ERROR: Open3D reconstruction failed (see $RUN/open3d.log)"; tail -20 "$RUN/open3d.log"; exit 1; }
T1=$(date +%s)
grep -A8 "Elapsed time" "$RUN/open3d.log" || true

LOG="$WORK/scene/trajectory.log"
if [ "$SLAC" -eq 1 ]; then
  LOG="$(ls "$WORK"/slac/*/optimized_trajectory_slac.log | head -1)"
  P="$(ls "$WORK"/slac/*/output_slac_pointcloud.ply 2>/dev/null | head -1 || true)"
  [ -n "$P" ] && cp "$P" "$RUN/native_slac_pointcloud.ply"
fi
python "$REPO/eval/open3d_log_to_tum.py" "$WORK" "$LOG" "$RUN/open3d_traj.tum"
ENGINE_VER="$("$VENV/bin/python" -c 'import open3d; print(open3d.__version__)')"
python - "$RUN" "$VARIANT" $((T1 - T0)) "$ENGINE_VER" "$DS" "$MAXF" "$(git -C "$TOOLS" rev-parse HEAD)" "${STEPS[@]}" <<'EOF'
import json, sys
d, variant, secs, ver, ds, maxf, commit, *steps = sys.argv[1:]
json.dump({"engine": "open3d", "variant": variant, "runtime_s": int(secs), "open3d_version": ver,
           "scripts_commit": commit, "dataset": ds, "max_frames": int(maxf) or None, "steps": steps,
           "config": json.load(open(f"{d}/config.json"))}, open(f"{d}/run.json", "w"), indent=2)
EOF
python "$REPO/eval/convert_traj.py" "$DS" "$RUN/open3d_traj.tum" "$RUN"
python "$REPO/eval/evaluate.py" "$DS" "$RUN"
[ "$FUSE" -eq 1 ] && python "$REPO/eval/fuse.py" "$DS" "$RUN"
rm -rf "$WORK/rgb" "$WORK/depth"      # links only; fragments/ and scene/ are kept
echo "Done. Results table: $ROOT/results.csv"
