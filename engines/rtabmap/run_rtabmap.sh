#!/usr/bin/env bash
# Run RTAB-Map on an exported dataset, then convert, evaluate and fuse.
#
#   R1 = rtabmap-rgbd_dataset with the tool's documented example settings
#        (DetectionRate 2, LinearUpdate 0, AngularUpdate 0, STMSize 30) + intermediate nodes
#        (a pose for every frame). Trajectory: <run>/rtabmap_poses.txt (all frames).
#   R2 = R1 database + rtabmap-detectMoreLoopClosures + rtabmap-globalBundleAdjustment,
#        trajectory from rtabmap-export --poses_camera (all frames).
#
# The dataset needs rtabmap_calib.yaml (written by the exporter): rtabmap-rgbd_dataset prints
# "Dataset ... is not supported" for any non-TUM name and then loads that file. Expected.
#
# Usage:
#   bash engines/rtabmap/run_rtabmap.sh DATASET RUNS_ROOT [--r2] [--max-frames N] [--native]
#        [--docker] [--image IMAGE] [--no-fuse]
#   --r2           also make the R2 run
#   --max-frames N smoke test on the first N frames
#   --native       native RTAB-Map: $RTABMAP_BIN, else ~/mv2_tools/rtabmap/bin (setup/install_engines.sh),
#                  else PATH. This is the default when ~/mv2_tools/rtabmap/bin exists.
#   --docker       force Docker; --image IMAGE (default introlab3it/rtabmap:noble)
# Needs the Python venv active (python with mapping_v2 requirements) for the evaluation steps.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

DEFAULT_BIN="${MV2_TOOLS:-$HOME/mv2_tools}/rtabmap/bin"
NATIVE=0; { [ -n "${RTABMAP_BIN:-}" ] || [ -x "$DEFAULT_BIN/rtabmap-rgbd_dataset" ]; } && NATIVE=1
DS="" ROOT="" R2=0 MAXF=0 FUSE=1 IMAGE="${RTABMAP_IMAGE:-introlab3it/rtabmap:noble}"
while [ $# -gt 0 ]; do
  case "$1" in
    --r2) R2=1 ;;
    --max-frames) MAXF="$2"; shift ;;
    --native) NATIVE=1 ;;
    --docker) NATIVE=0 ;;
    --image) IMAGE="$2"; NATIVE=0; shift ;;
    --no-fuse) FUSE=0 ;;
    -h|--help) sed -n 2,24p "$0"; exit 0 ;;
    *) if [ -z "$DS" ]; then DS="$1"; elif [ -z "$ROOT" ]; then ROOT="$1"; else echo "unknown argument $1"; exit 1; fi ;;
  esac
  shift
done
[ -n "$DS" ] && [ -n "$ROOT" ] || { sed -n 2,24p "$0"; exit 1; }
DS="$(cd "$DS" && pwd)"
mkdir -p "$ROOT"; ROOT="$(cd "$ROOT" && pwd)"
[ -f "$DS/rtabmap_calib.yaml" ] && [ -d "$DS/rgb_sync" ] && [ -d "$DS/depth_sync" ] \
  || { echo "ERROR: $DS lacks rtabmap_calib.yaml / rgb_sync / depth_sync (not an exported dataset?)"; exit 1; }
case "$(basename "$DS")" in *freiburg*|*rgbd_bonn*)
  echo "ERROR: dataset folder name contains 'freiburg' or 'rgbd_bonn': RTAB-Map would use TUM/Bonn calibration. Rename it."; exit 1 ;;
esac
python -c "import open3d, pyrealsense2" 2>/dev/null || { echo "ERROR: activate the venv first: source ~/venvs/mv2/bin/activate"; exit 1; }

STAMP="$(date +%Y%m%d-%H%M)"
TAG="$( [ "$MAXF" -gt 0 ] && echo "_smoke$MAXF" || true )"
RUN1="$ROOT/${STAMP}_rtabmap_R1${TAG}"
mkdir -p "$RUN1"

# ---- how to call RTAB-Map tools (native or Docker; same paths inside and outside the container)
if [ "$NATIVE" -eq 1 ]; then
  BIN="${RTABMAP_BIN:-}"
  [ -z "$BIN" ] && [ -x "$DEFAULT_BIN/rtabmap-rgbd_dataset" ] && BIN="$DEFAULT_BIN"
  [ -n "$BIN" ] || command -v rtabmap-rgbd_dataset >/dev/null \
    || { echo "ERROR: no native RTAB-Map (run: bash setup/install_engines.sh rtabmap), or use --docker"; exit 1; }
  rt() { local tool="$1"; shift; "${BIN:+$BIN/}$tool" "$@"; }
  RT_VERSION="native $(rt rtabmap-console -v 2>/dev/null | tail -1 || true)"
  IMAGE_ID="native ${BIN:-PATH}"
else
  command -v docker >/dev/null || { echo "ERROR: docker not found (install it, or use --native with RTAB-Map installed)"; exit 1; }
  docker image inspect "$IMAGE" >/dev/null 2>&1 || docker pull "$IMAGE"
  IMAGE_ID="$(docker image inspect --format '{{index .RepoDigests 0}}' "$IMAGE" 2>/dev/null || echo "$IMAGE")"
  rt() { docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp \
           -v "$DS:$DS:ro" -v "$ROOT:$ROOT" -w "$ROOT" "$IMAGE" "$@"; }
  RT_VERSION="$(rt rtabmap-console -v 2>/dev/null | tail -1 || true)"
fi
VERS="$(rt rtabmap-rgbd_dataset --version 2>&1 || true)"
grep -q "^RTAB-Map:" <<< "$VERS" || { echo "ERROR: rtabmap-rgbd_dataset does not run: $(tail -2 <<< "$VERS")"; exit 1; }
echo "RTAB-Map $RT_VERSION  ($IMAGE_ID)"

# ---- dataset to process (full, or a subset of symlinks for the smoke test)
INPUT="$DS"
if [ "$MAXF" -gt 0 ]; then
  INPUT="$RUN1/subset"
  mkdir -p "$INPUT/rgb_sync" "$INPUT/depth_sync"
  cp "$DS/rtabmap_calib.yaml" "$INPUT/"
  mapfile -t FILES < <(cd "$DS/rgb" && printf '%s\n' *.png | sort)
  for f in "${FILES[@]:0:$MAXF}"; do
    ln -s "$DS/rgb/$f" "$INPUT/rgb_sync/$f"; ln -s "$DS/depth/$f" "$INPUT/depth_sync/$f"
  done
  echo "Smoke test: first $MAXF frames"
fi

PARAMS=(--Rtabmap/DetectionRate 2 --RGBD/LinearUpdate 0 --RGBD/AngularUpdate 0 --Mem/STMSize 30
        --Rtabmap/CreateIntermediateNodes true)

# ---- R1
echo "R1: rtabmap-rgbd_dataset ... (log: $RUN1/rtabmap.log)"
T0=$(date +%s)
rt rtabmap-rgbd_dataset --output "$RUN1" --quiet "${PARAMS[@]}" "$INPUT" > "$RUN1/rtabmap.log" 2>&1 || true
T1=$(date +%s)
grep -q "Camera init failed" "$RUN1/rtabmap.log" && { echo "ERROR: RTAB-Map could not read the dataset/calibration (see rtabmap.log)"; exit 1; }
[ -s "$RUN1/rtabmap_poses.txt" ] || { echo "ERROR: no trajectory written (see $RUN1/rtabmap.log)"; tail -20 "$RUN1/rtabmap.log"; exit 1; }
rt rtabmap-info "$RUN1/rtabmap.db" > "$RUN1/rtabmap_info.txt" 2>&1 || true

write_runjson() {  # dir variant seconds params...
  local dir="$1" variant="$2" secs="$3"; shift 3
  python - "$dir" "$variant" "$secs" "$RT_VERSION" "$IMAGE_ID" "$DS" "$MAXF" "$@" <<'EOF'
import json, sys
d, variant, secs, ver, image, ds, maxf, *params = sys.argv[1:]
json.dump({"engine": "rtabmap", "variant": variant, "runtime_s": int(secs), "rtabmap_version": ver,
           "image": image, "dataset": ds, "max_frames": int(maxf) or None, "parameters": params},
          open(f"{d}/run.json", "w"), indent=2)
EOF
}
write_runjson "$RUN1" R1 $((T1 - T0)) "${PARAMS[@]}"
grep -E "Processing|Iteration .*/|Total time|translational_rmse|rotational_rmse" "$RUN1/rtabmap.log" | tail -4 || true

finish() {  # run_dir engine_traj
  python "$REPO/eval/convert_traj.py" "$DS" "$2" "$1"
  python "$REPO/eval/evaluate.py" "$DS" "$1"
  if [ "$FUSE" -eq 1 ]; then python "$REPO/eval/fuse.py" "$DS" "$1"; fi
}
finish "$RUN1" "$RUN1/rtabmap_poses.txt"

# ---- R2
if [ "$R2" -eq 1 ]; then
  RUN2="$ROOT/${STAMP}_rtabmap_R2${TAG}"
  mkdir -p "$RUN2"
  cp "$RUN1/rtabmap.db" "$RUN2/rtabmap.db"
  echo "R2: detectMoreLoopClosures + globalBundleAdjustment (log: $RUN2/postprocess.log)"
  T0=$(date +%s)
  { rt rtabmap-detectMoreLoopClosures "$RUN2/rtabmap.db"
    rt rtabmap-globalBundleAdjustment "$RUN2/rtabmap.db"
    rt rtabmap-export --poses_camera --poses_format 10 --opt 2 --output_dir "$RUN2" --output r2 "$RUN2/rtabmap.db"
  } > "$RUN2/postprocess.log" 2>&1
  T1=$(date +%s)
  grep -E "total loop closures|bundle adjustment... done|camera poses exported" "$RUN2/postprocess.log" || true
  write_runjson "$RUN2" R2 $(( $(python -c "import json;print(json.load(open('$RUN1/run.json'))['runtime_s'])") + T1 - T0 )) \
    "${PARAMS[@]}" "+detectMoreLoopClosures" "+globalBundleAdjustment"
  finish "$RUN2" "$RUN2/r2_camera_poses.txt"
fi
echo "Done. Results table: $ROOT/results.csv"
