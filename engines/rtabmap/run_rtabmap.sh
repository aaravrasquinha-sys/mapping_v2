#!/usr/bin/env bash
# Run RTAB-Map on an exported dataset, then convert, evaluate and fuse.
#
#   R1 = rtabmap-rgbd_dataset with the tool's documented example settings
#        (DetectionRate 2, LinearUpdate 0, AngularUpdate 0, STMSize 30) + intermediate nodes
#        (a pose for every frame) + Odom/ResetCountdown 1 (restart odometry after a tracking loss; the
#        sessions are joined by loop closures). Trajectory: rtabmap-export --opt 0 (all nodes).
#   R2 = R1 database + rtabmap-detectMoreLoopClosures + rtabmap-globalBundleAdjustment,
#        trajectory from rtabmap-export --poses_camera --opt 2 (the bundle-adjusted poses, all frames).
#
# The dataset needs rtabmap_calib.yaml (written by the exporter): rtabmap-rgbd_dataset prints
# "Dataset ... is not supported" for any non-TUM name and then loads that file. Expected.
#
# Usage:
#   bash engines/rtabmap/run_rtabmap.sh DATASET RUNS_ROOT [--r2] [--max-frames N] [--native]
#        [--docker] [--image IMAGE] [--no-fuse] [--r2-only R1_RUN_DIR] [--param NAME VALUE]... [--tag TAG]
#   --r2           also make the R2 run
#   --r2-only DIR  make only R2, from the database of an R1 run that already finished (DIR = its run folder,
#                  e.g. ~/mv2_data/runs/20261016-1604_rtabmap_R1); R1 is not run again
#   --max-frames N smoke test on the first N frames
#   --native       native RTAB-Map: $RTABMAP_BIN, else ~/mv2_tools/rtabmap/bin (setup/install_engines.sh),
#                  else PATH. This is the default when ~/mv2_tools/rtabmap/bin exists.
#   --docker       force Docker; --image IMAGE (default introlab3it/rtabmap:noble)
#   --param N V    extra RTAB-Map parameter on top of R1's (repeatable), e.g. --param Vis/MaxDepth 7
#   --tag TAG      suffix for the run folders (..._rtabmap_R1_TAG), so a variant never mixes with plain R1/R2
# rtabmap.log has one line per frame (odometry inliers/features), used by eval/rtabmap_report.py.
# R2: if a post-processing tool fails (e.g. out of memory in bundle adjustment), R2 is still exported from
# what did finish and run.json says which steps failed.
# Needs the Python venv active (python with mapping_v2 requirements) for the evaluation steps.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

DEFAULT_BIN="${MV2_TOOLS:-$HOME/mv2_tools}/rtabmap/bin"
NATIVE=0; { [ -n "${RTABMAP_BIN:-}" ] || [ -x "$DEFAULT_BIN/rtabmap-rgbd_dataset" ]; } && NATIVE=1
DS="" ROOT="" R2=0 R2ONLY="" MAXF=0 FUSE=1 EXTRA=() UTAG="" IMAGE="${RTABMAP_IMAGE:-introlab3it/rtabmap:noble}"
while [ $# -gt 0 ]; do
  case "$1" in
    --r2) R2=1 ;;
    --r2-only) R2=1; R2ONLY="$2"; shift ;;
    --max-frames) MAXF="$2"; shift ;;
    --native) NATIVE=1 ;;
    --docker) NATIVE=0 ;;
    --image) IMAGE="$2"; NATIVE=0; shift ;;
    --no-fuse) FUSE=0 ;;
    --param) EXTRA+=("--$2" "$3"); shift 2 ;;
    --tag) UTAG="_$2"; shift ;;
    -h|--help) sed -n 2,33p "$0"; exit 0 ;;
    *) if [ -z "$DS" ]; then DS="$1"; elif [ -z "$ROOT" ]; then ROOT="$1"; else echo "unknown argument $1"; exit 1; fi ;;
  esac
  shift
done
[ -n "$DS" ] && [ -n "$ROOT" ] || { sed -n 2,33p "$0"; exit 1; }
DS="$(cd "$DS" && pwd)"
mkdir -p "$ROOT"; ROOT="$(cd "$ROOT" && pwd)"
[ -f "$DS/rtabmap_calib.yaml" ] && [ -d "$DS/rgb_sync" ] && [ -d "$DS/depth_sync" ] \
  || { echo "ERROR: $DS lacks rtabmap_calib.yaml / rgb_sync / depth_sync (not an exported dataset?)"; exit 1; }
case "$(basename "$DS")" in *freiburg*|*rgbd_bonn*)
  echo "ERROR: dataset folder name contains 'freiburg' or 'rgbd_bonn': RTAB-Map would use TUM/Bonn calibration. Rename it."; exit 1 ;;
esac
python -c "import open3d, pyrealsense2" 2>/dev/null || { echo "ERROR: activate the venv first: source ~/venvs/mv2/bin/activate"; exit 1; }

STAMP="$(date +%Y%m%d-%H%M)"
TAG="$( [ "$MAXF" -gt 0 ] && echo "_smoke$MAXF" || true )$UTAG"
RUN1="$ROOT/${STAMP}_rtabmap_R1${TAG}"
if [ -n "$R2ONLY" ]; then
  [ -s "$R2ONLY/rtabmap.db" ] && [ -f "$R2ONLY/run.json" ] \
    || { echo "ERROR: --r2-only: $R2ONLY has no rtabmap.db / run.json (is it a finished R1 run folder?)"; exit 1; }
  RUN1="$(cd "$R2ONLY" && pwd)"
  echo "R2 only: reusing the R1 database in $RUN1"
else
  mkdir -p "$RUN1"
fi

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
        --Rtabmap/CreateIntermediateNodes true --Odom/ResetCountdown 1 ${EXTRA[@]+"${EXTRA[@]}"})
if [ -n "$R2ONLY" ]; then     # R2 from an existing R1: label it with that R1's parameters
  mapfile -t PARAMS < <(python -c "import json,sys; print('\n'.join(json.load(open(sys.argv[1]))['parameters']))" "$RUN1/run.json")
fi

if [ -z "$R2ONLY" ]; then
# ---- R1
echo "R1: rtabmap-rgbd_dataset ... (log: $RUN1/rtabmap.log)"
T0=$(date +%s)
rt rtabmap-rgbd_dataset --output "$RUN1" "${PARAMS[@]}" "$INPUT" > "$RUN1/rtabmap.log" 2>&1 || true
T1=$(date +%s)
grep -q "Camera init failed" "$RUN1/rtabmap.log" && { echo "ERROR: RTAB-Map could not read the dataset/calibration (see rtabmap.log)"; exit 1; }
[ -s "$RUN1/rtabmap.db" ] || { echo "ERROR: no database written (see $RUN1/rtabmap.log)"; tail -20 "$RUN1/rtabmap.log"; exit 1; }
# trajectory: full global optimisation of every node (incl. intermediate ones) from the database.
# (rgbd_dataset's own rtabmap_poses.txt is not written when frames were lost: "stamps and poses maps
#  should have the same size"; after an odometry reset the sessions are joined by loop closures)
rt rtabmap-export --poses_camera --poses_format 10 --opt 0 --output_dir "$RUN1" --output r1 "$RUN1/rtabmap.db" \
  > "$RUN1/export.log" 2>&1 || true
[ -s "$RUN1/r1_camera_poses.txt" ] || { echo "ERROR: no trajectory exported (see $RUN1/export.log, rtabmap.log)"; tail -5 "$RUN1/export.log"; exit 1; }
rt rtabmap-info "$RUN1/rtabmap.db" > "$RUN1/rtabmap_info.txt" 2>&1 || true
grep -E "^(Sessions|Maps in graph|Optimized graph|Global graph):" "$RUN1/rtabmap_info.txt" | sed 's/^/  /' || true
echo "  (more than 1 session = odometry was lost and restarted; 'Maps in graph' lists the sessions joined into the map)"
fi

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
if [ -z "$R2ONLY" ]; then
  write_runjson "$RUN1" R1 $((T1 - T0)) "${PARAMS[@]}"
  grep -E "Processing|Iteration .*/|Total time|translational_rmse|rotational_rmse" "$RUN1/rtabmap.log" | tail -4 || true
fi

finish() {  # run_dir engine_traj
  python "$REPO/eval/convert_traj.py" "$DS" "$2" "$1"
  python "$REPO/eval/evaluate.py" "$DS" "$1"
  if [ "$FUSE" -eq 1 ]; then python "$REPO/eval/fuse.py" "$DS" "$1"; fi
}
[ -z "$R2ONLY" ] && finish "$RUN1" "$RUN1/r1_camera_poses.txt"

# ---- R2
if [ "$R2" -eq 1 ]; then
  RUN2="$ROOT/${STAMP}_rtabmap_R2${TAG}"
  mkdir -p "$RUN2"
  cp "$RUN1/rtabmap.db" "$RUN2/rtabmap.db"
  cp "$RUN1/rtabmap.log" "$RUN2/rtabmap_r1.log" 2>/dev/null || true     # R2's odometry is R1's
  echo "R2: detectMoreLoopClosures + globalBundleAdjustment (log: $RUN2/postprocess.log)"
  T0=$(date +%s)
  PLOG="$RUN2/postprocess.log"; : > "$PLOG"
  STEPS=()
  if rt rtabmap-detectMoreLoopClosures "$RUN2/rtabmap.db" >> "$PLOG" 2>&1; then STEPS+=("+detectMoreLoopClosures")
  else rc=$?; STEPS+=("+detectMoreLoopClosures FAILED (exit $rc)"); echo "WARNING: detectMoreLoopClosures failed (exit $rc)"; fi
  cp "$RUN2/rtabmap.db" "$RUN2/rtabmap.before_ba.db"            # bundle adjustment rewrites the database
  if rt rtabmap-globalBundleAdjustment "$RUN2/rtabmap.db" >> "$PLOG" 2>&1; then
    STEPS+=("+globalBundleAdjustment"); OPT=2; rm -f "$RUN2/rtabmap.before_ba.db"
  else
    rc=$?; STEPS+=("+globalBundleAdjustment FAILED (exit $rc): exported with --opt 0 instead")
    echo "WARNING: globalBundleAdjustment failed (exit $rc; 137 = out of memory); R2 = R1 + extra loop closures"
    mv -f "$RUN2/rtabmap.before_ba.db" "$RUN2/rtabmap.db"; OPT=0
  fi
  rt rtabmap-export --poses_camera --poses_format 10 --opt "$OPT" --output_dir "$RUN2" --output r2 "$RUN2/rtabmap.db" >> "$PLOG" 2>&1
  T1=$(date +%s)
  grep -E "total loop closures|bundle adjustment... done|camera poses exported" "$PLOG" || true
  write_runjson "$RUN2" R2 $(( $(python -c "import json;print(json.load(open('$RUN1/run.json'))['runtime_s'])") + T1 - T0 )) \
    "${PARAMS[@]}" "${STEPS[@]}"
  finish "$RUN2" "$RUN2/r2_camera_poses.txt"
fi
echo "Done. Results table: $ROOT/results.csv"
