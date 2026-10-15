#!/usr/bin/env bash
# Run ORB-SLAM3 RGB-D (S1) on an exported dataset, then convert, evaluate and fuse.
# Every frame is tracked in order (the example paces at the recording rate, so it takes about as
# long as the recording). Output trajectory: CameraTrajectory.txt (TUM, camera frame).
#
# Usage:
#   bash engines/orbslam3/run_orbslam3.sh DATASET RUNS_ROOT [--max-frames N] [--native|--docker] [--image IMG] [--no-fuse]
#   --native   native build: $ORB_SLAM3_DIR (default ~/mv2_tools/ORB_SLAM3, made by setup/install_engines.sh);
#              the default when that build exists
#   --docker   Docker image (--image, default mv2/orbslam3:4452a3c, built from engines/orbslam3/Dockerfile)
# Needs the project venv active.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
TOOLS="${MV2_TOOLS:-$HOME/mv2_tools}"
O="${ORB_SLAM3_DIR:-$TOOLS/ORB_SLAM3}"
NATIVE=0; [ -x "$O/Examples/RGB-D/rgbd_tum" ] && NATIVE=1
DS="" ROOT="" MAXF=0 FUSE=1 IMAGE="${ORBSLAM3_IMAGE:-mv2/orbslam3:4452a3c}"
while [ $# -gt 0 ]; do
  case "$1" in
    --max-frames) MAXF="$2"; shift ;;
    --native) NATIVE=1 ;;
    --docker) NATIVE=0 ;;
    --image) IMAGE="$2"; NATIVE=0; shift ;;
    --no-fuse) FUSE=0 ;;
    -h|--help) sed -n 2,12p "$0"; exit 0 ;;
    *) if [ -z "$DS" ]; then DS="$1"; elif [ -z "$ROOT" ]; then ROOT="$1"; else echo "unknown argument $1"; exit 1; fi ;;
  esac
  shift
done
[ -n "$DS" ] && [ -n "$ROOT" ] || { sed -n 2,12p "$0"; exit 1; }
python -c "import open3d, pyrealsense2" 2>/dev/null || { echo "ERROR: activate the project venv first"; exit 1; }
DS="$(cd "$DS" && pwd)"
mkdir -p "$ROOT"; ROOT="$(cd "$ROOT" && pwd)"
STAMP="$(date +%Y%m%d-%H%M)"
TAG="$( [ "$MAXF" -gt 0 ] && echo "_smoke$MAXF" || true )"
RUN="$ROOT/${STAMP}_orbslam3_S1${TAG}"
mkdir -p "$RUN"

python "$HERE/make_settings.py" "$DS" "$RUN/settings.yaml"
if [ "$MAXF" -gt 0 ]; then head -n "$MAXF" "$DS/associations.txt" > "$RUN/associations.txt" || true
else cp "$DS/associations.txt" "$RUN/associations.txt"; fi
echo "ORB-SLAM3 RGB-D on $(wc -l < "$RUN/associations.txt") frames -> $RUN"

if [ "$NATIVE" -eq 1 ]; then
  [ -x "$O/Examples/RGB-D/rgbd_tum" ] || { echo "ERROR: no ORB-SLAM3 build in $O (run: bash setup/install_engines.sh orbslam3)"; exit 1; }
  export LD_LIBRARY_PATH="$O/lib:$O/Thirdparty/DBoW2/lib:$O/Thirdparty/g2o/lib:$TOOLS/pangolin/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"
  run_orb() { ( cd "$RUN" && "$O/Examples/RGB-D/rgbd_tum" "$O/Vocabulary/ORBvoc.txt" "$RUN/settings.yaml" "$DS" "$RUN/associations.txt" ); }
  VER="native $(git -C "$O" rev-parse --short HEAD 2>/dev/null || echo '?')"
else
  command -v docker >/dev/null || { echo "ERROR: docker not found"; exit 1; }
  docker image inspect "$IMAGE" >/dev/null 2>&1 || { echo "ERROR: image $IMAGE missing: docker build -t $IMAGE engines/orbslam3"; exit 1; }
  run_orb() { docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$DS:$DS:ro" -v "$RUN:$RUN" -w "$RUN" "$IMAGE" \
                /opt/ORB_SLAM3/Examples/RGB-D/rgbd_tum /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt \
                "$RUN/settings.yaml" "$DS" "$RUN/associations.txt"; }
  VER="docker $(docker image inspect --format '{{.Id}}' "$IMAGE" | cut -c1-19)"
fi
T0=$(date +%s)
run_orb > "$RUN/orbslam3.log" 2>&1 || { echo "ERROR: ORB-SLAM3 failed (see $RUN/orbslam3.log)"; tail -20 "$RUN/orbslam3.log"; exit 1; }
T1=$(date +%s)
[ -s "$RUN/CameraTrajectory.txt" ] || { echo "ERROR: no CameraTrajectory.txt"; tail -20 "$RUN/orbslam3.log"; exit 1; }
grep -E "median tracking time|mean tracking time|Map [0-9]+ has|New Map created|Fail to track" "$RUN/orbslam3.log" | sort | uniq -c | head -10 || true
python - "$RUN" $((T1 - T0)) "$VER" "$DS" "$MAXF" <<'EOF'
import json, sys
d, secs, ver, ds, maxf = sys.argv[1:]
json.dump({"engine": "orbslam3", "variant": "S1", "runtime_s": int(secs), "orbslam3": ver,
           "commit": "4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4", "dataset": ds, "max_frames": int(maxf) or None,
           "settings": open(f"{d}/settings.yaml").read()}, open(f"{d}/run.json", "w"), indent=2)
EOF
python "$REPO/eval/convert_traj.py" "$DS" "$RUN/CameraTrajectory.txt" "$RUN"
python "$REPO/eval/evaluate.py" "$DS" "$RUN"
[ "$FUSE" -eq 1 ] && python "$REPO/eval/fuse.py" "$DS" "$RUN"
echo "Done. Results table: $ROOT/results.csv"
