#!/usr/bin/env bash
# One command from a .db3 recording to a finished map:
#   inspect -> export + independent check -> revisit pairs -> engines (RTAB-Map R1+R2, ORB-SLAM3 S1,
#   then Open3D O1 [+ O2 with --slac]) -> comparison + recommendation -> final map + replay video.
#
# Resumable: every step that already finished is skipped, so after a failure (or Ctrl-C) just run the
# same command again. A failing engine does not stop the others.
#
# Usage:
#   bash run_all.sh BAG.db3 [--name NAME] [--data DIR] [--engines "rtabmap orbslam3 open3d"] [--slac]
#                   [--smoke N] [--no-video] [--revisit-minutes N]
#   --name      dataset name (default: bag file name without .db3, e.g. walk1)
#   --data      data folder (default ~/mv2_data): datasets/, runs/, final/, logs/
#   --engines   which engines to run (default: all three; Open3D is the slow one, 1-3 h)
#   --slac      also run Open3D O2 (SLAC; roughly twice O1's time)
#   --smoke N   quick machine test on the first N frames (dataset NAME_smokeN; everything else identical)
#   --no-video  skip the replay video
#   --revisit-minutes N  time limit of the revisit search (default 40; fewer minutes = fewer pairs found)
#
# Needs: setup/make_venv.sh and setup/install_engines.sh done once.
# Results: DATA/final/NAME/ (the map), DATA/runs/compare_NAME.md/.png, DATA/runs/SUMMARY_NAME.txt
set -uo pipefail
REPO="$(cd "$(dirname "$0")" && pwd)"
BAG="" NAME="" DATA="$HOME/mv2_data" ENGINES="rtabmap orbslam3 open3d" SLAC=0 SMOKE=0 VIDEO=1 REVISIT_MIN=40
while [ $# -gt 0 ]; do
  case "$1" in
    --name) NAME="$2"; shift ;;
    --data) DATA="$2"; shift ;;
    --engines) ENGINES="$2"; shift ;;
    --slac) SLAC=1 ;;
    --smoke) SMOKE="$2"; shift ;;
    --no-video) VIDEO=0 ;;
    --revisit-minutes) REVISIT_MIN="$2"; shift ;;
    -h|--help) sed -n 2,23p "$0"; exit 0 ;;
    *) [ -z "$BAG" ] && BAG="$1" || { echo "unknown argument $1"; exit 1; } ;;
  esac
  shift
done
[ -n "$BAG" ] && [ -f "$BAG" ] || { sed -n 2,22p "$0"; echo "ERROR: give an existing .db3 file"; exit 1; }
BAG="$(cd "$(dirname "$BAG")" && pwd)/$(basename "$BAG")"
NAME="${NAME:-$(basename "$BAG" .db3)}"
[ "$SMOKE" -gt 0 ] && NAME="${NAME}_smoke${SMOKE}"
mkdir -p "$DATA"/{datasets,runs,final,logs/"$NAME"}
DATA="$(cd "$DATA" && pwd)"
DS="$DATA/datasets/$NAME" RUNS="$DATA/runs" LOGS="$DATA/logs/$NAME" FINAL="$DATA/final/$NAME"
SUMMARY="$RUNS/SUMMARY_$NAME.txt"

if [ -z "${VIRTUAL_ENV:-}" ]; then
  [ -f "$HOME/venvs/mv2/bin/activate" ] || { echo "ERROR: no venv; run: bash setup/make_venv.sh"; exit 1; }
  # shellcheck disable=SC1091
  source "$HOME/venvs/mv2/bin/activate"
fi
python -c "import open3d, pyrealsense2" 2>/dev/null || { echo "ERROR: the venv lacks open3d/pyrealsense2"; exit 1; }

STATUS=()
note() { echo "[$(date +%H:%M:%S)] $*"; }
step() {  # name logfile command... : run, tee to log, record status
  local name="$1" log="$2"; shift 2
  note "$name ... (log: $log)"
  local t0; t0=$(date +%s)
  "$@" 2>&1 | tee "$log"
  local rc=${PIPESTATUS[0]}
  local dt=$(( $(date +%s) - t0 ))
  if [ "$rc" -eq 0 ]; then STATUS+=("OK     $name (${dt} s)"); else STATUS+=("FAILED $name (exit $rc, ${dt} s, see $log)"); fi
  return "$rc"
}
latest_run() {  # engine_variant -> newest finished, non-smoke run dir for this dataset (or nothing)
  python - "$RUNS" "$1" "$NAME" <<'EOF'
import json, os, sys
root, tag, name = sys.argv[1:]
best = None
for d in sorted(os.listdir(root)):
    p = os.path.join(root, d)
    if not d.endswith("_" + tag) or not os.path.isfile(os.path.join(p, "metrics.json")) \
            or not os.path.isfile(os.path.join(p, "map_frame.json")):
        continue
    if os.path.basename(json.load(open(os.path.join(p, "metrics.json")))["dataset"].rstrip("/")) == name:
        best = p
print(best or "")
EOF
}
summary() {
  { echo "mapping_v2 run_all: $NAME  ($(date '+%Y-%m-%d %H:%M'))"
    echo "bag: $BAG"; echo "repo: $(git -C "$REPO" rev-parse --short HEAD 2>/dev/null)"
    printf '%s\n' "${STATUS[@]}"
    [ -f "$RUNS/compare_$NAME.md" ] && { echo; sed -n '/^| metric/,$p' "$RUNS/compare_$NAME.md"; }
    [ -f "$FINAL/MANIFEST.json" ] && echo "final map: $FINAL  (from $(python -c "import json;print(json.load(open('$FINAL/MANIFEST.json')).get('run','?'))"))"
  } > "$SUMMARY"
  echo; cat "$SUMMARY"
}
trap 'echo; note "interrupted"; summary; exit 130' INT

note "run_all: $BAG -> $NAME (data in $DATA)"

# ---- 1. inspect
INSPECT="$DATA/$NAME.inspect.json"
if [ -f "$INSPECT" ]; then STATUS+=("skip   inspect (exists)")
else step "inspect" "$LOGS/1_inspect.log" python "$REPO/bagtools/inspect_db3.py" "$BAG" --out "$INSPECT" || { summary; exit 1; }
fi

# ---- 2. export + check (the check must pass; it then locks the dataset read-only)
if [ -f "$DS/export_report.json" ] && python -c "import json,sys; sys.exit(0 if json.load(open('$DS/export_report.json')).get('pass') else 1)" 2>/dev/null; then
  STATUS+=("skip   export + check (passed before)")
else
  if [ -d "$DS" ] && [ ! -f "$DS/meta.json" ]; then rm -rf "$DS"; fi       # half-finished export
  if [ ! -f "$DS/meta.json" ]; then
    MAXF=(); [ "$SMOKE" -gt 0 ] && MAXF=(--max-frames "$SMOKE")
    step "export" "$LOGS/2_export.log" python "$REPO/bagtools/export_db3.py" "$BAG" "$DS" "${MAXF[@]}" || { summary; exit 1; }
  fi
  step "export check" "$LOGS/2_check.log" python "$REPO/bagtools/check_export.py" "$BAG" "$DS" --lock \
    || { note "the export check FAILED: the dataset is not trustworthy, stopping here (see the log)"; summary; exit 1; }
fi

# ---- 3. revisit pairs (independent of every engine; used to score them)
PAIRS="$RUNS/_cache/$NAME/revisit_pairs.json"
if [ -f "$PAIRS" ]; then STATUS+=("skip   revisit pairs (exist)")
else mkdir -p "$(dirname "$PAIRS")"
     step "revisit pairs" "$LOGS/3_revisits.log" python "$REPO/eval/revisit_pairs.py" "$DS" "$PAIRS" --max-minutes "$REVISIT_MIN" || true
fi

compare_and_finish() {  # label
  step "compare ($1)" "$LOGS/5_compare.log" python "$REPO/eval/compare_runs.py" "$RUNS" --dataset "$NAME" --pick || return
  local rec; rec="$(python -c "import json;print(json.load(open('$RUNS/recommended_$NAME.json'))['run_dir'])")"
  if [ -f "$FINAL/MANIFEST.json" ] && python -c "import json,sys; sys.exit(0 if json.load(open('$FINAL/MANIFEST.json')).get('run','').rstrip('/')=='$rec' else 1)"; then
    STATUS+=("skip   final map (already from $(basename "$rec"))")
  else
    rm -rf "$FINAL"
    step "final map from $(basename "$rec")" "$LOGS/6_final.log" python "$REPO/finish/finalize_map.py" "$DS" "$rec" "$FINAL" || return
  fi
  if [ "$VIDEO" -eq 1 ] && [ ! -f "$rec/replay.mp4" ]; then
    local st=1; [ "$(ls "$DS/rgb" | wc -l)" -gt 3000 ] && st=2
    step "replay video" "$LOGS/7_video.log" python "$REPO/finish/render_video.py" "$DS" "$rec" --step "$st" \
      && cp "$rec/replay.mp4" "$FINAL/replay.mp4"
  fi
}

# ---- 4a. fast engines
has() { case " $ENGINES " in *" $1 "*) return 0;; *) return 1;; esac; }
if has rtabmap; then
  if [ -n "$(latest_run rtabmap_R1)" ] && [ -n "$(latest_run rtabmap_R2)" ]; then STATUS+=("skip   RTAB-Map R1+R2 (done)")
  else step "RTAB-Map R1+R2" "$LOGS/4_rtabmap.log" bash "$REPO/engines/rtabmap/run_rtabmap.sh" "$DS" "$RUNS" --r2 || true
  fi
fi
if has orbslam3; then
  if [ -n "$(latest_run orbslam3_S1)" ]; then STATUS+=("skip   ORB-SLAM3 S1 (done)")
  else step "ORB-SLAM3 S1" "$LOGS/4_orbslam3.log" bash "$REPO/engines/orbslam3/run_orbslam3.sh" "$DS" "$RUNS" || true
  fi
fi
compare_and_finish "fast engines" || true

# ---- 4b. Open3D (slow); then compare again and redo the final map only if the pick changes
if has open3d; then
  if [ -n "$(latest_run open3d_O1)" ]; then STATUS+=("skip   Open3D O1 (done)")
  else note "Open3D O1 takes 1-3 h; a provisional final map is already in $FINAL"
       step "Open3D O1" "$LOGS/4_open3d_O1.log" bash "$REPO/engines/open3d_recon/run_open3d.sh" "$DS" "$RUNS" || true
  fi
  if [ "$SLAC" -eq 1 ]; then
    if [ -n "$(latest_run open3d_O2)" ]; then STATUS+=("skip   Open3D O2 (done)")
    else step "Open3D O2 (SLAC)" "$LOGS/4_open3d_O2.log" bash "$REPO/engines/open3d_recon/run_open3d.sh" "$DS" "$RUNS" --slac || true
    fi
  fi
  compare_and_finish "all engines" || true
fi

summary
note "done. Map: $FINAL   Comparison: $RUNS/compare_$NAME.md and .png"
