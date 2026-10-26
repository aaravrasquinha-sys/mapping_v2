#!/usr/bin/env bash
# RTAB-Map study of one recording, unattended: R1 + R2 (+ variants), then a report for comparing
# against another SLAM pipeline (tracking per 5 s window, resets, loop closures, legs/turn, spin, map views).
#
#   bash rtabmap_study.sh BAG.db3 [--variant TAG "NAME VALUE [NAME VALUE ...]"]... [--no-variants] [--data DIR] [--force] [--turn-window 150-260]
#
#   1. queue.sh BAG -- --engines rtabmap --no-revisits --no-video   (export + check, R1 + R2, final map; retries)
#   2. variants: R1 with extra RTAB-Map parameters, one run each (default: d7 = "Vis/MaxDepth 7")
#   3. eval/rtabmap_report.py on every RTAB-Map run of this recording -> DATA/final/NAME/rtabmap_report/
# Every step is skipped when already done, so running the same command again resumes.
# Start it detached:  setsid nohup bash rtabmap_study.sh BAG.db3 > ~/study.log 2>&1 < /dev/null &
set -uo pipefail
REPO="$(cd "$(dirname "$0")" && pwd)"
BAG="" DATA="$HOME/mv2_data" VARIANTS=() NOVAR=0 FORCE=() TURNWIN=""
while [ $# -gt 0 ]; do
  case "$1" in
    --variant) VARIANTS+=("$2|$3"); shift 2 ;;
    --no-variants) NOVAR=1 ;;
    --force) FORCE=(--force) ;;
    --turn-window) TURNWIN="$2"; shift ;;
    --data) DATA="$2"; shift ;;
    -h|--help) sed -n 2,13p "$0"; exit 0 ;;
    *) BAG="$1" ;;
  esac
  shift
done
[ -n "$BAG" ] && [ -f "$BAG" ] || { sed -n 2,13p "$0"; echo "ERROR: give an existing .db3 file"; exit 1; }
[ ${#VARIANTS[@]} -eq 0 ] && [ "$NOVAR" -eq 0 ] && VARIANTS=("d7|Vis/MaxDepth 7")
[ "$NOVAR" -eq 1 ] && VARIANTS=()
BAG="$(cd "$(dirname "$BAG")" && pwd)/$(basename "$BAG")"
NAME="$(basename "$BAG" .db3)"
note() { echo "[$(date '+%a %H:%M:%S')] $*"; }

if [ -z "${MV2_QUEUE_INHIBITED:-}" ] && command -v systemd-inhibit >/dev/null 2>&1 \
   && systemd-inhibit --what=sleep:idle --who=mv2 --why=test true >/dev/null 2>&1; then
  export MV2_QUEUE_INHIBITED=1
  ARGS=("$BAG" --data "$DATA" ${FORCE[@]+"${FORCE[@]}"}); [ "$NOVAR" -eq 1 ] && ARGS+=(--no-variants)
  [ -n "$TURNWIN" ] && ARGS+=(--turn-window "$TURNWIN")
  for v in ${VARIANTS[@]+"${VARIANTS[@]}"}; do ARGS+=(--variant "${v%%|*}" "${v#*|}"); done
  exec systemd-inhibit --what=sleep:idle:handle-lid-switch --who=mapping_v2 --why="RTAB-Map study running" \
    bash "$0" "${ARGS[@]}"
fi
[ -z "${VIRTUAL_ENV:-}" ] && [ -f "$HOME/venvs/mv2/bin/activate" ] && source "$HOME/venvs/mv2/bin/activate"

# ---- 1. export + R1 + R2 + final map
note "study: $NAME  step 1: export, R1 + R2, final map"
bash "$REPO/queue.sh" "$BAG" ${FORCE[@]+"${FORCE[@]}"} -- --engines rtabmap --no-revisits --no-video --data "$DATA"
DS="$DATA/datasets/$NAME" RUNS="$DATA/runs" OUT="$DATA/final/$NAME/rtabmap_report"
[ -f "$DS/export_report.json" ] || { note "ERROR: no checked export of $NAME; stopping (see $DATA/logs/$NAME/)"; exit 1; }

runs_for() {  # suffix -> finished run dirs of this dataset ending in _SUFFIX (one per line)
  python - "$RUNS" "$1" "$NAME" <<'EOF'
import json, os, sys
root, tag, name = sys.argv[1:]
for d in sorted(os.listdir(root)):
    p = os.path.join(root, d)
    if d.endswith("_" + tag) and os.path.isfile(os.path.join(p, "map_frame.json")) and os.path.isfile(os.path.join(p, "metrics.json")):
        if os.path.basename(json.load(open(os.path.join(p, "metrics.json")))["dataset"].rstrip("/")) == name:
            print(p)
EOF
}

# ---- 2. variants
for v in ${VARIANTS[@]+"${VARIANTS[@]}"}; do
  tag="${v%%|*}"; read -r -a kv <<< "${v#*|}"
  P=(); for ((i = 0; i + 1 < ${#kv[@]}; i += 2)); do P+=(--param "${kv[$i]}" "${kv[$((i + 1))]}"); done
  for attempt in 1 2; do
    [ -n "$(runs_for "rtabmap_R1_$tag" | tail -1)" ] && { note "variant $tag: done"; break; }
    note "variant $tag (${v#*|}): attempt $attempt"
    bash "$REPO/engines/rtabmap/run_rtabmap.sh" "$DS" "$RUNS" --tag "$tag" "${P[@]}" 2>&1 \
      | tee -a "$DATA/logs/$NAME/4_rtabmap_$tag.log" || true
  done
done

# ---- 3. report
mapfile -t ALL < <({ runs_for rtabmap_R1 | tail -1; runs_for rtabmap_R2 | tail -1
                     for v in ${VARIANTS[@]+"${VARIANTS[@]}"}; do runs_for "rtabmap_R1_${v%%|*}" | tail -1; done; } | grep -v '^$')
note "report on ${#ALL[@]} run(s): ${ALL[*]##*/}"
mkdir -p "$OUT"
python "$REPO/eval/rtabmap_report.py" "$DS" "${ALL[@]}" --out "$OUT" ${TURNWIN:+--turn-window "$TURNWIN"} 2>&1 | tee "$OUT/report.log"
python "$REPO/eval/compare_runs.py" "$RUNS" "${ALL[@]##*/}" --dataset "$NAME" > "$OUT/compare.log" 2>&1 || true
for r in "${ALL[@]}"; do
  rm -rf "$OUT/${r##*/}"; mkdir -p "$OUT/${r##*/}"
  cp -r "$r/report/." "$OUT/${r##*/}/" 2>/dev/null
  cp "$r/views.png" "$r/run.json" "$r/metrics.json" "$r/map_frame.json" "$r/trajectory.tum" "$OUT/${r##*/}/" 2>/dev/null
  cp "$r/rtabmap_info.txt" "$OUT/${r##*/}/" 2>/dev/null
done
cp "$RUNS/compare_$NAME.md" "$RUNS/compare_$NAME.png" "$OUT/" 2>/dev/null
note "study done: $OUT  (start with rtabmap_comparison.md)"
