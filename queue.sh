#!/usr/bin/env bash
# Run several recordings one after another, unattended (e.g. over a weekend), with retries.
#
#   bash queue.sh BAG1.db3 [BAG2.db3 ...] [--retries N] [--force] [-- run_all options]
#
#   run_all options default to: --engines rtabmap --no-r2 --no-revisits --no-video   (RTAB-Map R1 only)
#   --retries N  extra attempts per bag after a failure (default 2). run_all.sh resumes, so a retry
#                only redoes the step that failed (a map step that ran out of memory is redone alone,
#                and the map builder then picks a coarser voxel size by itself).
#   --force      start even if the disk space check says the bags may not fit
#
# Before starting it checks: every bag exists, swap is on, free disk space for all exports, and it keeps
# the machine from going to sleep while it runs (systemd-inhibit). One bag failing does not stop the next.
# Start it detached so closing the terminal does not stop it:
#   setsid nohup bash queue.sh A.db3 B.db3 > ~/queue.log 2>&1 < /dev/null &
# Results: one line per bag at the end of the log, and ~/mv2_data/runs/QUEUE_<date>.txt
set -uo pipefail
REPO="$(cd "$(dirname "$0")" && pwd)"
BAGS=() RETRIES=2 FORCE=0 OPTS=() DATA="$HOME/mv2_data"
while [ $# -gt 0 ]; do
  case "$1" in
    --retries) RETRIES="$2"; shift ;;
    --force) FORCE=1 ;;
    --) shift; OPTS=("$@"); break ;;
    -h|--help) sed -n 2,17p "$0"; exit 0 ;;
    *) BAGS+=("$1") ;;
  esac
  shift
done
[ ${#OPTS[@]} -eq 0 ] && OPTS=(--engines rtabmap --no-r2 --no-revisits --no-video)
for ((i = 0; i < ${#OPTS[@]}; i++)); do [ "${OPTS[$i]}" = "--data" ] && DATA="${OPTS[$((i + 1))]}"; done
note() { echo "[$(date '+%a %H:%M:%S')] $*"; }
[ ${#BAGS[@]} -gt 0 ] || { sed -n 2,17p "$0"; exit 1; }

# keep the machine awake (re-run this script under systemd-inhibit once)
if [ -z "${MV2_QUEUE_INHIBITED:-}" ] && command -v systemd-inhibit >/dev/null 2>&1 \
   && systemd-inhibit --what=sleep:idle --who=mv2 --why=test true >/dev/null 2>&1; then
  export MV2_QUEUE_INHIBITED=1
  exec systemd-inhibit --what=sleep:idle:handle-lid-switch --who=mapping_v2 --why="mapping queue running" \
    bash "$0" "${BAGS[@]}" --retries "$RETRIES" $([ "$FORCE" -eq 1 ] && echo --force) -- "${OPTS[@]}"
fi

# ---- checks before starting
note "queue: ${#BAGS[@]} bag(s), run_all options: ${OPTS[*]}, retries $RETRIES"
bad=0
for b in "${BAGS[@]}"; do [ -f "$b" ] || { note "ERROR: no such file: $b"; bad=1; }; done
[ "$bad" -eq 0 ] || exit 1
[ -n "${MV2_QUEUE_INHIBITED:-}" ] && note "sleep/suspend blocked while the queue runs" \
  || note "WARNING: could not block sleep; make sure automatic suspend is off (see RUNBOOK)"
swap_gb=$(awk '/SwapTotal/ {print int($2 / 1048576)}' /proc/meminfo)
ram_gb=$(awk '/MemTotal/ {printf "%.1f", $2 / 1048576}' /proc/meminfo)
note "RAM ${ram_gb} GB, swap ${swap_gb} GB"
if [ "$swap_gb" -lt 4 ]; then note "ERROR: less than 4 GB swap; long runs may be killed. Turn swap on first."; [ "$FORCE" -eq 1 ] || exit 1; fi
if systemctl is-active --quiet systemd-oomd 2>/dev/null; then
  note "WARNING: systemd-oomd is running and may kill a long job under memory pressure: sudo systemctl disable --now systemd-oomd"
fi
mkdir -p "$DATA"
need=0
for b in "${BAGS[@]}"; do
  n="$(basename "$b" .db3)"
  [ -f "$DATA/datasets/$n/meta.json" ] || need=$(( need + $(stat -c %s "$b") / 1073741824 + ${MV2_DISK_MARGIN_GB:-10} ))
done
avail=$(( $(df --output=avail -B1 "$DATA" | tail -1) / 1073741824 ))
note "disk: ${avail} GB free, the exports need up to ~${need} GB"
if [ "$avail" -lt "$need" ]; then
  note "ERROR: not enough disk space for all exports (delete old datasets in $DATA/datasets first, or --force)"
  [ "$FORCE" -eq 1 ] || exit 1
fi

# ---- run
[ -z "${VIRTUAL_ENV:-}" ] && [ -f "$HOME/venvs/mv2/bin/activate" ] && source "$HOME/venvs/mv2/bin/activate"
REPORT="$DATA/runs/QUEUE_$(date +%Y%m%d-%H%M).txt"; mkdir -p "$DATA/runs"
RESULTS=()
for b in "${BAGS[@]}"; do
  n="$(basename "$b" .db3)"
  t0=$(date +%s) ok=0
  for ((a = 1; a <= RETRIES + 1; a++)); do
    note "=== $n: attempt $a of $((RETRIES + 1))"
    mkdir -p "$DATA/logs/$n"
    bash "$REPO/run_all.sh" "$b" "${OPTS[@]}" 2>&1 | tee -a "$DATA/logs/$n/run_all_queue.log"
    if [ -f "$DATA/final/$n/MANIFEST.json" ]; then ok=1; break; fi
    note "=== $n: no final map after attempt $a"
    [ "$a" -le "$RETRIES" ] && sleep "${MV2_RETRY_WAIT_S:-120}"
  done
  dt=$(( ($(date +%s) - t0) / 60 ))
  if [ "$ok" -eq 1 ]; then RESULTS+=("DONE    $n  (${dt} min, attempt $a)  map: $DATA/final/$n")
  else RESULTS+=("FAILED  $n  (${dt} min, $((RETRIES + 1)) attempts)  see $DATA/runs/SUMMARY_$n.txt and $DATA/logs/$n/"); fi
  note "${RESULTS[-1]}"
done

{ echo "mapping_v2 queue finished $(date '+%Y-%m-%d %H:%M')"; printf '%s\n' "${RESULTS[@]}"
  for b in "${BAGS[@]}"; do n="$(basename "$b" .db3)"; echo; echo "--- $n"; cat "$DATA/runs/SUMMARY_$n.txt" 2>/dev/null; done
} > "$REPORT"
echo; cat "$REPORT"
note "queue done. Report: $REPORT"
