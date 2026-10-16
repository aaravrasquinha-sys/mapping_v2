#!/usr/bin/env bash
# Self-test of bagtools on synthetic recordings made with the real SDK. Takes about a minute.
# Usage: bash tests/run_bagtools_tests.sh        (with the venv active)
set -uo pipefail
cd "$(dirname "$0")/.."
T="$(mktemp -d /tmp/mv2_selftest.XXXX)"
fail=0
ok()  { echo "TEST PASS: $1"; }
bad() { echo "TEST FAIL: $1"; fail=1; }

python tests/make_synth_db3.py "$T/synth.db3" --frames 90 >/dev/null || bad "make synthetic recording"
python tests/make_synth_db3.py "$T/step.db3" --frames 30 --step >/dev/null || bad "make step recording"

out=$(python bagtools/inspect_db3.py "$T/synth.db3" --out "$T/inspect.json")
echo "$out" | grep -q "colour (sensor_1/Color_0): 89 frames.*lost while recording: 1" && ok "inspect: colour count + lost frame" || bad "inspect: colour"
echo "$out" | grep -q "depth (sensor_0/Depth_0): 88 frames.*lost while recording: 2" && ok "inspect: depth count + lost frames" || bad "inspect: depth"
echo "$out" | grep -q "IMU full rate: YES" && ok "inspect: IMU full rate" || bad "inspect: IMU"
echo "$out" | grep -q "colour/depth start offset: 3.0 ms" && ok "inspect: colour/depth offset" || bad "inspect: offset"

python bagtools/export_db3.py "$T/synth.db3" "$T/ex" > "$T/export.log" 2>&1 || bad "export ran"
grep -q "colour_frames_written: 87" "$T/export.log" && grep -q "colour_frames_no_depth: 2" "$T/export.log" \
  && grep -q "gyro_samples: 1200" "$T/export.log" && grep -q "accel_samples: 600" "$T/export.log" \
  && ok "export: exact counts" || bad "export counts (see $T/export.log)"
python bagtools/check_export.py "$T/synth.db3" "$T/ex" > "$T/check.log" 2>&1 && ok "check_export: PASS" || bad "check_export (see $T/check.log)"

# a recording with one accel sample stamped near 0 (as in the real walk1.db3): inspect must not report 0 Hz,
# the exporter must drop it, and the independent check must still pass
python tests/make_synth_db3.py "$T/badimu.db3" --frames 60 --bad-imu-stamp >/dev/null || bad "make bad-IMU-stamp recording"
out2=$(python bagtools/inspect_db3.py "$T/badimu.db3" --out "$T/inspect_bad.json")
echo "$out2" | grep -q "IMU full rate: YES" && echo "$out2" | grep -q "WARNING.*1 sample(s) with a timestamp" \
  && ok "inspect: bad IMU stamp flagged, rate still correct" || bad "inspect: bad IMU stamp"
python bagtools/export_db3.py "$T/badimu.db3" "$T/ex_bad" > "$T/export_bad.log" 2>&1 || bad "export (bad IMU stamp) ran"
python bagtools/check_export.py "$T/badimu.db3" "$T/ex_bad" > "$T/check_bad.log" 2>&1 \
  && grep -q "dropped 1" "$T/check_bad.log" && ok "export drops the bad IMU stamp; check_export: PASS" || bad "bad IMU stamp (see $T/check_bad.log)"

python bagtools/export_db3.py "$T/step.db3" "$T/ex_step" > /dev/null 2>&1
python - "$T/ex_step" <<'EOF' && ok "alignment direction (1 m edge at colour column 329)" || bad "alignment direction"
import sys, glob, cv2
row = cv2.imread(sorted(glob.glob(sys.argv[1] + "/depth/*.png"))[3], cv2.IMREAD_UNCHANGED)[240]
# 1 m (5000) up to column 329, 2 m (10000) from 330 (expected edge 606.75*0.015/1.0 + 320.19 = 329.3)
sys.exit(0 if row[329] == 5000 and row[331] == 10000 and row[320] == 5000 else 1)
EOF

python bagtools/export_db3.py "$T/synth.db3" "$T/ex_small" --max-frames 30 > /dev/null 2>&1
python bagtools/check_export.py "$T/synth.db3" "$T/ex_small" > /dev/null 2>&1 && ok "--max-frames export passes its check" || bad "--max-frames"

rm -f "$(ls -d "$T"/ex/depth/*.png | sed -n 10p)"
python bagtools/check_export.py "$T/synth.db3" "$T/ex" > /dev/null 2>&1 && bad "damaged export was not detected" || ok "damaged export is detected"

rm -rf "$T"
if [ $fail -eq 0 ]; then echo "ALL TESTS PASSED"; else echo "SOME TESTS FAILED"; fi
exit $fail
