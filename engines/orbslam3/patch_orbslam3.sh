#!/usr/bin/env bash
# Patch an ORB-SLAM3 checkout (commit 4452a3c) so it builds on Ubuntu 22.04/24.04 with Pangolin 0.9
# and runs headless. String-anchored and idempotent: each patch is applied once; if an anchor is
# missing (different source), it stops with an error instead of guessing.
# Usage: bash engines/orbslam3/patch_orbslam3.sh ORB_SLAM3_DIR
set -euo pipefail
D="${1:?usage: patch_orbslam3.sh ORB_SLAM3_DIR}"

patch() {  # file old new label
  local f="$D/$1" old="$2" new="$3" label="$4"
  if grep -qF -- "$new" "$f"; then echo "already applied: $label"; return; fi
  grep -qF -- "$old" "$f" || { echo "ERROR: anchor not found for '$label' in $1"; exit 1; }
  python3 - "$f" "$old" "$new" <<'EOF'
import sys
f, old, new = sys.argv[1:]
s = open(f).read()
open(f, "w").write(s.replace(old, new))
EOF
  echo "applied: $label"
}

# Pangolin 0.9 needs C++17 (ORB-SLAM3 forces C++11)
patch CMakeLists.txt 'set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")' \
                     'set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")' "C++17 for ORB-SLAM3"
# C++17 forbids ++ on bool; the counter is used as an int index (idx != mnFullBAIdx)
patch include/LoopClosing.h '    bool mnFullBAIdx;' '    int mnFullBAIdx;' "mnFullBAIdx is an int counter"
# headless: no Pangolin viewer window in the RGB-D example
patch Examples/RGB-D/rgbd_tum.cc 'ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);' \
                                 'ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,false);' "headless rgbd_tum"
echo "ORB-SLAM3 patches OK"
