#!/usr/bin/env bash
# Patch an ORB-SLAM3 checkout (commit 4452a3c) so it builds on Ubuntu 22.04/24.04 with Pangolin 0.9
# and runs headless. String-anchored and idempotent: each patch is applied once; if an anchor is
# missing (different source), it stops with an error instead of guessing.
# Usage: bash engines/orbslam3/patch_orbslam3.sh ORB_SLAM3_DIR
set -euo pipefail
D="${1:?usage: patch_orbslam3.sh ORB_SLAM3_DIR}"

patch() {  # file old new label   (exact multi-line strings; old must occur exactly once)
  python3 - "$D/$1" "$2" "$3" "$4" <<'EOF'
import sys
f, old, new, label = sys.argv[1:]
s = open(f).read()
if new in s:
    print("already applied:", label)
    sys.exit(0)
n = s.count(old)
if n != 1:
    print(f"ERROR: anchor for '{label}' found {n} times in {f} (expected once)")
    sys.exit(1)
open(f, "w").write(s.replace(old, new))
print("applied:", label)
EOF
}

# Pangolin 0.9 needs C++17 (ORB-SLAM3 forces C++11)
patch CMakeLists.txt 'set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")' \
                     'set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")' "C++17 for ORB-SLAM3"
# C++17 forbids ++ on bool; the counter is used as an int index (idx != mnFullBAIdx)
patch include/LoopClosing.h '    bool mnFullBAIdx;' '    int mnFullBAIdx;' "mnFullBAIdx is an int counter"
# headless: no Pangolin viewer window in the RGB-D example
patch Examples/RGB-D/rgbd_tum.cc 'ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);' \
                                 'ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,false);' "headless rgbd_tum"
# SaveTrajectoryTUM mixes coordinate frames when tracking was lost and a new (unmerged) map was started;
# SaveTrajectoryEuRoC keeps only the biggest map (frames of other maps are left out = no pose)
patch Examples/RGB-D/rgbd_tum.cc '    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");' \
'    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory_biggest_map.txt");   // mv2: biggest map only, t in ns' \
  "rgbd_tum also saves the biggest-map trajectory"
echo "ORB-SLAM3 patches OK"
