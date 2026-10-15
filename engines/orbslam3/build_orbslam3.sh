#!/usr/bin/env bash
# Build ORB-SLAM3 (commit 4452a3c) third-party libraries and the RGB-D example only.
# Used inside the Dockerfile and for a native build. Needs Pangolin installed (CMake can find it).
# Usage: bash engines/orbslam3/build_orbslam3.sh ORB_SLAM3_DIR [JOBS]   (default: from available RAM)
set -euo pipefail
D="$(cd "${1:?usage: build_orbslam3.sh ORB_SLAM3_DIR [JOBS]}" && pwd)"
# parallel jobs: about 3 GB of RAM per compiler job (Tracking.cc alone needs ~2.5 GB)
MEM_GB=$(awk '/MemAvailable/ {print int($2/1048576)}' /proc/meminfo)
AUTO_J=$(( MEM_GB / 3 )); [ "$AUTO_J" -lt 1 ] && AUTO_J=1; [ "$AUTO_J" -gt "$(nproc)" ] && AUTO_J=$(nproc)
J="${2:-$AUTO_J}"
echo "building with $J parallel jobs (${MEM_GB} GB available)"
HERE="$(cd "$(dirname "$0")" && pwd)"
bash "$HERE/patch_orbslam3.sh" "$D"
for lib in DBoW2 g2o Sophus; do
  cmake -S "$D/Thirdparty/$lib" -B "$D/Thirdparty/$lib/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF > /dev/null
  cmake --build "$D/Thirdparty/$lib/build" -j "$J" > /dev/null
  echo "built $lib"
done
[ -f "$D/Vocabulary/ORBvoc.txt" ] || tar -xf "$D/Vocabulary/ORBvoc.txt.tar.gz" -C "$D/Vocabulary"
cmake -S "$D" -B "$D/build" -DCMAKE_BUILD_TYPE=Release > /dev/null
cmake --build "$D/build" -j "$J" --target rgbd_tum
ls -la "$D/Examples/RGB-D/rgbd_tum"
echo "ORB-SLAM3 rgbd_tum built"
