#!/usr/bin/env bash
# Build the mapping engines natively on Ubuntu 22.04 (no Docker needed). Everything goes into
# ~/mv2_tools (override with MV2_TOOLS=...); only the apt step needs sudo.
#
#   g2o        20230806_git        (the version RTAB-Map was tested with; Ubuntu 22.04 has no g2o package)
#   RTAB-Map   commit af1ced2      (tested here; reports 0.24.2)       -> ~/mv2_tools/rtabmap/bin
#   Pangolin   v0.9.2 + ORB-SLAM3 commit 4452a3c (patched, headless)   -> ~/mv2_tools/ORB_SLAM3
#   Open3D     0.19.0 env for the Open3D reconstruction system          -> ~/venvs/o3d019
#
# Usage: bash setup/install_engines.sh [rtabmap] [orbslam3] [open3d]     (default: all three)
#        KEEP_BUILD=1 keeps build folders (otherwise removed after install to save ~2 GB)
# Safe to re-run: finished parts are skipped (delete the folder to rebuild).
# Time on a Z440: roughly 30-60 min in total. Disk: ~3 GB apt packages + ~1.5 GB in ~/mv2_tools.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
TOOLS="${MV2_TOOLS:-$HOME/mv2_tools}"
SRC="$TOOLS/src"
WANT="${*:-rtabmap orbslam3 open3d}"
mkdir -p "$SRC"

G2O_TAG=20230806_git
RTABMAP_COMMIT=af1ced217f3a05a124bb418261f175baddb08adc
PANGOLIN_TAG=v0.9.2
ORB_COMMIT=4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4

# compiler jobs: ~3 GB RAM per job (ORB-SLAM3 needs ~6 GB per job, handled in its own script)
MEM_GB=$(awk '/MemAvailable/ {m=$2} /SwapFree/ {s=$2} END {print int((m+s)/1048576)}' /proc/meminfo)
J=$(( MEM_GB / 3 )); [ "$J" -lt 1 ] && J=1; [ "$J" -gt "$(nproc)" ] && J=$(nproc)
echo "install_engines: tools in $TOOLS, $J parallel jobs (${MEM_GB} GB RAM+swap available), parts: $WANT"

. /etc/os-release
[ "${VERSION_ID:-}" = "22.04" ] || echo "WARNING: written and tested for Ubuntu 22.04; this is ${PRETTY_NAME:-unknown}"

SUDO=""; [ "$(id -u)" -ne 0 ] && SUDO="sudo"
fetch() {  # url dir ref : shallow fetch of one tag or commit
  local url="$1" dir="$2" ref="$3"
  if [ ! -d "$dir/.git" ]; then
    git init -q "$dir"; git -C "$dir" remote add origin "$url"
  fi
  git -C "$dir" fetch -q --depth 1 origin "$ref"
  git -C "$dir" checkout -q FETCH_HEAD
}
has() { case " $WANT " in *" $1 "*) return 0;; *) return 1;; esac; }

# ---------------------------------------------------------------- apt packages (once)
PKGS="build-essential cmake git pkg-config ca-certificates python3-venv python3-dev
      libeigen3-dev libsuitesparse-dev libopencv-dev libsqlite3-dev libyaml-cpp-dev"
has rtabmap && PKGS="$PKGS libpcl-dev libceres-dev libboost-all-dev"
has orbslam3 && PKGS="$PKGS libboost-serialization-dev libssl-dev libglew-dev libepoxy-dev libgl1-mesa-dev"
MISSING=$(for p in $PKGS; do dpkg -s "$p" >/dev/null 2>&1 || echo "$p"; done)
if [ -n "$MISSING" ]; then
  echo "apt: installing $(echo $MISSING | wc -w) packages (sudo)"
  $SUDO apt-get update -qq
  $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends $MISSING > "$SRC/apt.log" 2>&1 \
    || { echo "ERROR: apt-get failed, see $SRC/apt.log"; tail -20 "$SRC/apt.log"; exit 1; }
fi
echo "apt packages OK"

# ---------------------------------------------------------------- RTAB-Map (+ g2o)
if has rtabmap; then
  if [ -x "$TOOLS/rtabmap/bin/rtabmap-rgbd_dataset" ]; then
    echo "RTAB-Map already installed in $TOOLS/rtabmap (delete it to rebuild)"
  else
    if [ ! -f "$TOOLS/g2o/lib/libg2o_core.so" ]; then
      echo "g2o $G2O_TAG: building (log $SRC/g2o.log)"
      fetch https://github.com/RainerKuemmerle/g2o.git "$SRC/g2o" "$G2O_TAG"
      { cmake -S "$SRC/g2o" -B "$SRC/g2o/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$TOOLS/g2o" \
          -DBUILD_SHARED_LIBS=ON -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF \
          -DCMAKE_INSTALL_RPATH="$TOOLS/g2o/lib"
        cmake --build "$SRC/g2o/build" -j "$J"
        cmake --install "$SRC/g2o/build"; } > "$SRC/g2o.log" 2>&1 \
        || { echo "ERROR: g2o build failed"; tail -30 "$SRC/g2o.log"; exit 1; }
    fi
    echo "RTAB-Map ${RTABMAP_COMMIT:0:7}: building (log $SRC/rtabmap.log; 15-40 min)"
    fetch https://github.com/introlab/rtabmap.git "$SRC/rtabmap" "$RTABMAP_COMMIT"
    { G2O_ROOT="$TOOLS/g2o" cmake -S "$SRC/rtabmap" -B "$SRC/rtabmap/build" -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$TOOLS/rtabmap" -DCMAKE_PREFIX_PATH="$TOOLS/g2o" \
        -DWITH_QT=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF \
        -DCMAKE_INSTALL_RPATH="$TOOLS/rtabmap/lib;$TOOLS/g2o/lib" -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON
      cmake --build "$SRC/rtabmap/build" -j "$J"
      cmake --install "$SRC/rtabmap/build"; } > "$SRC/rtabmap.log" 2>&1 \
      || { echo "ERROR: RTAB-Map build failed"; tail -30 "$SRC/rtabmap.log"; exit 1; }
    grep -E "^ *With (g2o|GTSAM|Ceres|TORO)" "$SRC/rtabmap.log" | head -5 || true
    [ "${KEEP_BUILD:-0}" = 1 ] || rm -rf "$SRC/rtabmap/build" "$SRC/g2o/build"
  fi
  "$TOOLS/rtabmap/bin/rtabmap-rgbd_dataset" --version 2>&1 | grep -E "^(RTAB-Map|PCL|OpenCV|With g2o|With GTSAM|With Ceres)" || true
  "$TOOLS/rtabmap/bin/rtabmap-rgbd_dataset" --version 2>&1 | grep -q "^RTAB-Map:" \
    && echo "RTAB-Map OK: $TOOLS/rtabmap/bin" || { echo "ERROR: RTAB-Map binaries do not run"; exit 1; }
fi

# ---------------------------------------------------------------- Pangolin + ORB-SLAM3
if has orbslam3; then
  O="$TOOLS/ORB_SLAM3"
  if [ -x "$O/Examples/RGB-D/rgbd_tum" ]; then
    echo "ORB-SLAM3 already built in $O (delete it to rebuild)"
  else
    if [ ! -d "$TOOLS/pangolin/lib/cmake/Pangolin" ] && [ ! -d "$TOOLS/pangolin/lib64/cmake/Pangolin" ]; then
      echo "Pangolin $PANGOLIN_TAG: building (log $SRC/pangolin.log)"
      fetch https://github.com/stevenlovegrove/Pangolin.git "$SRC/Pangolin" "$PANGOLIN_TAG"
      { cmake -S "$SRC/Pangolin" -B "$SRC/Pangolin/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF \
          -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX="$TOOLS/pangolin" \
          -DCMAKE_INSTALL_RPATH="$TOOLS/pangolin/lib"
        cmake --build "$SRC/Pangolin/build" -j "$J"
        cmake --install "$SRC/Pangolin/build"; } > "$SRC/pangolin.log" 2>&1 \
        || { echo "ERROR: Pangolin build failed"; tail -30 "$SRC/pangolin.log"; exit 1; }
    fi
    echo "ORB-SLAM3 ${ORB_COMMIT:0:7}: building (log $SRC/orbslam3.log; 15-40 min, ~6 GB RAM per compiler job)"
    fetch https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$O" "$ORB_COMMIT"
    CMAKE_PREFIX_PATH="$TOOLS/pangolin" bash "$REPO/engines/orbslam3/build_orbslam3.sh" "$O" > "$SRC/orbslam3.log" 2>&1 \
      || { echo "ERROR: ORB-SLAM3 build failed"; tail -30 "$SRC/orbslam3.log"; exit 1; }
    grep -E "applied|patches OK|parallel jobs|WARNING" "$SRC/orbslam3.log" || true
    [ "${KEEP_BUILD:-0}" = 1 ] || rm -rf "$O/build" "$O"/Thirdparty/*/build "$SRC/Pangolin/build"
  fi
  [ -x "$O/Examples/RGB-D/rgbd_tum" ] && echo "ORB-SLAM3 OK: $O" || { echo "ERROR: rgbd_tum missing"; exit 1; }
fi

# ---------------------------------------------------------------- Open3D 0.19.0 env + reconstruction scripts
if has open3d; then
  if [ -x "$HOME/venvs/o3d019/bin/python" ]; then
    echo "Open3D env already exists: ~/venvs/o3d019"
  else
    bash "$REPO/engines/open3d_recon/setup_open3d.sh"
  fi
fi
echo "install_engines: done"
