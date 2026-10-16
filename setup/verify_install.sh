#!/usr/bin/env bash
# Check that everything the pipeline needs is installed and runs. Prints OK / WARN / FAIL per item;
# exit status 1 if anything required is missing.
# Usage: bash setup/verify_install.sh [BAG.db3]      (with a bag: also checks size and its .sha256 file)
# Paths follow the same variables as the runners (MV2_TOOLS, RTABMAP_BIN, ORB_SLAM3_DIR, O3D_VENV, O3D_TOOLS).
TOOLS="${MV2_TOOLS:-$HOME/mv2_tools}"
RT="${RTABMAP_BIN:-$TOOLS/rtabmap/bin}"
ORB="${ORB_SLAM3_DIR:-$TOOLS/ORB_SLAM3}"
O3D_ENV="${O3D_VENV:-$HOME/venvs/o3d019}"
O3D_TOOLS="${O3D_TOOLS:-$HOME/mv2_data/tools/open3d-v0.19.0}"
MV2_VENV="${MV2_VENV:-$HOME/venvs/mv2}"
DATA="${MV2_DATA:-$HOME/mv2_data}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
BAG="${1:-}"
FAIL=0
ok()   { echo "OK    $*"; }
warn() { echo "WARN  $*"; }
bad()  { echo "FAIL  $*"; FAIL=1; }

. /etc/os-release 2>/dev/null
[ "${VERSION_ID:-}" = "22.04" ] && ok "Ubuntu 22.04" || warn "OS is ${PRETTY_NAME:-unknown} (tested on 22.04)"
echo "INFO  $(nproc) CPU threads, $(awk '/MemTotal/ {printf "%d", $2/1048576}' /proc/meminfo) GB RAM, repo $(git -C "$REPO" rev-parse --short HEAD 2>/dev/null)"

mkdir -p "$DATA" 2>/dev/null
FREE=$(df --output=avail -BG "$DATA" 2>/dev/null | tail -1 | tr -dc 0-9)
if [ -z "$FREE" ]; then warn "could not read free disk space of $DATA"
elif [ "$FREE" -ge 45 ]; then ok "free disk $FREE GB on $DATA (need ~35 GB, 45 recommended)"
elif [ "$FREE" -ge 30 ]; then warn "free disk only $FREE GB on $DATA (tight: 35 GB needed)"
else bad "free disk only $FREE GB on $DATA (need ~35 GB)"; fi

# RTAB-Map
if V=$("$RT/rtabmap-rgbd_dataset" --version 2>&1 | head -1) && [[ "$V" == RTAB-Map:* ]]; then ok "RTAB-Map runs: ${V//  /}"
  for t in rtabmap-export rtabmap-detectMoreLoopClosures rtabmap-globalBundleAdjustment rtabmap-info; do
    [ -x "$RT/$t" ] || bad "missing $RT/$t"
  done
else bad "RTAB-Map not usable in $RT (run: bash setup/install_engines.sh rtabmap)"; fi

# ORB-SLAM3
export LD_LIBRARY_PATH="$ORB/lib:$ORB/Thirdparty/DBoW2/lib:$ORB/Thirdparty/g2o/lib:$TOOLS/pangolin/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"
if [ -x "$ORB/Examples/RGB-D/rgbd_tum" ] && [ -f "$ORB/Vocabulary/ORBvoc.txt" ]; then
  MISSING=$(ldd "$ORB/Examples/RGB-D/rgbd_tum" 2>&1 | grep "not found" | head -3)
  [ -n "$MISSING" ] && bad "ORB-SLAM3 libraries missing: $MISSING" || ok "ORB-SLAM3 built in $ORB"
  grep -q "SaveTrajectoryEuRoC" "$ORB/Examples/RGB-D/rgbd_tum.cc" 2>/dev/null \
    && ok "ORB-SLAM3 patches present (incl. biggest-map trajectory)" \
    || bad "ORB-SLAM3 is an older build without the biggest-map patch (rebuild: rm -rf $ORB; bash setup/install_engines.sh orbslam3)"
else bad "ORB-SLAM3 not built in $ORB (run: bash setup/install_engines.sh orbslam3)"; fi

# Open3D reconstruction engine (0.19.0)
if [ -x "$O3D_ENV/bin/python" ] && OV=$("$O3D_ENV/bin/python" -c "import open3d; print(open3d.__version__)" 2>/dev/null); then
  [ "$OV" = "0.19.0" ] && ok "Open3D engine env: $OV" || warn "Open3D engine env has $OV (expected 0.19.0)"
else bad "Open3D engine env missing: $O3D_ENV (run: bash setup/install_engines.sh open3d)"; fi
[ -f "$O3D_TOOLS/examples/python/reconstruction_system/run_system.py" ] \
  && ok "Open3D reconstruction scripts in $O3D_TOOLS" || bad "Open3D scripts missing in $O3D_TOOLS (run: bash engines/open3d_recon/setup_open3d.sh)"

# project venv
if [ -x "$MV2_VENV/bin/python" ]; then
  OUT=$("$MV2_VENV/bin/python" -c "
import sys, pyrealsense2, open3d, cv2, numpy, scipy, yaml, zstandard, rosbags
print(sys.version.split()[0], open3d.__version__, pyrealsense2.__version__)" 2>&1 | tail -1)
  [[ "$OUT" == 3.10.* ]] && ok "project venv: python/open3d/pyrealsense2 = $OUT" || bad "project venv broken: $OUT (rm -rf $MV2_VENV; bash setup/make_venv.sh)"
else bad "project venv missing: $MV2_VENV (run: bash setup/make_venv.sh)"; fi

# bag
if [ -n "$BAG" ]; then
  if [ -f "$BAG" ]; then
    SZ=$(stat -c %s "$BAG"); ok "bag $BAG: $((SZ / 1048576)) MB"
    SUM="${BAG%.db3}.jetson.sha256"
    if [ -f "$SUM" ]; then
      A=$(sha256sum "$BAG" | cut -d' ' -f1); B=$(cut -d' ' -f1 "$SUM")
      [ "$A" = "$B" ] && ok "bag sha256 matches the Jetson's ($SUM)" || bad "bag sha256 DIFFERS from the Jetson's: copy it again"
    else warn "no $SUM to compare against"; fi
  else bad "bag not found: $BAG"; fi
fi
echo; [ "$FAIL" -eq 0 ] && echo "VERIFY: ALL OK" || echo "VERIFY: PROBLEMS FOUND (see FAIL lines)"
exit "$FAIL"
