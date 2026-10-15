#!/usr/bin/env bash
# One-time setup of the Open3D reconstruction-system engine, isolated from the project venv.
#
# Why a separate environment: in Open3D 0.20.0 (both open3d and open3d-cpu) the legacy
# ScalableTSDFVolume used by the reconstruction system fuses nothing (checked 2026-10-15:
# 0 triangles; 0.19.0 gives a normal mesh). So this engine runs Open3D 0.19.0 with the matching
# v0.19.0 scripts; the project's own tools stay on 0.20.0 (they use the working VoxelBlockGrid).
#
# Creates ~/venvs/o3d019 and ~/mv2_data/tools/open3d-v0.19.0 (scripts from the v0.19.0 tag).
# Usage: bash engines/open3d_recon/setup_open3d.sh
set -euo pipefail
VENV="${O3D_VENV:-$HOME/venvs/o3d019}"
TOOLS="${O3D_TOOLS:-$HOME/mv2_data/tools/open3d-v0.19.0}"
PY="${PYTHON:-python3}"

case "$(uname -m)" in
  x86_64)  O3D="open3d-cpu==0.19.0" ;;
  aarch64) O3D="open3d==0.18.0"; echo "NOTE: aarch64 has no 0.19.0 wheel; using 0.18.0 (untested)";;
  *) echo "unsupported machine $(uname -m)"; exit 1 ;;
esac
if [ ! -x "$VENV/bin/python" ]; then
  "$PY" -m venv "$VENV"
  "$VENV/bin/python" -m pip install --upgrade pip
  "$VENV/bin/python" -m pip install "$O3D" numpy==1.26.4 opencv-python-headless==4.11.0.86
fi
"$VENV/bin/python" -c "import open3d; print('engine Open3D', open3d.__version__)"

if [ ! -f "$TOOLS/examples/python/reconstruction_system/run_system.py" ]; then
  rm -rf "$TOOLS"
  mkdir -p "$(dirname "$TOOLS")"
  git clone -q --depth 1 --branch v0.19.0 --filter=blob:none --no-checkout https://github.com/isl-org/Open3D "$TOOLS"
  git -C "$TOOLS" sparse-checkout set --no-cone /examples/python/reconstruction_system/ /examples/python/open3d_example.py
  git -C "$TOOLS" checkout -q
fi
echo "commit: $(git -C "$TOOLS" rev-parse HEAD)"
echo "Open3D engine ready: venv $VENV, scripts $TOOLS"
