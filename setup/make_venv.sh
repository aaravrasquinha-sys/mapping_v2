#!/usr/bin/env bash
# Create the project's Python environment (default ~/venvs/mv2) with pinned versions.
# Usage: bash setup/make_venv.sh [VENV_DIR]
# Env:   PYTHON=python3.10 to choose the interpreter (default: python3)
set -euo pipefail
cd "$(dirname "$0")/.."
VENV="${1:-$HOME/venvs/mv2}"
PY="${PYTHON:-python3}"

if ! "$PY" -c 'import sys; sys.exit(0 if sys.version_info[:2] == (3, 10) else 1)'; then
  echo "WARNING: $PY is $("$PY" --version 2>&1); the pins were checked for Python 3.10."
fi
if [ -e "$VENV" ]; then
  echo "ERROR: $VENV already exists. Remove it first (rm -rf $VENV) or pass another path." >&2
  exit 1
fi
if ! "$PY" -m venv "$VENV"; then
  echo "ERROR: could not create a venv. On Ubuntu 22.04: sudo apt install python3.10-venv" >&2
  rm -rf "$VENV"
  exit 1
fi
"$VENV/bin/python" -m pip install --upgrade pip
"$VENV/bin/python" -m pip install -r setup/requirements.txt
case "$(uname -m)" in
  x86_64)  "$VENV/bin/python" -m pip install "open3d-cpu==0.20.0" ;;
  aarch64) "$VENV/bin/python" -m pip install "open3d==0.20.0" ;;
  *) echo "ERROR: unsupported machine $(uname -m)" >&2; exit 1 ;;
esac
"$VENV/bin/python" - <<'EOF'
import pyrealsense2 as rs, numpy, cv2, zstandard, rosbags, yaml, scipy, open3d
print("imports OK: pyrealsense2", rs.__version__, "| open3d", open3d.__version__, "| opencv", cv2.__version__,
      "| numpy", numpy.__version__)
EOF
"$VENV/bin/python" -m pip freeze > "$VENV/freeze.txt"
echo "Environment ready: $VENV   (activate with: source $VENV/bin/activate)"
echo "Exact package list saved in $VENV/freeze.txt"
