#!/usr/bin/env bash
# Print the facts the plan depends on. Every line prints something, even when a tool is missing.
# Usage: bash setup/check_machine.sh [JETSON_USER@JETSON_IP]   (default drdo@192.168.1.101)
JETSON="${1:-drdo@192.168.1.101}"
echo "== os";      lsb_release -ds 2>&1 || cat /etc/os-release 2>&1 | head -2
echo "== machine"; uname -m
echo "== docker";  docker --version 2>&1 || echo "docker: NOT INSTALLED"
echo "== daemon";  timeout 10 docker info --format 'server {{.ServerVersion}}, data {{.DockerRootDir}}' 2>&1 | head -3
echo "== groups";  id -nG
echo "== disk";    df -h ~ 2>&1
echo "== gpu";     nvidia-smi -L 2>/dev/null || lspci 2>/dev/null | grep -i -E "vga|3d|nvidia" || echo "gpu: none found"
echo "== cpu/ram"; nproc; free -h | head -2
echo "== python";  python3 --version 2>&1; python3 -c "import ensurepip, venv; print('venv: ok')" 2>&1 | tail -1
echo "== git/ssh"; git --version 2>&1; ssh -V 2>&1
echo "== display"; echo "DISPLAY=${DISPLAY:-none}"
echo "== jetson ($JETSON, may ask for its password)"
ssh -o ConnectTimeout=5 "$JETSON" 'ls -l ~/bags/walk1.db3; df -h ~ | tail -1' 2>&1
