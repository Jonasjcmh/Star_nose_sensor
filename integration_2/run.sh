#!/usr/bin/env bash
# Launcher: ensures ~/.local site-packages are visible even when
# VIRTUAL_ENV=/usr (set by ROS2 sourcing) would otherwise hide them.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USER_SITE="/home/cao/.local/lib/python3.10/site-packages"

export PYTHONPATH="$USER_SITE:$PYTHONPATH"

if [ $# -eq 0 ]; then
    echo "Usage: ./run.sh <script.py> [args...]"
    echo "Example: ./run.sh calibrate_points.py --scan"
    exit 1
fi

cd "$SCRIPT_DIR"
exec python3 "$@"
