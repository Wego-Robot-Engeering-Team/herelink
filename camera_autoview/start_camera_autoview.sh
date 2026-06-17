#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="/home/agilex/agilex_ws"

sleep 5

set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

exec ros2 launch limo_ultra_bringup camera_autoview.launch.py \
  fullscreen:=false \
  maximize_window:=true \
  fill_mode:=crop
