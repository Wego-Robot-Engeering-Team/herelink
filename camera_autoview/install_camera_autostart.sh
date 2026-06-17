#!/usr/bin/env bash

set -euo pipefail

BASE_DIR="/home/agilex/herelink/camera_autoview"
AUTOSTART_DIR="/home/agilex/.config/autostart"
DESKTOP_FILE="${AUTOSTART_DIR}/limo_camera_autostart.desktop"

mkdir -p "${AUTOSTART_DIR}"

cat > "${DESKTOP_FILE}" <<EOF
[Desktop Entry]
Type=Application
Name=LIMO Camera Autoview
Comment=Start the LIMO camera in fullscreen on login
Exec=${BASE_DIR}/start_camera_autoview.sh
Terminal=false
X-GNOME-Autostart-enabled=true
EOF

chmod +x "${BASE_DIR}/start_camera_autoview.sh"

echo "Installed autostart entry:"
echo "  ${DESKTOP_FILE}"
