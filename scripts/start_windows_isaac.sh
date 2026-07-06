#!/usr/bin/env bash
set -euo pipefail

default_wsl_alfa_dir="/mnt/c/Users/Administrator/Desktop/alfa_ws/alfa_demo_direct"
windows_alfa_dir="${WINDOWS_ALFA_DIR:-$(wslpath -w "$default_wsl_alfa_dir")}"

if ! command -v powershell.exe >/dev/null 2>&1; then
  echo "powershell.exe not found. Start Windows Isaac manually:" >&2
  echo "  cd C:\\Users\\Administrator\\Desktop\\alfa_ws\\alfa_demo_direct" >&2
  echo "  .\\run_ros2_direct.bat" >&2
  exit 1
fi

ps_command="\$wd='${windows_alfa_dir}'; Start-Process -FilePath 'cmd.exe' -ArgumentList '/k','run_ros2_direct.bat' -WorkingDirectory \$wd"

if [ "${DRY_RUN:-0}" = "1" ]; then
  echo "DRY_RUN powershell command:"
  echo "$ps_command"
  exit 0
fi

powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "$ps_command"

echo "Windows Isaac start requested:"
echo "  ${windows_alfa_dir}\\run_ros2_direct.bat"
echo "Wait for DDS readiness with:"
echo "  alfa_wait_win"
