#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WINDOWS_POWERSHELL="/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"
WINDOWS_LAUNCHER="$(wslpath -w "$ROOT_DIR/scripts/run_live_c920.ps1")"

if [[ ! -x "$WINDOWS_POWERSHELL" ]]; then
  echo "Windows PowerShell was not found at $WINDOWS_POWERSHELL" >&2
  exit 1
fi

exec "$WINDOWS_POWERSHELL" \
  -NoLogo \
  -NoProfile \
  -ExecutionPolicy Bypass \
  -File "$WINDOWS_LAUNCHER" \
  "$@"
