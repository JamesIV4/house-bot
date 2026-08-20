#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose_file="$repo_root/navigation/compose.yaml"
calibration_file="${HOUSE_BOT_BASE_CALIBRATION:-$repo_root/config/local/base_calibration.yaml}"
pi_host="${HOUSE_BOT_PI_HOST:-192.168.0.241}"

if [[ ! -f "$calibration_file" ]]; then
    echo "Measured calibration is missing: $calibration_file" >&2
    echo "Follow docs/BASE_CALIBRATION.md before starting the real base driver." >&2
    exit 2
fi

export HOUSE_BOT_BASE_CALIBRATION="$calibration_file"
docker compose -f "$compose_file" build navigation
if docker container inspect house-bot-base-driver >/dev/null 2>&1; then
    echo "Container house-bot-base-driver already exists; stop it before restarting." >&2
    exit 3
fi
exec docker compose -f "$compose_file" run --rm --no-deps \
    --name house-bot-base-driver navigation \
    ros2 launch house_bot_navigation base.launch.py \
    calibration_file:=/opt/house_bot/config/base_calibration.yaml \
    pi_host:="$pi_host"
