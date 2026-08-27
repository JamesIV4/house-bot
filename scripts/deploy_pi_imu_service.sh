#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
    echo "Usage: $0 [pi-host-or-ip]" >&2
    exit 2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pi_host="${1:-192.168.0.241}"
ssh_key="${HOUSEBOT_SSH_KEY:-$HOME/.ssh/housebot_ed25519}"
remote_stage="/home/james/house-bot-deploy"

if [[ ! -f "$ssh_key" ]]; then
    echo "House Bot SSH key is missing: $ssh_key" >&2
    exit 1
fi

ssh_args=(-i "$ssh_key" -o BatchMode=yes)
ssh "${ssh_args[@]}" "james@$pi_host" "mkdir -p '$remote_stage'"
scp "${ssh_args[@]}" \
    "$repo_root/scripts/imu_service.py" \
    "$repo_root/scripts/mpu6050.py" \
    "$repo_root/deploy/house-bot-imu.service" \
    "james@$pi_host:$remote_stage/"
ssh "${ssh_args[@]}" "james@$pi_host" \
    "install -m 0755 '$remote_stage/imu_service.py' /home/james/imu_service.py && \
     install -m 0644 '$remote_stage/mpu6050.py' /home/james/mpu6050.py && \
     mkdir -p /home/james/.config/systemd/user && \
     install -m 0644 '$remote_stage/house-bot-imu.service' /home/james/.config/systemd/user/house-bot-imu.service && \
     systemctl --user daemon-reload && \
     systemctl --user restart house-bot-imu.service && \
     sleep 4 && \
     systemctl --user is-active house-bot-imu.service && \
     journalctl --user -u house-bot-imu.service -n 12 --no-pager"

echo "Pi IMU service deployed on $pi_host, publishing yaw on UDP 8766."
