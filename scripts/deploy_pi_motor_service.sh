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
    "$repo_root/scripts/pi_motor_service.py" \
    "$repo_root/scripts/remote_gpio_controller.py" \
    "$repo_root/deploy/house-bot-motors.service" \
    "james@$pi_host:$remote_stage/"
ssh "${ssh_args[@]}" "james@$pi_host" \
    "install -m 0755 '$remote_stage/pi_motor_service.py' /home/james/pi_motor_service.py && \
     install -m 0644 '$remote_stage/remote_gpio_controller.py' /home/james/remote_gpio_controller.py && \
     mkdir -p /home/james/.config/systemd/user && \
     install -m 0644 '$remote_stage/house-bot-motors.service' /home/james/.config/systemd/user/house-bot-motors.service && \
     systemctl --user daemon-reload && \
     systemctl --user restart house-bot-motors.service && \
     systemctl --user is-active house-bot-motors.service"

echo "Pi motor service deployed and active on $pi_host. Motors remain released."
