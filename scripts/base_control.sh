#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: $0 enable|disable|status|stop" >&2
}

action="${1:-}"
if [[ $# -ne 1 ]]; then
    usage
    exit 2
fi
if [[ "$(docker inspect --format '{{.State.Running}}' house-bot-base-driver 2>/dev/null || true)" != "true" ]]; then
    echo "Base driver container is not running." >&2
    exit 1
fi

ros2_exec=(docker exec house-bot-base-driver /opt/house_bot/entrypoint.sh ros2)
case "$action" in
    enable)
        "${ros2_exec[@]}" service call /house_bot/base/enable \
            std_srvs/srv/SetBool "{data: true}"
        ;;
    disable)
        "${ros2_exec[@]}" service call /house_bot/base/enable \
            std_srvs/srv/SetBool "{data: false}"
        ;;
    status)
        "${ros2_exec[@]}" topic echo --once /house_bot/base/status
        ;;
    stop)
        # The node's shutdown hook sends redundant zero commands; the Pi also
        # releases the remote matrix independently after its 350 ms watchdog.
        docker stop --time 2 house-bot-base-driver >/dev/null
        ;;
    *)
        usage
        exit 2
        ;;
esac
