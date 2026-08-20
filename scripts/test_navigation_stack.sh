#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose_file="$repo_root/navigation/compose.yaml"
ui_port="${HOUSE_BOT_UI_PORT:-5000}"

compose() {
    docker compose -f "$compose_file" "$@"
}

compose up --detach --build

for _ in $(seq 1 90); do
    if curl --fail --silent "http://localhost:${ui_port}" >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

if ! curl --fail --silent "http://localhost:${ui_port}" >/dev/null; then
    compose logs --tail=200 navigation >&2
    exit 1
fi

nav_ready=false
for _ in $(seq 1 60); do
    lifecycle_state="$(
        compose exec --no-TTY navigation /opt/house_bot/entrypoint.sh \
            ros2 lifecycle get /bt_navigator 2>/dev/null || true
    )"
    # controller_server cannot activate until the loopback simulator has
    # supplied odom -> base_link. bt_navigator reaching active therefore gates
    # both the action server and the transform without requiring wheel motion.
    if [[ "$lifecycle_state" == *"active"* ]]; then
        nav_ready=true
        break
    fi
    sleep 1
done
if ! $nav_ready; then
    compose logs --tail=200 navigation >&2
    echo "Nav2 did not become active with live loopback odometry." >&2
    exit 1
fi

compose exec --no-TTY navigation \
    /opt/house_bot/entrypoint.sh \
    python3 -m pytest -q /opt/house_bot/src/house_bot_navigation/test
compose exec --no-TTY navigation \
    /opt/house_bot/entrypoint.sh \
    ros2 run house_bot_navigation navigation_smoke_test

echo "Navigation unit and end-to-end loopback tests passed."
