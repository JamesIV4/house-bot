#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose_file="$repo_root/navigation/compose.yaml"
ui_port="${HOUSE_BOT_UI_PORT:-5000}"
ui_url="http://localhost:${ui_port}"

usage() {
    echo "Usage: $0 [start|stop|restart|logs|status|test] [--no-open]"
}

command_name="${1:-start}"
if [[ $# -gt 0 ]]; then
    shift
fi
open_browser=true
for argument in "$@"; do
    case "$argument" in
        --no-open) open_browser=false ;;
        *) usage; exit 2 ;;
    esac
done

compose() {
    docker compose -f "$compose_file" "$@"
}

wait_for_ui() {
    for _ in $(seq 1 90); do
        if curl --fail --silent --show-error "$ui_url" >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    echo "UI did not become ready at $ui_url" >&2
    compose logs --tail=160 navigation >&2
    return 1
}

start_stack() {
    compose up --detach --build
    wait_for_ui
    echo "House Bot navigation UI: $ui_url"
    if $open_browser && command -v powershell.exe >/dev/null 2>&1; then
        powershell.exe -NoProfile -Command "Start-Process '$ui_url'" >/dev/null
    fi
}

case "$command_name" in
    start) start_stack ;;
    stop) compose down ;;
    restart)
        compose down
        start_stack
        ;;
    logs) compose logs --follow --tail=160 navigation ;;
    status) compose ps ;;
    test) "$repo_root/scripts/test_navigation_stack.sh" ;;
    *) usage; exit 2 ;;
esac

