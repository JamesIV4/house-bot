#!/usr/bin/env bash
# Capture a DPV-SLAM trajectory while driving the timestamped scale route, then
# solve the metric scale.
#
# The route must run *inside* the capture window: DPV-SLAM needs camera motion
# to initialise, and align_dpvo_scale.py matches each straight leg's command
# timestamps against poses in the saved trajectory.
#
# Usage: ./scripts/run_scale_route.sh [pi-host] [max-poses]
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PI_HOST="${1:-192.168.0.241}"
MAX_POSES="${2:-600}"
# Anything after the first two arguments is forwarded to the route driver.
ROUTE_ARGS=("${@:3}")
SCALE_ROUTE_FILE="${SCALE_ROUTE_FILE:-$ROOT_DIR/config/scale-route.json}"
SCALE_ROUTE_GAP="${SCALE_ROUTE_GAP:-2.5}"
OUTPUT_DIR="$ROOT_DIR/data/output/dpvo-pi-live"
ROUTE_DIR="$ROOT_DIR/data/output/scale-routes"
STAMP="$(date +%Y%m%d-%H%M%S)"
ROUTE_LOG="$ROUTE_DIR/scale-route-$STAMP.json"
SCALE_OUT="$ROUTE_DIR/scale-$STAMP.json"
WARMUP_S="${SCALE_ROUTE_WARMUP_S:-6}"

BASE_SUMMARY="$ROOT_DIR/config/local/base_calibration.summary.json"
MEASUREMENTS="$ROOT_DIR/config/local/base_calibration_measurements.json"

for required in "$BASE_SUMMARY" "$MEASUREMENTS"; do
    if [[ ! -f "$required" ]]; then
        echo "Missing $required; run scripts/calibrate_base.py first." >&2
        exit 1
    fi
done

mkdir -p "$ROUTE_DIR" "$OUTPUT_DIR"

# Snapshot existing trajectories so we can identify the one this run produces.
BEFORE="$(mktemp)"
ls -1 "$OUTPUT_DIR"/*.tum.txt 2>/dev/null | sort > "$BEFORE" || true

echo "=== starting DPV-SLAM capture (max ${MAX_POSES} poses) ==="
"$ROOT_DIR/scripts/run_pi_camera_dpvo.sh" "$PI_HOST" 5600 "$MAX_POSES" \
    > "$ROUTE_DIR/dpvo-$STAMP.log" 2>&1 &
DPVO_PID=$!

cleanup() {
    if kill -0 "$DPVO_PID" 2>/dev/null; then
        echo "Stopping DPV-SLAM capture..."
        kill "$DPVO_PID" 2>/dev/null || true
        wait "$DPVO_PID" 2>/dev/null || true
    fi
    rm -f "$BEFORE"
}
trap cleanup EXIT INT TERM

echo "Waiting ${WARMUP_S}s for the camera stream and tracker to warm up..."
sleep "$WARMUP_S"
if ! kill -0 "$DPVO_PID" 2>/dev/null; then
    echo "DPV-SLAM capture exited during warm-up. See $ROUTE_DIR/dpvo-$STAMP.log" >&2
    exit 1
fi

echo "=== driving the scale route ==="
echo "Watch both treads for the whole route; a dropout invalidates the scale fit."
# Driven by drive_route.py: one process, one socket, one session, and an
# uninterrupted command stream with gaps sent as explicit zeros. The previous
# driver held heading by dropping command slots on one tread at 20 Hz, and every
# dropped slot is a set_wheels transition that makes the Pi reclaim a GPIO line
# instead of catching a 215 us scan window. Scale alignment needs each straight
# leg to be *timestamped*, not straight, so open-loop constant-power legs are
# both safer and sufficient.
python3 "$ROOT_DIR/scripts/drive_route.py" \
    --host "$PI_HOST" --route-file "$SCALE_ROUTE_FILE" --log "$ROUTE_LOG" \
    --gap "$SCALE_ROUTE_GAP" --imu-log --execute \
    "${ROUTE_ARGS[@]}"
ROUTE_RESULT=$?

echo "=== waiting for DPV-SLAM to finish writing ==="
wait "$DPVO_PID" || true
trap - EXIT INT TERM
rm -f "$BEFORE.done" 2>/dev/null || true

TRAJECTORY="$(ls -1 "$OUTPUT_DIR"/*.tum.txt 2>/dev/null | sort | comm -13 "$BEFORE" - | tail -1)"
rm -f "$BEFORE"
if [[ -z "$TRAJECTORY" ]]; then
    echo "No new trajectory was produced; DPV-SLAM likely failed to initialise." >&2
    echo "See $ROUTE_DIR/dpvo-$STAMP.log" >&2
    exit 1
fi
METRICS="${TRAJECTORY%.tum.txt}.json"
echo "Trajectory: $TRAJECTORY"
echo "Metrics:    $METRICS"
echo "Route log:  $ROUTE_LOG"

if [[ "$ROUTE_RESULT" -ne 0 ]]; then
    echo "Route did not complete cleanly; not attempting scale alignment." >&2
    exit "$ROUTE_RESULT"
fi

cat <<NEXT

=== route and capture complete ===
Scale alignment requires you to confirm that BOTH treads drove for the whole
route. Acknowledged packets do not prove physical motion, and the right tread
has been intermittent. If you saw any dropout, discard this run.

If both treads ran clean, solve the scale with:

  python3 scripts/align_dpvo_scale.py \\
    --trajectory "$TRAJECTORY" \\
    --metrics "$METRICS" \\
    --route "$ROUTE_LOG" \\
    --base-summary "$BASE_SUMMARY" \\
    --measurements "$MEASUREMENTS" \\
    --output "$SCALE_OUT" \\
    --accept-observed-motion
NEXT
