#!/usr/bin/env bash
set -euo pipefail

DEVICE="${1:-/dev/video0}"
PORT="${2:-5600}"
WIDTH="${C920_WIDTH:-1280}"
HEIGHT="${C920_HEIGHT:-720}"
FPS="${C920_FPS:-30}"
CODEC="${C920_CODEC:-h264}"
AUTOFOCUS="${C920_AUTOFOCUS:-}"
FOCUS_ABSOLUTE="${C920_FOCUS_ABSOLUTE:-}"

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg is required on the Raspberry Pi." >&2
  exit 1
fi

if [[ ! -e "$DEVICE" ]]; then
  echo "Camera device not found: $DEVICE" >&2
  exit 1
fi

if command -v v4l2-ctl >/dev/null 2>&1; then
  # Low-light exposure may otherwise reduce a nominal 30 FPS mode to roughly
  # half rate, which is harmful to visual odometry.
  v4l2-ctl -d "$DEVICE" -c exposure_dynamic_framerate=0 >/dev/null
  if [[ -n "$AUTOFOCUS" ]]; then
    if [[ "$AUTOFOCUS" == "0" && -n "$FOCUS_ABSOLUTE" ]]; then
      # The C920 leaves focus_absolute inactive for a short time after
      # autofocus is disabled, so these must be separate control transactions.
      v4l2-ctl -d "$DEVICE" -c focus_automatic_continuous=0 >/dev/null
      sleep 0.25
      v4l2-ctl -d "$DEVICE" -c "focus_absolute=$FOCUS_ABSOLUTE" >/dev/null
      echo "Focus: fixed at $FOCUS_ABSOLUTE"
    else
      v4l2-ctl -d "$DEVICE" -c "focus_automatic_continuous=$AUTOFOCUS" >/dev/null
      echo "Continuous autofocus: $AUTOFOCUS"
    fi
  fi
fi

echo "Serving $DEVICE as native $CODEC on TCP port $PORT"
echo "Mode: ${WIDTH}x${HEIGHT} at ${FPS} FPS"

case "$CODEC" in
  h264)
    # The C920 produces H.264 itself. FFmpeg only supplies timestamps and a
    # transport container, keeping Pi 3B CPU use and Wi-Fi bandwidth low.
    exec ffmpeg \
      -hide_banner \
      -loglevel warning \
      -thread_queue_size 512 \
      -fflags +genpts \
      -use_wallclock_as_timestamps 1 \
      -f v4l2 \
      -input_format h264 \
      -video_size "${WIDTH}x${HEIGHT}" \
      -framerate "$FPS" \
      -i "$DEVICE" \
      -an \
      -r "$FPS" \
      -c:v copy \
      -muxdelay 0 \
      -muxpreload 0 \
      -flush_packets 1 \
      -f mpegts \
      "tcp://0.0.0.0:${PORT}?listen=1&tcp_nodelay=1"
    ;;
  mjpeg)
    exec ffmpeg \
      -hide_banner \
      -loglevel warning \
      -thread_queue_size 512 \
      -f v4l2 \
      -input_format mjpeg \
      -video_size "${WIDTH}x${HEIGHT}" \
      -framerate "$FPS" \
      -i "$DEVICE" \
      -an \
      -c:v copy \
      -f mjpeg \
      "tcp://0.0.0.0:${PORT}?listen=1&tcp_nodelay=1"
    ;;
  *)
    echo "Unsupported C920_CODEC: $CODEC (expected h264 or mjpeg)" >&2
    exit 2
    ;;
esac
