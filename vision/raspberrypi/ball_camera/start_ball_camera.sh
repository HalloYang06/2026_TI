#!/bin/sh
set -eu

while :; do
  camera_link=$(find /dev/v4l/by-id -name '*-video-index0' -print -quit 2>/dev/null || true)
  if [ -n "$camera_link" ]; then
    camera_device=$(readlink -f "$camera_link")
    break
  fi
  if [ -e /dev/video0 ]; then
    camera_device=/dev/video0
    break
  fi
  sleep 1
done
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec "$script_dir/build/ball_vision_sender" \
  --camera "$camera_device" --width 640 --height 480 --fps 120 --stream-fps 60 --port 8080 \
  --roi 60,225,560,75 \
  --threshold 170 --min-area 80 --max-area 650 --max-center-offset 18 --edge-ignore 28
