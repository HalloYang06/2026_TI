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
exec /home/halloyang/2026_TI/vision/raspberrypi/ball_camera/build/ball_vision_sender \
  --camera "$camera_device" --width 640 --height 480 --fps 120 --port 8080 \
  --roi 60,225,560,75 --left-cm -11.44 --right-cm 12.31 \
  --threshold 170 --min-area 50 --max-area 1200 --max-center-offset 18 --edge-ignore 28
