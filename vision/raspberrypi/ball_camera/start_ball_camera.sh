#!/bin/sh
set -eu

camera_link=$(find /dev/v4l/by-id -name '*-video-index0' -print -quit)
camera_device=$(readlink -f "$camera_link")
exec /home/halloyang/2026_TI/vision/raspberrypi/ball_camera/build/ball_vision_sender \
  --camera "$camera_device" --width 640 --height 480 --fps 120 --port 8080 \
  --roi 60,225,560,75 --left-cm -12.5 --right-cm 12.5 \
  --threshold 170 --min-area 50 --max-area 1200 --max-center-offset 18 --edge-ignore 28
