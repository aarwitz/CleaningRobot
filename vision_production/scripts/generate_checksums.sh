#!/usr/bin/env bash
set -euo pipefail

# Generate checksums for critical model and configuration files

OUTPUT="${1:-checksums.txt}"

echo "# Vision Production Checksums - Generated $(date -u +%Y-%m-%dT%H:%M:%SZ)" > "${OUTPUT}"
echo "" >> "${OUTPUT}"

if [ -f "models/yolov8s.onnx" ]; then
  echo "yolov8s.onnx: $(sha256sum models/yolov8s.onnx | awk '{print $1}')" >> "${OUTPUT}"
fi

if [ -f "models/yolov8s.plan" ]; then
  echo "yolov8s.plan: $(sha256sum models/yolov8s.plan | awk '{print $1}')" >> "${OUTPUT}"
fi

if [ -f "launch/vision_bringup.launch.py" ]; then
  echo "vision_bringup.launch.py: $(sha256sum launch/vision_bringup.launch.py | awk '{print $1}')" >> "${OUTPUT}"
fi

if [ -f "docker/Dockerfile" ]; then
  echo "Dockerfile: $(sha256sum docker/Dockerfile | awk '{print $1}')" >> "${OUTPUT}"
fi

echo "" >> "${OUTPUT}"
echo "Checksums written to ${OUTPUT}"
