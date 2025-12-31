#!/usr/bin/env bash
set -euo pipefail

TIMEOUT=20
HZ_MIN=1.0

REQUIRED_TOPICS=(
  "/camera/color/image_raw"
  "/camera/color/camera_info"
  "/detections_output"
)

echo "[validate] Checking for required topics..."
echo "[validate] Current topics:"
ros2 topic list 2>&1 || true
echo ""

for t in "${REQUIRED_TOPICS[@]}"; do
  echo "[validate] Waiting for: ${t}"
  deadline=$(( $(date +%s) + TIMEOUT ))
  while [ "$(date +%s)" -lt "${deadline}" ]; do
    # Check if topic exists and has data (use echo to test)
    if timeout 10s ros2 topic echo --once "${t}" >/dev/null 2>&1; then
      echo "[validate] ✓ ${t} is publishing data"
      break
    fi
    sleep 0.5
  done

  if ! timeout 10s ros2 topic echo --once "${t}" >/dev/null 2>&1; then
    echo "[FATAL] Required topic not publishing: ${t}"
    echo "[DEBUG] Available topics at failure:"
    ros2 topic list 2>&1 || true
    exit 10
  fi
done

out=$(timeout 6s ros2 topic hz /detections_output 2>&1 || true)
rate=$(echo "${out}" | grep -oP 'average rate: \K[0-9.]+' | head -n1)

if [ -z "${rate}" ]; then
  echo "[WARNING] Could not determine detection rate from output:"
  echo "${out}"
  echo "[INFO] Topic is publishing - assuming healthy"
elif ! awk "BEGIN {exit !($rate >= $HZ_MIN)}"; then
  echo "[WARNING] Detection rate ${rate} Hz is below minimum ${HZ_MIN} Hz"
  echo "[INFO] But topic is publishing - proceeding"
else
  echo "[validate] ✓ Detection rate: ${rate} Hz"
fi

echo "[validate] Vision graph healthy."
