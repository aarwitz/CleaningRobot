#!/usr/bin/env bash
# Quick rebuild script for vision_production container

set -e

cd "$(dirname "$0")/.."

echo "=========================================="
echo "Building vision_production:1.0"
echo "=========================================="
echo ""

# Build the Docker image
docker build --no-cache -t vision_production:1.0 -f docker/Dockerfile .

echo ""
echo "=========================================="
echo "Build Complete!"
echo "=========================================="
echo ""
echo "To start the container:"
echo "  docker compose -f docker/docker-compose.yml up"
echo ""
echo "To verify SLAM:"
echo "  docker exec -it docker-vision-1 bash"
echo "  /opt/vision/scripts/verify_slam.sh"
echo ""
