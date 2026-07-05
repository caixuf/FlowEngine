#!/bin/bash
# Download nuScenes mini dataset (~4GB)
# After download, use flow_nuscenes_demo to process LiDAR data

set -e

NUSCENES_URL="https://www.nuscenes.org/data/v1.0-mini.tgz"
DEST_DIR="${1:-./data/nuscenes}"

echo "Downloading nuScenes mini to $DEST_DIR ..."
echo "Size: ~4GB, may take 10-30 minutes depending on network"
echo ""

mkdir -p "$DEST_DIR"
cd "$DEST_DIR"

if [ -f "v1.0-mini.tgz" ]; then
    echo "v1.0-mini.tgz already exists, skipping download"
else
    wget -c "$NUSCENES_URL" -O v1.0-mini.tgz
fi

echo "Extracting..."
tar xzf v1.0-mini.tgz

echo ""
echo "Done! nuScenes mini extracted to $DEST_DIR"
echo ""
echo "Directory structure:"
echo "  samples/LIDAR_TOP/   — LiDAR point clouds (.bin, ~30k-60k pts each)"
echo "  samples/CAM_FRONT/   — Camera images"
echo "  v1.0-mini/           — Annotations (sample_annotation.json)"
echo ""
echo "Run: ./flow_nuscenes_demo $DEST_DIR/samples/LIDAR_TOP/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin"
