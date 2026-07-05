#!/bin/bash
# Process KITTI LiDAR frames through DBSCAN pipeline
# Usage: bash scripts/run_kitti.sh <velodyne_data_dir> [num_frames=10]

set -e
LIDAR_DIR="${1:-data/kitti/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data}"
N_FRAMES="${2:-10}"
BIN="./build-algo/bin/flow_nuscenes_demo"

if [ ! -f "$BIN" ]; then
    echo "Building flow_nuscenes_demo..."
    cmake --build build-algo --target flow_nuscenes_demo -j$(nproc)
fi

echo "=== KITTI DBSCAN Pipeline ==="
echo "LiDAR dir: $LIDAR_DIR"
echo "Frames:    $N_FRAMES"
echo ""

total_detections=0
total_clusters=0
total_points=0
processed=0

for f in $(ls "$LIDAR_DIR"/*.bin 2>/dev/null | head -n "$N_FRAMES"); do
    frame=$(basename "$f" .bin)
    result=$($BIN "$f" "" 2.0 12 2>&1)
    pts=$(echo "$result" | grep "DBSCAN:" | grep -oP '\d+ pts' | grep -oP '\d+')
    cl=$(echo "$result" | grep "DBSCAN:" | grep -oP '\d+ clusters' | grep -oP '\d+')
    det=$(echo "$result" | grep -cE "vehicle @|pedestrian @|cyclist @" 2>/dev/null || echo 0)
    total_points=$((total_points + pts))
    total_clusters=$((total_clusters + cl))
    total_detections=$((total_detections + det))
    processed=$((processed + 1))
    echo "[$processed] $frame: ${pts}pts → ${cl}clusters, ${det}objects"
done

echo ""
echo "=== Summary ==="
echo "Frames processed: $processed"
echo "Total points:     $total_points"
echo "Total clusters:   $total_clusters"
echo "Total detections: $total_detections"
echo "Avg points/frame: $((total_points / (processed > 0 ? processed : 1)))"
echo "Avg objects/frame: $((total_detections / (processed > 0 ? processed : 1)))"
