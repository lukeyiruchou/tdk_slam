#!/bin/bash

MAPS_DIR="/home/tdk/tdk_slam_ws/src/tdk_slam_manager/maps"
echo "input: $0 [slamtb | carto]"

MODE=$1

case "$MODE" in
    "slamtb")
        PREFIX="slam_map_"
        EXT=".posegraph"
        ;;
    "carto")
        PREFIX="carto_map_"
        EXT=".pbstream"
        ;;
    *)
        exit 1
        ;;
esac

INDEX=0
while [ -f "${MAPS_DIR}/${PREFIX}${INDEX}${EXT}" ]; do
    INDEX=$((INDEX+1))
done

FILENAME="${PREFIX}${INDEX}"
FULL_PATH="${MAPS_DIR}/${FILENAME}"

case "$MODE" in
    "slamtb")
        ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${FULL_PATH}'}" # for localization mode of slam_toolbox pkg
        ;;
        
    "carto")
        echo "Step 1: 停止 Cartographer 軌跡..."
        ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
        
        echo "Step 2: 儲存包含未完成子圖的狀態..."
        ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${FULL_PATH}.pbstream', include_unfinished_submaps: true}"
        
        echo "Step 3: 序列化完整地圖狀態..."
        ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${FULL_PATH}.pbstream'}"
        
        echo "Step 4: 將 pbstream 轉換為 Nav2 圖片與 YAML..."
        ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
            -pbstream_filename "${FULL_PATH}.pbstream" \
            -map_filestem "${FULL_PATH}" \
            -resolution 0.05
        ;;
esac
