#!/usr/bin/env bash
set -e -o pipefail

export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "[A] tcp_plc_node 실행 (계속 대기)"

ros2 launch tcp_plc_reader tcp_plc.launch.py \
  params_file:=$HOME/ros2_ws/src/tcp_plc_reader/config/params.yaml

