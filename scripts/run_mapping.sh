#!/usr/bin/env bash
# Phase 1: drive the mapping loop with slam_toolbox (online async) and save the map.
cd "$(dirname "$0")/.."
export MAMBA_ROOT_PREFIX=/workspace/ros/mamba
eval "$(/workspace/ros/bin/micromamba shell hook -s bash)"; micromamba activate humble
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-41} RCUTILS_LOGGING_BUFFERED_STREAM=0
mkdir -p results/logs maps
L=results/logs
python3 amr_sim/sim_node.py --ros-args -p use_sim_time:=false > $L/map_sim.log 2>&1 & P1=$!
sleep 2
python3 amr_sim/map_recorder.py results/mapping_run.npz --ros-args -p use_sim_time:=true > $L/map_rec.log 2>&1 & P2=$!
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file config/slam.yaml -p use_sim_time:=true > $L/slam.log 2>&1 & P3=$!
sleep 4
timeout 900 python3 amr_sim/tour_node.py --ros-args -p use_sim_time:=true > $L/tour.log 2>&1
sleep 5
ros2 run nav2_map_server map_saver_cli -f maps/warehouse --ros-args -p use_sim_time:=true > $L/saver.log 2>&1
for p in $P2 $P3 $P1; do kill -INT $p 2>/dev/null; done; sleep 3
for p in $P2 $P3 $P1; do kill -9 $p 2>/dev/null; done
grep -c "map " $L/map_rec.log; tail -2 $L/tour.log; ls maps
