#!/usr/bin/env bash
# Phase 2: AMCL + Nav2 on the saved map, 20 goals.  usage: run_nav.sh <name> <params.yaml>
cd "$(dirname "$0")/.."
export MAMBA_ROOT_PREFIX=/workspace/ros/mamba
eval "$(/workspace/ros/bin/micromamba shell hook -s bash)"; micromamba activate humble
set -u
NAME=$1; PARAMS=$(realpath $2)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} RCUTILS_LOGGING_BUFFERED_STREAM=0
L=results/logs/$NAME; mkdir -p $L
python3 amr_sim/sim_node.py --ros-args -p use_sim_time:=false > $L/sim.log 2>&1 & P1=$!
sleep 2
python3 amr_sim/map_recorder.py results/nav_$NAME.npz --ros-args -p use_sim_time:=true > $L/rec.log 2>&1 & P2=$!
ros2 launch nav2_bringup bringup_launch.py slam:=False map:=$(realpath maps/warehouse.yaml) \
  params_file:=$PARAMS use_sim_time:=True autostart:=True use_composition:=False > $L/nav2.log 2>&1 & P3=$!
timeout 3600 python3 amr_sim/goal_runner.py results/goals_$NAME.json --ros-args -p use_sim_time:=true > $L/goals.log 2>&1
for p in $P2 $P3 $P1; do kill -INT $p 2>/dev/null; done; sleep 4
pkill -9 -f "lib/nav2_|lib/amcl|lib/nav2_map_server|lifecycle_manager|python3 amr_sim/" 2>/dev/null
grep -c CONTACT $L/sim.log; grep -o "'result': '[A-Z]*'" $L/goals.log | sort | uniq -c
