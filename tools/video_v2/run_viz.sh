#!/usr/bin/env bash
# Re-run of scripts/run_nav.sh with an extra recorder for particles / global plan / scans.
NAME=$1; PARAMS=$(realpath /workspace/ros2-amr-slam-nav2/config/nav2_$NAME.yaml)
cd /workspace/ros2-amr-slam-nav2
export MAMBA_ROOT_PREFIX=/workspace/ros/mamba
eval "$(/workspace/ros/bin/micromamba shell hook -s bash)"; micromamba activate humble
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-52} RCUTILS_LOGGING_BUFFERED_STREAM=0
O=/workspace/post3_video/runs/$NAME; mkdir -p $O
python3 amr_sim/sim_node.py --ros-args -p use_sim_time:=false > $O/sim.log 2>&1 & P1=$!
sleep 2
python3 /workspace/post3_video/viz_recorder.py $O/viz.pkl --ros-args -p use_sim_time:=true > $O/rec.log 2>&1 & P2=$!
ros2 launch nav2_bringup bringup_launch.py slam:=False map:=$(realpath maps/warehouse.yaml) \
  params_file:=$PARAMS use_sim_time:=True autostart:=True use_composition:=False > $O/nav2.log 2>&1 & P3=$!
timeout 3600 python3 amr_sim/goal_runner.py $O/goals.json --ros-args -p use_sim_time:=true > $O/goals.log 2>&1
for p in $P2 $P3 $P1; do kill -INT $p 2>/dev/null; done; sleep 4
pkill -9 -f "lib/nav2_|lib/amcl|lib/nav2_map_server|lifecycle_manager|python3 amr_sim/|viz_recorder" 2>/dev/null
echo DONE $NAME; grep -c CONTACT $O/sim.log
