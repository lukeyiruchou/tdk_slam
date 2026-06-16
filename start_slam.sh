#!/bin/bash

SESSION="slam"
SOURCE="source /opt/ros/humble/setup.bash && source install/setup.bash"

tmux new-session -d -s $SESSION

# Pane 0: maze_world
tmux send-keys -t $SESSION:0 "$SOURCE && ros2 launch tdk_slam_manager maze_world_launch.py" Enter

# Pane 1: sim_spawn
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "$SOURCE && ros2 launch tdk_slam_manager sim_spawn_launch.py" Enter

# Window 1 Pane 0: nav2
tmux new-window -t $SESSION:1
tmux send-keys -t $SESSION:1 "$SOURCE && ros2 launch tdk_nav2_manager nav_launch.py" Enter

# Window 1 Pane 1: rviz2
tmux split-window -h -t $SESSION:1
tmux send-keys -t $SESSION:1.1 "$SOURCE && rviz2" Enter

# attach
tmux attach -t $SESSION