#!/bin/bash

# 定義要執行的指令
CMD1="ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6 --baudrate 1000000"
CMD2="ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8777"

# 定義 tmux session 名稱
SESSION_NAME="ros2_workspace"

# 檢查是否已經在 tmux session 中
if [ -n "$TMUX" ]; then
    # 如果已在 tmux 中，直接切割當前視窗並執行指令
    tmux split-window -h "$CMD2"
    tmux send-keys "$CMD1" C-m
else
    # 如果不在 tmux 中，建立一個新的 session 並在第一個面板執行 CMD1
    tmux new-session -d -s $SESSION_NAME "$CMD1"

    # 左右切割視窗（-h 代表 horizontal split，在 tmux 中是左右切分）並執行 CMD2
    tmux split-window -h -t $SESSION_NAME "$CMD2"

    # 附加（attach）到該 session
    tmux attach-session -t $SESSION_NAME
fi
