#!/bin/bash

# 定義要執行的指令
CMD1="ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6 --baudrate 1000000"
CMD2="ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8777"

# 定義 tmux session 名稱
SESSION_NAME="ros2_workspace"

# 檢查是否已經在 tmux session 中
if [ -n "$TMUX" ]; then
    # 如果已在 tmux 中，先左右切分視窗
    tmux split-window -h
    # 對左邊（當前）面板發送 CMD1 並執行
    tmux send-keys "$CMD1" C-m
    # 對右邊（剛切出來）面板發送 CMD2 並執行
    tmux select-pane -t :.+
    tmux send-keys "$CMD2" C-m
else
    # 如果不在 tmux 中，建立一個新的 session（預設會停留在 bash 提示字元）
    tmux new-session -d -s $SESSION_NAME
    
    # 左右切割視窗（此時會有兩個乾淨的 bash 面版）
    tmux split-window -h -t $SESSION_NAME
    
    # 分別向左邊（pane 0）和右邊（pane 1）發送指令
    tmux send-keys -t $SESSION_NAME:0.0 "$CMD1" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "$CMD2" C-m
    
    # 附加（attach）到該 session
    tmux attach-session -t $SESSION_NAME
fi