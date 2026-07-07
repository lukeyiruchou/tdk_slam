#!/bin/bash

# 定義要執行的指令（加入 cd 動作）
CMD1="cd .. && cd microros_ws/ && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6 --baudrate 1000000"
CMD2="ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8777"

# 定義 tmux session 名稱
SESSION_NAME="ros2_workspace"

# 檢查是否已經在 tmux session 中
if [ -n "$TMUX" ]; then
    # 1. 取得當前面板的 ID（避免切換時混亂）
    CURRENT_PANE=$(tmux display-message -p '#{pane_id}')
    
    # 2. 左右切分視窗，並取得新面板的 ID
    NEW_PANE=$(tmux split-window -h -P -F '#{pane_id}')
    
    # 3. 精準對左邊（原本）和右邊（新切）的面板發送指令
    tmux send-keys -t "$CURRENT_PANE" "$CMD1" C-m
    tmux send-keys -t "$NEW_PANE" "$CMD2" C-m
else
    # 如果不在 tmux 中，建立一個新的 session
    tmux new-session -d -s $SESSION_NAME
    
    # 左右切割視窗
    tmux split-window -h -t $SESSION_NAME
    
    # 分別向左邊（pane 0）和右邊（pane 1）發送指令
    tmux send-keys -t $SESSION_NAME:0.0 "$CMD1" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "$CMD2" C-m
    
    # 附加（attach）到該 session
    tmux attach-session -t $SESSION_NAME
fi