#!/bin/bash

# 创建一个新的tmux会话
tmux new-session -d -s mysession

tmux send-keys -t mysession:0.0 'source ../../../devel/setup.bash; roslaunch uav_control_ros px4_vicon.launch' C-m
sleep 0.5
tmux split-window -h -t mysession


tmux send-keys -t mysession:0.1 'source ../../../devel/setup.bash; roslaunch uav_control_ros vrpn_connect_single.launch' C-m
sleep 0.5
tmux split-window -v -t mysession:0.1


tmux send-keys -t mysession:0.2 'source ../../../devel/setup.bash' C-m
sleep 0.5
tmux split-window -v -t mysession:0.0


tmux send-keys -t mysession:0.3 'source ../../../devel/setup.bash' C-m

# 选择第一个窗格并附加会话
tmux select-pane -t mysession:0.0
tmux attach-session -t mysession
