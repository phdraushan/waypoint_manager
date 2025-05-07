#!/bin/bash

# Create a new tmux session named 'anscer' (or attach to it if it exists)
tmux new-session -d -s anscer

# Split the window into 3 panes
tmux split-window -h
tmux split-window -v

# Select the first pane and run the first launch file
tmux select-pane -t 0
tmux send-keys "roslaunch start_anscer start_anscer.launch" C-m

# Select the second pane and run the second launch file
tmux select-pane -t 1
tmux send-keys "roslaunch anscer_navigation anscer_navigation.launch" C-m

# Select the third pane and run the third launch file
tmux select-pane -t 2
tmux send-keys "roslaunch waypoint_manager waypoint_manager.launch" C-m

# Attach to the tmux session
tmux attach-session -t anscer
