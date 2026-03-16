#!/bin/bash

set -e

SESSION_NAME="sunray_exp_control"

tmux kill-session -t "${SESSION_NAME}" 2>/dev/null || true

tmux new-session -d -s "${SESSION_NAME}" -n control \
  'bash -ic "roscore; exec bash"'

tmux split-window -v -t "${SESSION_NAME}:control" \
  'bash -ic "sleep 2.0; roslaunch sunray_uav_control sunray_mavros_exp.launch; exec bash"'

tmux split-window -h -t "${SESSION_NAME}:control.0" \
  'bash -ic "sleep 2.0; roslaunch sunray_uav_control sunray_vrpn.launch; exec bash"'

tmux split-window -h -t "${SESSION_NAME}:control.0" \
  'bash -ic "sleep 2.0; roslaunch sunray_mocap mocap_odom.launch; exec bash"'

tmux split-window -h -t "${SESSION_NAME}:control.1" \
  'bash -ic "sleep 2.0; roslaunch uav_control sunray_fsm.launch; exec bash"'

tmux select-layout -t "${SESSION_NAME}:control" tiled
tmux select-pane -t "${SESSION_NAME}:control.0"

if [ -n "${TMUX}" ]; then
  tmux switch-client -t "${SESSION_NAME}"
else
  tmux attach-session -t "${SESSION_NAME}"
fi
