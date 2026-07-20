#!/usr/bin/env bash

cd "$HOME/Riccardo/franka_ws_013" || return 1

source /opt/ros/noetic/setup.bash
source install/setup.bash

export FRANKA_013_PREFIX="$HOME/Riccardo/libfranka-0.13.3/install"
export LD_LIBRARY_PATH="$PWD/install/lib:$FRANKA_013_PREFIX/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
