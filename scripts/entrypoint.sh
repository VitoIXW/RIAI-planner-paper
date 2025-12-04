#!/bin/bash
set -e

source /ros2_ws/install/setup.bash && \
ros2 launch riai_launch riai.launch.py num_vehicles:=2