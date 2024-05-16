#!/bin/bash

source install/local_setup.bash

cd ~/PX4-Autopilot/ 
x-terminal-emulator -e make px4_sitl gazebo PX4_SITL_WORLD=/home/giorgio/ws_sensor_combined/src/running_box/worlds/empty.world &
x-terminal-emulator -e MicroXRCEAgent udp4 -p 8888 &
ros2 launch running_box sim_with_drone.launch.py
