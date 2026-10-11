#!/usr/bin/env bash
xhost +local:root >/dev/null 2>&1
podman run --rm \
  --net=host \
  --ipc=host \
  -e DISPLAY="${DISPLAY}" \
  -e ROS_DOMAIN_ID=42 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/ricou/Projects/Internship_Project_1:/home/ricou/Projects/Internship_Project_1:Z \
  -w /home/ricou/Projects/Internship_Project_1 \
  docker.io/osrf/ros:humble-desktop \
  bash -c "
    apt-get update >/dev/null 2>&1 &&
    apt-get install -y ros-humble-joint-state-publisher-gui ros-humble-xacro python3-pip ros-humble-control-msgs ros-humble-moveit-msgs >/dev/null 2>&1 &&
    pip install python-fcl >/dev/null 2>&1 &&
    source /opt/ros/humble/setup.bash &&
    colcon build --packages-select arm_description arm_planning arm_bringup --symlink-install &&
    source install/setup.bash &&
    ros2 launch arm_bringup simulation.launch.py
  "
