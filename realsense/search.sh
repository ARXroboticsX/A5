#!/bin/bash
workspace=$(pwd)

shell_type=${SHELL##*/}

shell_exec="exec $shell_type"

gnome-terminal --title="realsense" -- $shell_type -c "ros2 run realsense2_camera list_camera_node; $shell_exec"