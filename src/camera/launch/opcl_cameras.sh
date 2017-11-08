#!/bin/bash
cd ~/ros_workspace/final_project
rosrun baxter_tools camera_control.py -c right_hand_camera
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
rosrun baxter_tools camera_control.py -o head_camera -r 640x400
