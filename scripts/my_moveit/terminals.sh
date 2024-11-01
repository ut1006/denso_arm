#!/bin/bash

# 1. roscoreのタブを作成
gnome-terminal --tab --title="roscore" -- bash -c "roscore; exec bash"

sleep 8; 

# 2. ZEDのタブを作成
gnome-terminal --tab --title="ZED" -- bash -c "roslaunch zed_wrapper zedm.launch; exec bash"

# 3. DENSO_ARMのタブを作成
gnome-terminal --tab --title="DENSO_ARM" -- bash -c "roslaunch denso_robot_bringup vs060_bringup.launch sim:=false ip_address:=192.168.11.100; exec bash"

# 4. save_imageのタブを作成し、指定のディレクトリに移動してスクリプトを実行
gnome-terminal --tab --title="save_image" -- bash -c "cd ~/catkin_ws/src/denso_arm/scripts/my_moveit && python save_img_tf.py; exec bash"
