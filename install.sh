#! /bin/bash

echo "╔══╣ Install: YOLO v8 ROS (STARTING) ╠══╗"

sudo apt-get update

pip3 install ultralytics

cd ~/catkin_ws/src/

git clone https://github.com/TeamSOBITS/bbox_to_tf.git


echo "╚══╣ Install: YOLO v8 ROS (FINISHED) ╠══╝"