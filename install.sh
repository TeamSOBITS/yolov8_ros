#! /bin/bash

echo "╔══╣ Install: YOLO v8 ROS (STARTING) ╠══╗"

sudo apt-get update

pip3 install ultralytics

cd ~/colcon_ws/src/

git clone -b feature/humble-devel https://github.com/TeamSOBITS/bbox_to_tf.git


echo "╚══╣ Install: YOLO v8 ROS (FINISHED) ╠══╝"