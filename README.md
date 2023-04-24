# yolov8_ros
YOLOv8のROS用パッケージ
/usb_cam/image_rawトピックにsensor_msgs/Image型の画像データを配信することでYOLOv8による推論を行います。

* yolov8 gihub: https://github.com/ultralytics/ultralytics
* yolov8Docs:https://docs.ultralytics.com/

## Requirements
* Ubuntu: 20.04
* ROS: Noetic
* Python: >=3.7
* Pytorch: >=1.7

## Installation
pipよりultralyticsとrequirementsをインストール
```
pip install ultralytics
```

Team/SOBITS/yolov8_rosのインストール

```
cd ~/catkin_ws/src
git clone https://github.com/TeamSOBITS/yolov8_ros.git master
```

## Usage
YOLOv8
```
roslaunch yolov8_ros yolov8.launch
```

## Notes
### Published Topics
```
detect_list (sobit_common_msg/StringArray): 検出物体一覧
detect_poses (sobit_common_msg/ObjectPoseArray): 検出物体位置
output_topic (sobit_common_msg/BoundingBoxes): 検出物体のBBox情報 (xyxyn)
detect_result (sensor_msgs/Image): 結果画像　to be developed
```
### Subscribed Topics
```
/usb_cam/image_raw (sensor_msgs/Image): YOLOv8の入力画像
```
### Scripts
```
detect_ros.py: YOLOv8実行用プログラム
pub_usb_cam.py: PC内蔵カメラ映像をPublishするプログラム
publish_image.py: Enterキーを押下ごとに画像をPublishするプログラム
```
