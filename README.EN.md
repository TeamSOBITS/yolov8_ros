<a name="readme-top"></a>

[JP](README.md) | [EN](README.EN.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
<!-- [![MIT License][license-shield]][license-url] -->

# レポジトリ名

<!-- 目次 -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#Introduction">Introduction</a>
    </li>
    <li>
      <a href="#Getting Started">Getting Started</a>
      <ul>
        <li><a href="#Requirements">Requirements</a></li>
        <li><a href="#Installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#Launch and Usage">Launch and Usage</a></li>
    <li><a href="#Milestone">Milestone</a></li>
    <li><a href="#Change-Log">Change-Log</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#Acknowledgements">Acknowledgements</a></li>
  </ol>
</details>



<!-- レポジトリの概要 -->
## Introduction

<!-- [![Product Name Screen Shot][product-screenshot]](https://example.com) -->
[![How to Train Ultralytics YOLOv8 models on Your Custom Dataset in Google Colab](https://img.youtube.com/vi/LNwODJXcvt4/0.jpg)](https://www.youtube.com/watch?v=LNwODJXcvt4)

<!-- YOLOv8のROS用パッケージ -->
The ROS package for YOLOv8
<!-- /usb_cam/image_rawトピックにsensor_msgs/Image型の画像データを配信することでYOLOv8による推論を行います。 -->
Subscribe to image data of sensor_msgs/Image type to infer object detection by YOLOv8.

* yolov8 gihub: https://github.com/ultralytics/ultralytics
* yolov8Docs:https://docs.ultralytics.com/


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- セットアップ -->
## Getting Started
<!-- ここで，本レポジトリのセットアップ方法について説明してください． -->

### Requirements
| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| Python | 3.7~ |
| Pytorch | 1.7~ |c

### Installation

1. Change directory
  ```sh
  $ cd　~/catkin_ws/src/
  ```
2. clone TeamSOBITS/yolov8_ros
  ```
  cd ~/catkin_ws/src
  git clone https://github.com/TeamSOBITS/yolov8_ros.git -b master
  ```
3. Change directory
  ```sh
  $ cd yolov8_ros
  ```
4. Install dependent packages
  ```sh
  $ bash install.sh
  ```
5. compile
   ```sh
   $ cd ~/catkin_ws/
   $ catkin_make
   ```
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- 実行・操作方法 -->
## Launch and Usage

<!-- デモの実行方法やスクリーンショットがあるとわかりやすくなるでしょう -->
YOLOv8
- only Image
```
roslaunch yolov8_ros yolov8.launch
```
> [!NOTE]
> Replace weights and image_topic_name in the launch" file with the topical name of the weight file or camera to be used.

- with TF
```
roslaunch yolov8_ros yolov8_with_tf.launch
```
> [!NOTE]
> And, the point cloud name point_cloud_name is rewritten to the Topic name of the camera to make it 3D.

### Published Topics
- only Image
```
/yolov8/detect_list (sobits_msgs/StringArray): a list of detected objects
/yolov8/detect_poses (sobits_msgs/ObjectPoseArray): the location of the objects in Image
/yolov8/objects_rect (sobits_msgs/BoundingBoxes): information of Bounding boxes of the objects
/yolov8/detect_result (sensor_msgs/Image): the result image
```
- with tf
```
/yolov8_bbox_to_tf/object_poses (sobits_msgs/ObjectPoseArray): the location of the objects in 3D
/yolov8_bbox_to_tf/object_cloud (sensor_msgs/PointCloud2): point cloud of object
```

### Service Server
- only Image
```
/yolov8/run_ctr (sobits_msgs/RunCtrl): Switching Inference(True/False)
```
- with tf
```
/yolov8_bbox_to_tf/run_ctr (sobits_msgs/RunCtrl): With 3D(to TF) (True/False)
```

### Scripts
```
detect_ros.py: YOLOv8 inference code

pub_usb_cam.py: the debug code (publishes PC Cam as Image type message)
publish_image.py: the debug code (publishes a image by press Enter)
train_yolov8.py: do the learning by YOLOv8: you must make a dataset, run the code with the path to your dataset. the learning is executed by epoch=500, batch=4, imagesize=640.
```

### sobits_msgs

sobits_msgs is a unique SOBITS message type.\
BoundingBox, etc., which also includes the detection result and Class name.\
the detail of sobits_msgs available at [TeamSOBTIS/sobits_msgs](https://github.com/TeamSOBITS/sobits_msgs)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTRIBUTING -->
<!-- ## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->



<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->



<!-- 参考文献 -->
## Acknowledgements

* [YOLOv8](https://github.com/ultralytics/ultralytics)
* [YOLOv8 Docs](https://docs.ultralytics.com/)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/yolov8_ros.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/yolov8_ros/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/yolov8_ros.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/yolov8_ros/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/yolov8_ros.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/yolov8_ros/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/yolov8_ros.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/yolov8_ros/issues
<!-- [license-shield]: https://img.shields.io/github/license/TeamSOBITS/yolov8_ros.svg?style=for-the-badge
[license-url]: https://github.com/TeamSOBITS/yolov8_ros/blob/master/LICENSE.txt -->