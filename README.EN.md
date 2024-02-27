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
the package performs the inference with YOLOv8, and publishes the sensor_msgs/Image type message to the /usb_cam/image_raw topic. the published topic and some settings are changeable on launch file.

* yolov8 gihub: https://github.com/ultralytics/ultralytics
* yolov8Docs:https://docs.ultralytics.com/


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- セットアップ -->
## Getting Started
<!-- ここで，本レポジトリのセットアップ方法について説明してください． -->

### Requirements
* Ubuntu: 20.04
* ROS: Noetic
* Python: >=3.7
* Pytorch: >=1.7

### Installation

<!-- 1. pipよりultralyticsとrequirementsをインストール -->
1. Install "ultralytics" from pip.
```
pip install ultralytics
```

<!-- 2. TeamSOBITS/yolov8_rosのインストール -->
2. clone TeamSOBITS/yolov8_ros

```
cd ~/catkin_ws/src
git clone https://github.com/TeamSOBITS/yolov8_ros.git -b master
```

<!-- 3. TeamSOBITS/sobits_msgsをインストール -->
3. clone TeamSOBITS/objects_msgs
```
git clone https://github.com/TeamSOBITS/sobits_msgs.git
```
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- 実行・操作方法 -->
## Launch and Usage

<!-- デモの実行方法やスクリーンショットがあるとわかりやすくなるでしょう -->
YOLOv8
```
roslaunch yolov8_ros yolov8.launch
```


### Published Topics
```
detect_list (sobits_msgs/StringArray): a list of detected objects
detect_poses (sobits_msgs/ObjectPoseArray): the location of the objects
output_topic (sobits_msgs/BoundingBoxes): information of Bounding boxes of the objects (xyxyn)
detect_result (sensor_msgs/Image): the result image: TBD
```
### Subscribed Topics
```
/usb_cam/image_raw (sensor_msgs/Image): the input image to YOLOv8
```
### Scripts
```
detect_ros.py: YOLOv8 inference code

pub_usb_cam.py: the debug code (publishes PC Cam as Image type message)
publish_image.py: the debug code (publishes a image by press Enter)
train_yolov8.py: do the learning by YOLOv8: you must make a dataset, run the code with the path to your dataset. the learning is executed by epoch=500, batch=4, imagesize=640.
```

### sobits_msgs
<!-- yolov8_rosはBoundingBoxの処理に独自のmsgとsrvを使用します． -->
yolov8_ros requires TeamSOBITS's msg and srv.
1.  `BoundingBox.msg` : includes a bounding box information.
    ```yaml
    string  Class
    float64 probability
    int32   xmin
    int32   ymin
    int32   xmax
    int32   ymax
    ```

> [!WARNING]
<!-- > `BoundingBox.msg`は今後廃止状態（deprecated）になる予定です． -->
> `BoundingBox.msg` will be deprecated soon.

2.  `BoundingBoxes.msg` : includes some BoundingBox.msg as a array
    ```yaml
    Header header
    BoundingBox[] bounding_boxes
    ```

> [!WARNING]
> `BoundingBoxes.msg` will be deprecated soon.

3.  `ObjectPose.msg` : includes 3D information of a detected object.
    ```yaml
    string Class
    geometry_msgs/Pose pose
    int32 detect_id
    ```

> [!WARNING]
> `ObjectPose.msg` will be deprecated soon.

4.  `ObjectPoseArray.msg` : includes an array of ObjectPose.msg
    ```yaml
    Header header
    ObjectPose[] object_poses
    ```

> [!WARNING]
> `ObjectPoseArray.msg` will be deprecated soon.

5.  `StringArray.msg` : includes an array of String data.
    ```yaml
    Header header
    string[] data
    ```

6.  `RunCtrl.srv` : controls an execution of yolov8_ros inference.
    ```yaml
    bool request
    ---
    bool response
    ```

the detail of sobits_msgs available at [TeamSOBTIS/sobits_msgs](https://github.com/TeamSOBITS/sobits_msgs)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- マイルストーン -->
## Milestone

- [ ] write README.en.md: in progress
- [ ] make the rate of publishment controllable

Issues page available at [Issueページ](https://github.com/TeamSOBITS/yolov8_ros/issues)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- 変更履歴 -->
## Change-Log

<!-- - 2.0: 代表的なタイトル
  - 詳細 1
  - 詳細 2
  - 詳細 3
- 1.1: 代表的なタイトル
  - 詳細 1
  - 詳細 2
  - 詳細 3
- 1.0: 代表的なタイトル
  - 詳細 1
  - 詳細 2
  - 詳細 3 -->

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
* []()

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