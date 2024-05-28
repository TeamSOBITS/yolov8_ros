<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
<!-- [![MIT License][license-shield]][license-url] -->

# YOLOv8 ROS

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#セットアップ">セットアップ</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#実行・操作方法">実行・操作方法</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#変更履歴">変更履歴</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>



<!-- レポジトリの概要 -->
## 概要

<!-- [![Product Name Screen Shot][product-screenshot]](https://example.com) -->
[![How to Train Ultralytics YOLOv8 models on Your Custom Dataset in Google Colab](https://img.youtube.com/vi/LNwODJXcvt4/0.jpg)](https://www.youtube.com/watch?v=LNwODJXcvt4)

YOLOv8のROS用パッケージ
sensor_msgs/Image型の画像データをSubscribeすることでYOLOv8による物体検出の推論を行います．

* yolov8 gihub: https://github.com/ultralytics/ultralytics
* yolov8Docs:https://docs.ultralytics.com/


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- セットアップ -->
## セットアップ

ここで，本レポジトリのセットアップ方法について説明してください．

### 環境条件
| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| Python | 3.7~ |
| Pytorch | 1.7~ |

### インストール方法

1. ROSの`src`フォルダに移動します．
   ```sh
   $ cd　~/catkin_ws/src/
   ```
2. 本レポジトリをcloneします．
   ```sh
   $ git clone https://github.com/TeamSOBITS/yolov8_ros.git
   ```
3. レポジトリの中へ移動します．
   ```sh
   $ cd yolov8_ros
   ```
4. 依存パッケージをインストールします．
    ```sh
    $ bash install.sh
    ```
5. パッケージをコンパイルします．
   ```sh
   $ cd ~/catkin_ws/
   $ catkin_make
   ```
<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## 実行・操作方法

<!-- デモの実行方法やスクリーンショットがあるとわかりやすくなるでしょう -->
YOLOv8
- ROSの画像のみで推論
```
roslaunch yolov8_ros yolov8.launch
```
> [!NOTE]
> launch内のweights，image_topic_nameをweightファイルや使用するカメラのTopic名に書き換える

- ROSの画像にTF(3次元情報)もつけて出力
```
roslaunch yolov8_ros yolov8_with_tf.launch
```
> [!NOTE]
> 画像のみでの推論と同様にlaunch内のweights，image_topic_nameをweightファイルや使用するカメラのTopic名に書き換える．
> また，3次元化するための点群名point_cloud_nameをカメラのTopic名に書き換える．
> base_frame_nameはもしカメラ単体で使う場合は存在するフレーム名にする．

### Published Topics
- ROSの画像のみで推論する場合
```
/yolov8/detect_list (sobits_msgs/StringArray): 検出物体一覧
/yolov8/detect_poses (sobits_msgs/ObjectPoseArray): 検出物体の画像におけるピクセル座標
/yolov8/objects_rect (sobits_msgs/BoundingBoxes): 検出物体のBoundingBox情報
/yolov8/detect_result (sensor_msgs/Image): 結果画像
```
- ROSの画像にTF(3次元情報)もつけて出力する場合
```
/yolov8_bbox_to_tf/object_poses (sobits_msgs/ObjectPoseArray): 物体の3次元座標
/yolov8_bbox_to_tf/object_cloud (sensor_msgs/PointCloud2): 物体にかかる点群
```

### Service Server
- ROSの画像のみで推論する場合
```
/yolov8/run_ctr (sobits_msgs/RunCtrl): 推論のON/OFFを切り替える(True/False)
```
- ROSの画像にTF(3次元情報)もつけて出力する場合
```
/yolov8_bbox_to_tf/run_ctr (sobits_msgs/RunCtrl): 3次元化(TF化)のON/OFFを切り替える(True/False)
```

### Scripts
```
detect_ros.py: YOLOv8実行用プログラム
pub_usb_cam.py: PC内蔵カメラ映像をPublishするプログラム
publish_image.py: Enterキーを押下ごとに画像をPublishするプログラム
train_yolov8.py: yolov8を用いて学習します　データセットを作って、5行目のdata引数にデータセットのパスを渡してください epoch=500, batch=4, imagesize=640で学習します
```

### sobits_msgsについて

sobits_msgsは，SOBITSの独自のメッセージ型です．\
検出結果やClass名も合わせたBoundingBoxなどがあります．\
sobits_msgsの詳細について，[TeamSOBTIS/sobits_msgs](https://github.com/TeamSOBITS/sobits_msgs) をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


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

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->



<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->



<!-- 参考文献 -->
## 参考文献

* [YOLOv8](https://github.com/ultralytics/ultralytics)
* [YOLOv8 Docs](https://docs.ultralytics.com/)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



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
