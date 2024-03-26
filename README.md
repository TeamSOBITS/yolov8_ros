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
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
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
/usb_cam/image_rawトピックにsensor_msgs/Image型の画像データを配信することでYOLOv8による推論を行います。

* yolov8 gihub: https://github.com/ultralytics/ultralytics
* yolov8Docs:https://docs.ultralytics.com/


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- セットアップ -->
## セットアップ

ここで，本レポジトリのセットアップ方法について説明してください．

### 環境条件
* Ubuntu: 20.04
* ROS: Noetic
* Python: >=3.7
* Pytorch: >=1.7

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
```
roslaunch yolov8_ros yolov8.launch
```


### Published Topics
```
detect_list (sobits_msgs/StringArray): 検出物体一覧
detect_poses (sobits_msgs/ObjectPoseArray): 検出物体位置
output_topic (sobits_msgs/BoundingBoxes): 検出物体のBBox情報 (xyxyn)
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
train_yolov8.py: yolov8を用いて学習します　データセットを作って、5行目のdata引数にデータセットのパスを渡してください epoch=500, batch=4, imagesize=640で学習します
```

### sobits_msgsについて
yolov8_rosはBoundingBoxの処理に独自のmsgとsrvを使用します．
1.  `BoundingBox.msg` : YOLOなどで得られた2次元情報をまとめたmsgです．
    ```yaml
    string  Class
    float64 probability
    int32   xmin
    int32   ymin
    int32   xmax
    int32   ymax
    ```

> [!WARNING]
> `BoundingBox.msg`は今後廃止状態（deprecated）になる予定です．

2.  `BoundingBoxes.msg` : 複数の`BoundingBox.msg`を配列にしたmsgです．
    ```yaml
    Header header
    BoundingBox[] bounding_boxes
    ```

> [!WARNING]
> `BoundingBoxes.msg`は今後廃止状態（deprecated）になる予定です．

3.  `ObjectPose.msg` : YOLOなどで得られた物体の3次元情報をまとめたmsgです．
    ```yaml
    string Class
    geometry_msgs/Pose pose
    int32 detect_id
    ```

> [!WARNING]
> `ObjectPose.msg`は今後廃止状態（deprecated）になる予定です．

4.  `ObjectPoseArray.msg` : 複数の`ObjectPose.msg`を配列にしたmsgです．
    ```yaml
    Header header
    ObjectPose[] object_poses
    ```

> [!WARNING]
> `ObjectPoseArray.msg`は今後廃止状態（deprecated）になる予定です．

5.  `StringArray.msg` : 複数の文字型情報を配列にしたmsgです．
    ```yaml
    Header header
    string[] data
    ```

6.  `RunCtrl.srv` : 起動・停止を指定するためのsrvです．
    ```yaml
    bool request
    ---
    bool response
    ```

sobits_msgsの詳細について，[TeamSOBTIS/sobits_msgs](https://github.com/TeamSOBITS/sobits_msgs) をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- マイルストーン -->
## マイルストーン

- [x] README.en.mdの執筆
- [ ] publish間隔を設定できる機能の実装(object_pose_publisherへのpublishを制御するために必要です．)

現時点のバッグや新規機能の依頼を確認するために[Issueページ](https://github.com/TeamSOBITS/yolov8_ros/issues) をご覧ください．

<p align="right">(<a href="#readme-top">上に</a>)</p>



<!-- 変更履歴 -->
## 変更履歴

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

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->



<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->



<!-- 参考文献 -->
## 参考文献

* [YOLOv8](https://github.com/ultralytics/ultralytics)
* [YOLOv8 Docs](https://docs.ultralytics.com/)
* []()

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
