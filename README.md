# apriltag_box_detection
AprilTagが貼られたboxの設定と表示確認をするROS1用パッケージ

# 環境
- Ubuntu 20.04
- ROS Noetic
- RViz
- Python3

# 環境構築メモ

## 依存パッケージ導入

```bash
sudo apt install ros-noetic-usb-cam ros-noetic-camera-calibration ros-noetic-realsense2-camera python3-catkin-tools python3-pil python3-tk python3-yaml poppler-utils
```

## apriltag / apriltag_ros / このパッケージの導入

```bash
mkdir -p ~/catkin_ws/apriltag_ws/src
cd ~/catkin_ws/apriltag_ws/src

git clone https://github.com/AprilRobotics/apriltag.git
git clone https://github.com/AprilRobotics/apriltag_ros.git
git clone https://github.com/ishry/apriltag_box_detection.git

cd ~/catkin_ws/apriltag_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

- 新しいterminalを開いたときに毎回使う場合は，以下を `~/.bashrc` に追加する。

```bash
source ~/catkin_ws/apriltag_ws/devel/setup.bash
```

# 使い方

## tag画像とdaeの生成
- 複数のAprilTagが入った画像またはPDFから，tagごとの画像とdaeを生成生成できる．

```bash
cd ~/catkin_ws/apriltag_ws
python3 src/apriltag_box_detection/scripts/generate_tag_meshes.py \
  --source src/apriltag_box_detection/assets/apriltags.pdf
```

PDFのページを指定する場合:

```bash
python3 src/apriltag_box_detection/scripts/generate_tag_meshes.py \
  --source src/apriltag_box_detection/assets/apriltags.pdf \
  --page 1 \
  --dpi 300
```

GUIでやること:
- tagの外枠をドラッグして矩形選択する
- 右側のTag IDに番号を入れる
- Add / Updateを押す
- 必要なtag分だけ繰り返す
- Generateを押す

生成されるファイル:

```text
src/apriltag_box_detection/assets/tags/tag_0.png
src/apriltag_box_detection/assets/meshes/tag_0.dae
```

切り抜き情報はsourceの横に保存される。

```text
src/apriltag_box_detection/assets/apriltags.page_1.crops.yaml
```

## box設定
boxとtagの位置関係は以下に書く。

```text
src/apriltag_box_detection/config/box.yaml
```

例:

```yaml
frame_id: box_config

boxes:
  - name: box_0
    size:
      x: 0.30
      y: 0.20
      z: 0.15

    tags:
      - id: 0
        face: 0
        offset:
          u: -0.03
          v: 0.02
          normal: 0.002
        rotation:
          yaw: 0.0
```

face番号:

```text
0: +X
1: -X
2: +Y
3: -Y
4: +Z
5: -Z
```

offsetの意味:

```text
u: 面内の横方向オフセット[m]
v: 面内の縦方向オフセット[m]
normal: 面から外側に浮かせる距離[m]
```

rotation:

```text
yaw: 面内回転[rad]
```

`box.yaml`ではtag idだけを書く。`assets/meshes/tag_0.dae` のようなファイルがあれば，RViz表示では自動でそのdaeを使う。なければ黒い板として表示する。

## robot設定表示
ロボット原点とロボットに貼ったtagの位置関係は以下に書く。

```text
src/apriltag_box_detection/config/robot.yaml
```

例:

```yaml
frame_id: robot_config

robot:
  name: manta

  tags:
    - id: 6
      pose:
        position:
          x: 0.0
          y: -0.4
          z: 0.1
        rotation:
          roll: 1.57
          pitch: 0.0
          yaw: 0.0
```

`tags[].pose` はロボット原点から見たtag座標系の位置姿勢を書く。tagの実寸は `tags.yaml` に書く。ロボット本体は `MANTA_HR_TAIL.urdf` を `robot_description` として読み込み，RVizのRobotModelで表示する。

別workspace内の `package://manta_ros_bridge_tutorials/...` meshを解決するため，launch内で `manta_package_root` を `ROS_PACKAGE_PATH` に追加している。環境全体でもmanta workspaceを見えるようにする場合は，起動前にsourceしておく。

```bash
source ~/catkin_ws/manta_ws/devel/setup.bash
source ~/catkin_ws/apriltag_ws/devel/setup.bash
```

表示確認:

```bash
roslaunch apriltag_box_detection robot_config_viewer.launch
```

RVizなしでmarkerだけ出す場合:

```bash
roslaunch apriltag_box_detection robot_config_viewer.launch rviz:=false
```

## robot相対box推定
ロボットtagが見えているときに `camera -> robot` を更新し，既存の `/box_pose` を `robot_config` 座標へ変換してpublishする。

```text
/robot_relative_box_pose
/robot_relative_box_markers
```

ロボットtagが一度見えた後にロストした場合は，最後に見えていた `camera -> robot` を保持して使う。まだ一度もロボットtagが見えていない場合はpublishしない。

```bash
roslaunch apriltag_box_detection robot_relative_box_detection.launch \
  image_topic:=/usb_cam/image_raw \
  camera_info_topic:=/usb_cam/camera_info
```

RealSenseなど別camera topicを使う場合:

```bash
roslaunch apriltag_box_detection robot_relative_box_detection.launch \
  image_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info
```

RealSense起動も含めて一括で動かす場合:

```bash
roslaunch apriltag_box_detection realsense_robot_relative_box_detection.launch
```

## AprilTag検出設定
AprilTag検出ノードに渡す設定はこのパッケージ内に置いている。

```text
src/apriltag_box_detection/config/settings.yaml
src/apriltag_box_detection/config/tags.yaml
```

`tags.yaml` には検出したいtag idと実寸を書く。今はbox用 `id 0-5` を `0.057m`，robot用 `id 6` を `0.142m` として登録している。boxやrobotの設定ではtag idと配置だけを書き，tag sizeはここに集約する。

## RVizで確認
```bash
cd ~/catkin_ws/apriltag_ws
catkin build
source devel/setup.bash
roslaunch apriltag_box_detection box_config_viewer.launch
```

表示されるもの:
- box本体
- face番号
- tag画像または黒い板
- tag id
- tag座標軸

RVizなしでmarkerだけ出す場合:

```bash
roslaunch apriltag_box_detection box_config_viewer.launch rviz:=false
```

## 任意カメラ + AprilTag + box推定
このlaunchはカメラを起動しない。既に出ている画像topicとcamera_info topicを使って，AprilTag検出とbox姿勢推定を起動する。

```bash
roslaunch apriltag_box_detection box_detection.launch \
  image_topic:=/usb_cam/image_raw \
  camera_info_topic:=/usb_cam/camera_info
```

RealSenseなど別カメラでも，画像topicとcamera_info topicを指定すればよい。

```bash
roslaunch apriltag_box_detection box_detection.launch \
  image_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info
```

出力topic:

```text
/tag_detections
/box_pose
/box_detection_markers
/box_detection_image
```

画像上のbox辺を確認する場合:

```bash
rqt_image_view
# /box_detection_image を選ぶ
```

## usb_cam込みで起動
USBカメラも一緒に起動したい場合は以下を使う。

```bash
roslaunch apriltag_box_detection webcam_box_detection.launch
```

カメラ設定を変える例:

```bash
roslaunch apriltag_box_detection webcam_box_detection.launch \
  video_device:=/dev/video2 \
  image_width:=1280 \
  image_height:=720 \
  camera_info_url:=file:///home/leus/.ros/camera_info/usb_cam.yaml
```

内部では `box_detection.launch` をincludeしている。

## RealSense込みで起動
`ros-noetic-realsense2-camera` を入れている場合は，RealSenseを起動してそのcolor画像をbox検出に渡せる。

```bash
sudo apt install ros-noetic-realsense2-camera
roslaunch apriltag_box_detection realsense_box_detection.launch
```

内部では `realsense2_camera` の `rs_camera.launch` と，このパッケージの `box_detection.launch` をincludeしている。
入力topicは標準のcolor streamを使う。

```text
/camera/color/image_raw
/camera/color/camera_info
```

RealSense単体で確認する場合:

```bash
roslaunch realsense2_camera rs_camera.launch
rqt_image_view
# /camera/color/image_raw を選ぶ
```

PCの画面が勝手に回転する場合は，`iio-sensor-proxy` を無効化する。

```bash
sudo systemctl disable iio-sensor-proxy
sudo systemctl stop iio-sensor-proxy
```

解像度やFPSを指定する例:

```bash
roslaunch apriltag_box_detection realsense_box_detection.launch \
  color_width:=640 \
  color_height:=480 \
  color_fps:=30
```

注意:
- RealSenseの起動条件は，まず `roslaunch realsense2_camera rs_camera.launch` のデフォルト設定に合わせる。
- `color_width` / `color_height` / `color_fps` の固定，`enable_depth`，`enable_confidence`，`publish_tf` などをデフォルトから変えると，RealSense driver側のstream構成やprofile選択が変わって大きな遅延が出ることがある。
- カメラ単体で軽いのに検出込みで重い場合は，検出ノードより先にRealSenseの起動設定差を疑う。

複数台接続時などでcamera namespaceを変える場合:

```bash
roslaunch apriltag_box_detection realsense_box_detection.launch camera:=camera_1
```

この場合，RealSenseの画像frame名も `camera_1_color_optical_frame` のように変わる。
RVizにboxが出ない場合は，RVizのFixed Frameを画像topicの `header.frame_id` に合わせる。
標準の `camera:=camera` では `camera_color_optical_frame` を使う。

## 設定ファイル差し替え
box設定やAprilTag設定を差し替える場合:

```bash
roslaunch apriltag_box_detection box_detection.launch \
  image_topic:=/usb_cam/image_raw \
  camera_info_topic:=/usb_cam/camera_info \
  box_config_file:=/path/to/box.yaml \
  tag_settings_file:=/path/to/settings.yaml \
  tag_config_file:=/path/to/tags.yaml
```

# 注意点メモ
- daeは `assets/meshes/tag_<id>.dae` という名前で探す
- tag画像は `assets/tags/tag_<id>.png` に保存される
- PDF読み込みには `pdftoppm` が必要。`poppler-utils` に入っている
- tagが小さくて選びにくい場合は，GUI windowを広げると画像表示も拡大される
- `normal` を0にするとbox面とtag表示が重なって見づらいので，少しだけ浮かせる
- box推定には画像topicだけでなく，キャリブ済みのcamera_info topicが必要
- `/box_detection_image` はcamera_infoの内部パラメータで3D boxを画像に投影している
- 歪みが気になる場合は，raw画像よりrectified画像topicを使う
