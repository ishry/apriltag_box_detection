# apriltag_box_detection

AprilTagを貼ったboxの姿勢推定，画像・RViz表示，robot相対姿勢への変換を行うROS 1パッケージ．

# 環境

- Ubuntu 20.04．
- ROS Noetic．
- RViz．
- Python 3．

# 環境構築メモ

## 依存パッケージ導入

```bash
sudo apt install ros-noetic-usb-cam ros-noetic-camera-calibration ros-noetic-realsense2-camera python3-catkin-tools python3-pil python3-tk python3-yaml poppler-utils
```

## パッケージの導入

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

新しいterminalでもworkspaceを使用する場合は，以下を `~/.bashrc` に追加する．

```bash
source ~/catkin_ws/apriltag_ws/devel/setup.bash
```

# 使い方

## 1. 準備

### tag画像とdaeの生成

RViz表示に使うtag画像とdae meshを生成する．入力には，複数のAprilTagを含む画像またはPDFを指定できる．

```bash
cd ~/catkin_ws/apriltag_ws
python3 src/apriltag_box_detection/scripts/generate_tag_meshes.py \
  --source src/apriltag_box_detection/assets/apriltags.pdf
```

PDFのページと解像度を指定する場合：

```bash
python3 src/apriltag_box_detection/scripts/generate_tag_meshes.py \
  --source src/apriltag_box_detection/assets/apriltags.pdf \
  --page 1 \
  --dpi 300
```

起動したGUIで以下を行う．

1. tagの外枠をドラッグして選択する．
2. `Tag ID`を入力する．
3. `Add / Update`を押す．
4. 必要なtagについて繰り返す．
5. `Generate`を押す．

生成物：

```text
src/apriltag_box_detection/assets/tags/tag_0.png
src/apriltag_box_detection/assets/meshes/tag_0.dae
```

切り抜き情報は入力ファイルと同じdirectoryに保存される．

```text
src/apriltag_box_detection/assets/apriltags.page_1.crops.yaml
```

### tagと検出精度の設定

```text
src/apriltag_box_detection/config/settings.yaml
src/apriltag_box_detection/config/tags.yaml
```

- `settings.yaml`：tag family，thread数，decimateなどの検出設定．
- `tags.yaml`：検出するtag IDと実寸．

`tags.yaml`の`size`には，印刷余白を含めず，黒い正方形の外辺間の長さをm単位で指定する．tag sizeはこのファイルに集約し，box・robot設定にはIDと配置だけを書く．

### box設定

boxの寸法と，各tagの配置を`box.yaml`に記述する．

```text
src/apriltag_box_detection/config/box.yaml
```

設定例：

```yaml
frame_id: box_config

boxes:
  - id: box_0
    name: box_0
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

`boxes`には複数のboxを記述できる．`id`は購読側が識別に使う一意な値，`name`は表示用の名前である．boxごとにTag IDを割り当て，同じTag IDを複数boxに割り当てると設定エラーになる．

`face`番号：

```text
0: +X
1: -X
2: +Y
3: -Y
4: +Z
5: -Z
```

`offset`：

```text
u: 面内の横方向オフセット[m]
v: 面内の縦方向オフセット[m]
normal: 面から外側に浮かせる距離[m]
```

`rotation`：

```text
yaw: 面内回転[rad]
```

`assets/meshes/tag_<id>.dae`が存在する場合，RVizではそのmeshを使用する．存在しない場合は黒い板として表示する．

boxとtagの配置をRVizで確認する：

```bash
cd ~/catkin_ws/apriltag_ws
catkin build
source devel/setup.bash
roslaunch apriltag_box_detection box_config_viewer.launch
```

RVizを起動せずmarkerだけpublishする場合：

```bash
roslaunch apriltag_box_detection box_config_viewer.launch rviz:=false
```

### robot設定

robot原点から見たrobotタグの位置・姿勢を`robot.yaml`に記述する．

```text
src/apriltag_box_detection/config/robot.yaml
```

設定例：

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

`tags[].pose`には，robot原点から見たtag座標系の位置・姿勢を指定する．tagの実寸は`tags.yaml`に記述する．

robot本体は`MANTA_HR_TAIL.urdf`を`robot_description`として読み込み，RVizのRobotModelで表示する．別workspaceにある`package://manta_ros_bridge_tutorials/...`のmeshを解決するため，launch内で`manta_package_root`を`ROS_PACKAGE_PATH`へ追加している．環境全体からmanta workspaceを参照する場合は，起動前にsourceする．

```bash
source ~/catkin_ws/manta_ws/devel/setup.bash
source ~/catkin_ws/apriltag_ws/devel/setup.bash
```

robot modelとtag配置をRVizで確認する：

```bash
roslaunch apriltag_box_detection robot_config_viewer.launch
```

RVizを起動せずmarkerだけpublishする場合：

```bash
roslaunch apriltag_box_detection robot_config_viewer.launch rviz:=false
```

## 2. 実行

用途に応じて以下のlaunchを使う．

```text
box_detection.launch
├─ webcam_box_detection.launch
├─ realsense_box_detection.launch
└─ robot_relative_box_detection.launch
      └─ realsense_robot_relative_box_detection.launch
```

### 2.1 通常のbox検出

通常のbox検出では以下をpublishする．

```text
/tag_detections
/box_poses
/box_pose
/box_detection_markers
/box_detection_image
```

`/box_poses`は検出された全boxを含む`apriltag_box_detection/BoxPoseArray`をpublishする．各要素には`box_id`，`box_name`，使用した`source_tag_id`，姿勢が含まれる．`/box_pose`は既存の購読側向けに先頭のboxだけを`geometry_msgs/PoseStamped`でpublishする．複数boxを扱う場合は`/box_poses`を使う．

`/box_detection_image`を確認する場合は`rqt_image_view`を起動し，表示topicとして選択する．

```bash
rqt_image_view
```

#### 任意カメラ

`box_detection.launch`はカメラを起動しない．既にpublishされている画像topicとcamera_info topicを指定して，AprilTag検出とbox姿勢推定を起動する．

```bash
roslaunch apriltag_box_detection box_detection.launch \
  image_topic:=/usb_cam/image_raw \
  camera_info_topic:=/usb_cam/camera_info
```

RealSenseなど，別のカメラtopicを指定することもできる．

```bash
roslaunch apriltag_box_detection box_detection.launch \
  image_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info
```

#### USB webcam込み

`webcam_box_detection.launch`は`usb_cam`と`box_detection.launch`をまとめて起動する．

```bash
roslaunch apriltag_box_detection webcam_box_detection.launch
```

カメラ設定を変える例：

```bash
roslaunch apriltag_box_detection webcam_box_detection.launch \
  video_device:=/dev/video2 \
  image_width:=1280 \
  image_height:=720 \
  camera_info_url:=file:///home/leus/.ros/camera_info/usb_cam.yaml
```

#### RealSense込み

`realsense_box_detection.launch`はRealSenseと`box_detection.launch`をまとめて起動し，color画像をbox検出へ渡す．

```bash
sudo apt install ros-noetic-realsense2-camera
roslaunch apriltag_box_detection realsense_box_detection.launch
```

標準では以下のcolor streamを使う．

```text
/camera/color/image_raw
/camera/color/camera_info
```

RealSense単体で確認する場合：

```bash
roslaunch realsense2_camera rs_camera.launch
rqt_image_view
# /camera/color/image_raw を選ぶ
```

PCの画面が勝手に回転する場合は，`iio-sensor-proxy`を無効化する．

```bash
sudo systemctl disable iio-sensor-proxy
sudo systemctl stop iio-sensor-proxy
```

解像度やFPSを指定する例：

```bash
roslaunch apriltag_box_detection realsense_box_detection.launch \
  color_width:=640 \
  color_height:=480 \
  color_fps:=30
```

注意点：

- RealSenseの起動条件は，まず`roslaunch realsense2_camera rs_camera.launch`のデフォルト設定に合わせる．
- `color_width` / `color_height` / `color_fps`の固定，`enable_depth`，`enable_confidence`，`publish_tf`などをデフォルトから変えると，RealSense driver側のstream構成やprofile選択が変わって大きな遅延が出ることがある．
- カメラ単体で軽いのに検出込みで重い場合は，検出ノードより先にRealSenseの起動設定差を疑う．

複数台接続時などでcamera namespaceを変える場合：

```bash
roslaunch apriltag_box_detection realsense_box_detection.launch camera:=camera_1
```

この場合，RealSenseの画像frame名も`camera_1_color_optical_frame`のように変わる．
RVizにboxが出ない場合は，RVizのFixed Frameを画像topicの`header.frame_id`に合わせる．
標準の`camera:=camera`では`camera_color_optical_frame`を使う．

### 2.2 robot相対box検出

robotタグから`camera -> robot`，boxタグから`camera -> box`を求め，box姿勢を`robot_config`座標へ変換する．通常box検出のtopicに加えて以下をpublishする．

```text
/robot_relative_box_pose
/robot_relative_box_poses
/robot_relative_box_twist
/robot_relative_box_markers
```

`/robot_relative_box_poses`には，検出された全boxをrobot座標系へ変換した`apriltag_box_detection/BoxPoseArray`をpublishする．`/robot_relative_box_pose`と`/robot_relative_box_twist`は既存のRL購読側との互換性のため，先頭のboxを対象にpublishする．複数boxを扱う場合は`/robot_relative_box_poses`を使う．

robotタグが一度見えた後にロストした場合は，最後に見えていた`camera -> robot`を保持して使う．まだ一度もrobotタグが見えていない場合はpublishしない．
`/robot_relative_box_twist`は`geometry_msgs/TwistStamped`で，線速度と角速度を`robot_config`座標でpublishする．最初の観測と，観測間隔が`max_velocity_dt`を超えた直後は速度を0へ初期化する．通常時はrobot相対poseの差分から速度を計算し，`velocity_filter_alpha`の指数移動平均を適用する．

速度推定parameterのdefaultは以下のとおり．

```text
velocity_filter_alpha: 0.2
min_velocity_dt:       0.001 s
max_velocity_dt:       0.5 s
```

#### 任意カメラ

`robot_relative_box_detection.launch`はカメラを起動しない．既にpublishされている画像topicとcamera_info topicを指定する．

```bash
roslaunch apriltag_box_detection robot_relative_box_detection.launch \
  image_topic:=/usb_cam/image_raw \
  camera_info_topic:=/usb_cam/camera_info
```

別のカメラtopicを使う場合：

```bash
roslaunch apriltag_box_detection robot_relative_box_detection.launch \
  image_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info
```

#### RealSense込み

`realsense_robot_relative_box_detection.launch`はRealSense，通常box検出，robot相対変換，robot model，RVizをまとめて起動する．

```bash
roslaunch apriltag_box_detection realsense_robot_relative_box_detection.launch
```

### 2.3 設定確認

検出を行わず，設定したbox・robot・tagの位置関係だけをRVizで確認する．

```bash
# boxとtagの配置
roslaunch apriltag_box_detection box_config_viewer.launch

# robot modelとrobot tagの配置
roslaunch apriltag_box_detection robot_config_viewer.launch
```

### 2.4 設定ファイル差し替え

box設定やAprilTag設定を差し替える場合：

```bash
roslaunch apriltag_box_detection box_detection.launch \
  image_topic:=/usb_cam/image_raw \
  camera_info_topic:=/usb_cam/camera_info \
  box_config_file:=/path/to/box.yaml \
  tag_settings_file:=/path/to/settings.yaml \
  tag_config_file:=/path/to/tags.yaml
```

# 注意点メモ
- daeは`assets/meshes/tag_<id>.dae`という名前で探す．
- tag画像は`assets/tags/tag_<id>.png`に保存される．
- PDF読み込みには`pdftoppm`が必要．`poppler-utils`に含まれる．
- tagが小さくて選びにくい場合は，GUI windowを広げると画像表示も拡大される．
- `normal`を0にするとbox面とtag表示が重なって見づらいため，少しだけ浮かせる．
- box推定には画像topicだけでなく，キャリブレーション済みのcamera_info topicが必要．
- `/box_detection_image`はcamera_infoの内部パラメータで3D boxを画像へ投影している．
- 歪みが気になる場合は，raw画像よりrectified画像topicを使う．
- robot相対の場合，robot位置推定とbox位置推定を同時に行うと誤差が増える．robot位置のtagは，一度認識された後に隠す方がbox検出は安定する．
