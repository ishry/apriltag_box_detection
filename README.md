# apriltag_box_detection
AprilTagが貼られたboxの設定と表示確認をするROS1用パッケージ

# 環境
- Ubuntu 20.04
- ROS Noetic
- RViz
- Python3

# 依存パッケージ
```bash
sudo apt install python3-pil python3-tk python3-yaml poppler-utils
```

# 使い方

## tag画像とdaeの生成
複数のAprilTagが入った画像またはPDFから，tagごとの画像とdaeを生成する。

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
        size: 0.054
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

# 注意点メモ
- daeは `assets/meshes/tag_<id>.dae` という名前で探す
- tag画像は `assets/tags/tag_<id>.png` に保存される
- PDF読み込みには `pdftoppm` が必要。`poppler-utils` に入っている
- tagが小さくて選びにくい場合は，GUI windowを広げると画像表示も拡大される
- `normal` を0にするとbox面とtag表示が重なって見づらいので，少しだけ浮かせる
