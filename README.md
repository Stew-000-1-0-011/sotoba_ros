# sotoba_ros

[sotoba](https://github.com/Stew-000-1-0-011/sotoba) の点対面ICPを使い、
`/scan` (`sensor_msgs/msg/LaserScan`) から既知オブジェクト群の
**LiDAR座標系での姿勢**を推定して `Pose` を publish する ROS 2 パッケージ。

対応環境: ROS 2 Lyrical Luth / Ubuntu 26.04。
sotoba が deducing this (P0847) と多次元 `operator[]` (P2128) を使うため C++23 必須
(GCC 14+ / Clang 18+)。

## 構成

| ファイル | 役割 |
| --- | --- |
| `include/sotoba_ros/objects.hpp` | オブジェクトの型 (`Surface`, `ObjectDef`) と `make_objects()` の宣言 |
| `src/objects.cpp` | **オブジェクトの実体。ここを書き換えて使う** |
| `include/sotoba_ros/icp_engine.hpp` | ICPラッパのインタフェース。sotobaのICPヘッダは出てこない (pimpl) |
| `src/icp_engine.cpp` | sotobaのICPテンプレートを実体化する唯一のTU |
| `include/sotoba_ros/sotoba_node.hpp` | ノード `SotobaNode` の宣言。オブジェクト群はコンストラクタで受け取る |
| `src/sotoba_node_impl.cpp` | ノードの実装 (ROSの入出力とパラメータ) |
| `src/sotoba_node.cpp` | `main()`。`make_objects()` の結果をノードへ渡す |
| `test/icp_smoke_test.cpp` | 合成スキャンでICPの収束を確認するスモークテスト (ROS不要) |

### コンパイルの分割

重いのは sotoba の ICP テンプレート (曲面の型リストで実体化される) なので、
その実体化を `src/icp_engine.cpp` の1TUに閉じ込めている。
`icp_engine.hpp` は `sotoba/icp_resource/*` をインクルードしない。

結果として、

- 形状を変えた → 再コンパイルは `objects.cpp` だけ
- ノードのROS周りを変えた → 再コンパイルは `sotoba_node_impl.cpp` だけ
- `objects.hpp` の `Surface` に曲面を足した → ICPの実体化 (`icp_engine.cpp`) が再コンパイル

という切り分けになる。

## ビルド

sotoba は rosdep に無いので、先に install しておく。

```bash
git clone https://github.com/Stew-000-1-0-011/sotoba.git
cmake -S sotoba -B sotoba/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/.local -DBUILD_DEVS=OFF \
  -DBUILD_STATIC_LIB=OFF -DBUILD_SHARED_LIB=OFF -DBUILD_MODULE_LIB=OFF
cmake --build sotoba/build --target install
```

その上でワークスペースをビルドする。

```bash
colcon build --packages-select sotoba_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$HOME/.local
```

sotoba を自前で install したくなければ、CMake側で取ってこさせてもよい。

```bash
colcon build --packages-select sotoba_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DSOTOBA_ROS_FETCH_SOTOBA=ON
```

## 実行

```bash
ros2 launch sotoba_ros sotoba_node.launch.py
# or
ros2 run sotoba_ros sotoba_node --ros-args --params-file config/sotoba_node.yaml
```

### トピック

| 方向 | トピック | 型 |
| --- | --- | --- |
| sub | `scan_topic` パラメータ (既定 `/scan`) | `sensor_msgs/msg/LaserScan` |
| pub | `~/objects/<name>/pose` | `geometry_msgs/msg/PoseStamped` |
| pub | `~/object_poses` | `geometry_msgs/msg/PoseArray` |

publish される姿勢は「オブジェクトローカル座標系をLiDAR座標系へ写す SE3」、
すなわち **LiDARから見たオブジェクトの位置姿勢**。
`frame_id` と `stamp` は入力 `LaserScan` のものをそのまま使う。

更新に失敗したオブジェクト (対応点不足・解けなかった) は publish されず、警告が出る。

## テスト

ROSに依存しないスモークテストを1つ入れてある
(`test/icp_smoke_test.cpp`)。`objects.cpp` のオブジェクトを真値の姿勢に置き、
2D LiDAR を模したレイキャストで合成スキャンを作り、初期姿勢をシードにしたICPが
真値へ戻るかを見る。

```bash
colcon test --packages-select sotoba_ros
colcon test-result --verbose
```

`objects.cpp` を書き換えたら、このテストの `truth` も合わせて直すこと。

## パラメータ

`config/sotoba_node.yaml` に全部コメント付きで並べてある。要点だけ:

- `tikhonov` (6要素, 回転3 + 並進3): **2D LiDARでは roll / pitch / z が観測できない**ので、
  該当成分 (添字 0, 1, 5) には正の値を入れておくこと。既定は `[1, 1, 0.01, 0, 0, 1]`。
  yaw (添字2) の僅かな値は、円柱のような回転対称オブジェクトで正規方程式がランク落ちして
  `solve_failed` になるのを防ぐためのもの。
- `accept_distance`: 対応点として採用する距離 [m]。大きすぎると誤対応、小さすぎると収束しない。
  初期ずれが大きいときは `accept_distance_begin` を大きめにして等比で絞ると入りやすい。
- `max_points` / `point_stride`: 計算量は点数に比例する。UST-10LXなら間引かなくても足りるはず。
- `sigma_range` / `sigma_angle` / `huber_k`: 外れ値が多いときに効かせる。0で無効。
- `reset_after_failures`: 連続失敗が続いたら `initial_pose` に戻す。0で無効。

## オブジェクトの書き方

`src/objects.cpp` の `make_objects()` が返す `ObjectDef` の並びがそのまま推定対象になる。

```cpp
ObjectDef{
  .name = "field",                 // トピック名に使われる
  .surfaces = std::move(surfaces), // オブジェクトローカル座標系での曲面群
  .initial_pose = SE3::trans(Vec3{3.f, 0.f, 0.f}), // ICPの初期シード
}
```

- 1つの `ObjectDef` に入れた曲面は**剛体として一緒に動く**。別々に動くものは別オブジェクトにする。
- 使える曲面は `objects.hpp` の `Surface` variant に並べたもの
  (`BoxInner`, `BoxOuter`, `Rectangle`, `CylinderOuter`)。使わないものは消してよい。
- `BoxInner` はセンサが内側にいる前提の直方体で、囲い壁1個ぶんとして使える。
- ICPは前回の推定値を次のシードにするので、`initial_pose` は起動直後のだいたいの位置でよい。
  ただし大きく外すと収束しない。
- オブジェクト数は255個、曲面数は255個まで (sotoba側の制限)。

## 既知の注意点

- 2D LiDAR の1スキャンは平面上の点しか無いので、面外自由度 (z, roll, pitch) は
  原理的に決まらない。`tikhonov` で拘束するか、そもそも面外に効く形状を置かないこと。
- sotoba の README/メモにもある通り、景色の対称性が高いとICPの解は飛ぶことがある。
  後段で異常値処理をすること (本ノードは `reset_after_failures` 程度しか面倒を見ない)。
- 円柱のように小さいオブジェクトは、シードがずれて真の点がシード形状の裏側へ回ると
  可視判定で対応点が全部消える。小さいオブジェクトほどシードを正確に。
- 点群容量は最初のスキャンで確定し、それを超えるスキャンが来たらエンジンを作り直す
  (その際は推定済み姿勢を引き継ぐ)。恒常的に作り直しが起きるなら `max_points` を見直すこと。
