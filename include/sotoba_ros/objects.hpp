#pragma once

/// @file objects.hpp
/// ICPの推定対象となるオブジェクト群の定義。
///
/// ここには「型と宣言」だけを置き、実体 (どんな形状をどこに置くか) は
/// src/objects.cpp に書く。こうしておくと、形状を書き換えても再コンパイルされるのは
/// objects.cpp だけで済み、ノード本体やICPのテンプレート展開は再利用される。

#include <string>
#include <variant>
#include <vector>

#include <sotoba/math/se3.hpp>
#include <sotoba/surface/box.hpp>
#include <sotoba/surface/cylinder.hpp>
#include <sotoba/surface/rectangle.hpp>

namespace sotoba_ros {
	using sotoba::math::SE3;
	using sotoba::math::SquareMat;
	using sotoba::math::UVec3;
	using sotoba::math::Vec3;
	using sotoba::math::Vec4;

	/// オブジェクトを構成しうる曲面。
	///
	/// ここに曲面を足すと ICP のテンプレート展開 (icp_engine.cpp) が増える。
	/// 使わない曲面は消しておくとコンパイル時間と実行時の分岐が減る。
	using Surface = std::variant<
		sotoba::surface::BoxInner,
		sotoba::surface::BoxOuter,
		sotoba::surface::Rectangle,
		sotoba::surface::CylinderOuter>;

	/// 1つのオブジェクト = 剛体として一緒に動く曲面の集合。
	struct ObjectDef final {
		/// トピック名に使うのでASCIIかつ空白なしで。
		std::string name;
		/// オブジェクトローカル座標系での曲面群。
		std::vector<Surface> surfaces;
		/// ICPの初期シード。オブジェクトローカル座標系をLiDAR座標系へ写すSE3
		/// (= LiDAR座標系におけるオブジェクトの姿勢)。
		///
		/// 2スキャン目以降のシードは「事前分布の平均」になる。既定では
		/// sotoba_node 内蔵の持続予測 (姿勢そのまま + 不確かさ増加) が作るが、
		/// 外部ノードから ~/prior_beliefs で与えることもできる。
		/// オブジェクト間の連動 (例: ノーツはフィールドに付いて動く) のような
		/// アプリ固有の予測は外部ノードの仕事。
		SE3 initial_pose{SE3::ide()};
	};

	/// オブジェクトを組み立てるときの寸法・初期姿勢。
	///
	/// 実機に合わせて変える必要があるものを集めてある。
	/// 既定値は robocon2026_field.json のもの。
	/// sotoba_node / fake_scan_publisher はこれをROSパラメータから読む
	/// (再ビルドせずに変えられる)。
	struct ObjectsConfig final {
		/// LiDAR の取付高 [m]。走査面がフィールド床から何mにあるか。
		/// **実機に合わせて必ず直すこと**。
		/// これが壁の高さ以上だと壁が1本も見えないし、
		/// ノーツの1辺 (0.15m) 以上だとノーツも見えない。
		float lidar_height{0.14f};
		/// 壁の高さ [m]。JSONでは暫定的に 0.3 に上げてあるが、規定値は 0.1。
		float wall_height{0.3f};
		/// 壁の厚み [m] (JSON: walls.thickness)。
		float wall_thickness{0.033f};
		/// ロボットの初期位置 [m] と向き [rad] (フィールド座標系)。
		/// ICPの初期シードになるので、実際の置き場所と 0.1m 程度以内で合わせること。
		///
		/// 既定値は「棚から2つ目のスラローム壁 (x=+0.021) の延長線上、
		/// センターライン壁までの隙間 (y: 0.0165..0.8715) の中央」で、
		/// その壁のほうを正面 (+y) に向けた姿勢。
		/// -y 側の半面に置くなら start_y と start_yaw の符号を反転させる。
		/// JSONのミッション start_to_bingo_left は (-2.419, 1.354, yaw 0)。
		float start_x{0.021f};
		float start_y{0.444f};
		float start_yaw{1.5707963f}; // +y を向く (pi/2)
		/// LiDAR のZ軸が床を向いている (上下逆さに取り付けてある) か。
		///
		/// true なら初期姿勢に x軸まわり180度の回転が入る。
		/// ここを間違えると姿勢が180度ずれた状態から始まるので、ICPは収束しない。
		bool lidar_upside_down{true};
		/// ノーツを推定対象に含めるか。
		bool include_notes{true};
	};

	/// 推定対象のオブジェクト群を作る。実装は src/objects.cpp。
	///
	/// 返り値の順序がそのままオブジェクトのインデックスになり、
	/// 姿勢はこの順で publish される。オブジェクト数は255個まで (sotoba側の制限)。
	auto make_objects(const ObjectsConfig& config = {}) -> std::vector<ObjectDef>;
} // namespace sotoba_ros
