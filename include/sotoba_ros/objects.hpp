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
		/// ICPの初期シード。
		///
		/// - parent が空のとき: オブジェクトローカル座標系をLiDAR座標系へ写すSE3
		///   (= LiDAR座標系におけるオブジェクトの姿勢)
		/// - parent があるとき: **親のローカル座標系での**姿勢
		///   (例: フィールド座標系で見たノーツの位置)
		SE3 initial_pose{SE3::ide()};
		/// 親オブジェクトの名前。空なら親なし。
		///
		/// 親を指定すると、毎スキャン「親の推定姿勢 × 親から見た自分の姿勢」を
		/// シードにしてからICPを走らせる。ロボットが動いても、親 (フィールド) の
		/// 推定が追えている限り子のシードはズレないので、
		/// ノーツのような小さいオブジェクトでも追い続けられる。
		///
		/// 親から見た相対姿勢は、推定が成功するたびに更新される
		/// (= オブジェクトが親の中で動いたことを追える)。
		/// 入れ子は1段まで。親自身に親がいる場合は警告して親なし扱いになる。
		std::string parent{};
	};

	/// 推定対象のオブジェクト群を作る。実装は src/objects.cpp。
	///
	/// 返り値の順序がそのままオブジェクトのインデックスになり、
	/// 姿勢はこの順で publish される。オブジェクト数は255個まで (sotoba側の制限)。
	auto make_objects() -> std::vector<ObjectDef>;
} // namespace sotoba_ros
