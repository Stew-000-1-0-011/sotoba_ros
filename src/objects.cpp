/// @file objects.cpp
/// 推定対象のオブジェクト定義。**ここを書き換えて使う**。
///
/// 座標系はオブジェクトローカル。initial_pose がそのローカル座標系を
/// LiDAR座標系へ写すSE3 (= LiDAR から見たオブジェクトの初期姿勢) になる。
/// ICPは前回の推定値を次のシードにするので、initial_pose は
/// 「起動直後のだいたいの位置」で構わないが、外すと収束しない。

#include "sotoba_ros/objects.hpp"

#include <array>
#include <utility>

namespace sotoba_ros {
	namespace {
		/// 壁で囲まれたフィールド。LiDARは内側にいる前提なので BoxInner。
		/// hlens はハーフサイズ。
		auto make_field() -> ObjectDef {
			std::vector<Surface> surfaces{};
			surfaces.emplace_back(sotoba::surface::BoxInner(
				Vec3{0.f, 0.f, 0.f}, // center (フィールド中心)
				SquareMat<3>::ide(), // rot
				Vec3{6.0f, 6.0f, 1.0f}, // hlens: 12m x 12m、高さ2m
				// 天井と床 (z軸の±面) は無いことにする。
				// 2D LiDAR の点は全て z = 0 の平面上にあるので、面外法線を持つ面を残すと
				// 誤対応でz方向に引っ張られるだけで、得るものが無い。
				std::array<bool, 6>{false, false, false, false, true, true}
			));

			return ObjectDef{
				.name = "field",
				.surfaces = std::move(surfaces),
				// LiDARがフィールド中心から x=-3m の位置に、正面を向いて立っている想定。
				// (フィールドローカル -> LiDAR座標系 なので、ロボット姿勢の逆変換)
				.initial_pose = SE3::trans(Vec3{3.0f, 0.f, 0.f}),
			};
		}

		/// フィールド内に置かれた円柱のポール。単体で1オブジェクト扱いにすると
		/// フィールドとは独立に姿勢が推定される。
		auto make_pole() -> ObjectDef {
			std::vector<Surface> surfaces{};
			surfaces.emplace_back(sotoba::surface::CylinderOuter{
				.center = Vec3{0.f, 0.f, 0.f},
				.radius = 0.15f,
				.axis = UVec3{0.f, 0.f, 1.f},
				.hheight = 0.5f,
			});

			return ObjectDef{
				.name = "pole",
				.surfaces = std::move(surfaces),
				.initial_pose = SE3::trans(Vec3{2.0f, 1.0f, 0.f}),
			};
		}
	} // namespace

	auto make_objects() -> std::vector<ObjectDef> {
		std::vector<ObjectDef> objects{};
		objects.emplace_back(make_field());
		objects.emplace_back(make_pole());

		// 板を1枚足したいときの例:
		// std::vector<Surface> surfaces{};
		// surfaces.emplace_back(sotoba::surface::Rectangle(
		// 	Vec3{0.f, 0.f, 0.f},          // center
		// 	Vec4{1.f, 0.f, 0.f, 0.5f},    // u軸(単位ベクトル) + 半長
		// 	Vec4{0.f, 0.f, 1.f, 0.5f},    // v軸(単位ベクトル) + 半長
		// 	UVec3{0.f, -1.f, 0.f}         // 法線 (センサ側を向ける)
		// ));
		// objects.emplace_back(ObjectDef{
		// 	.name = "board",
		// 	.surfaces = std::move(surfaces),
		// 	.initial_pose = SE3::trans(Vec3{1.5f, 0.f, 0.f}),
		// });

		return objects;
	}
} // namespace sotoba_ros
