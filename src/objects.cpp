/// @file objects.cpp
/// 推定対象のオブジェクト定義。
///
/// フィールドの形 (外周・内側の壁・ビンゴ棚・ノーツ) は ObjectsConfig (= パラメータの field.*) から作る。
/// 千葉大ロボコン2026のフィールドの値は oumuamua の src/oumuamua_bringup/config/field.yaml にある。
/// 座標系は、原点はフィールド中心の床面、+x がスタート->ビンゴ方向、+y が左、+z が上。長さの単位は m。
///
/// initial_pose はフィールドローカル座標系をLiDAR座標系へ写すSE3
/// (= LiDARから見たフィールドの姿勢) で、ロボット姿勢の逆変換にあたる。
/// ICPは前回の推定値を次のシードにするので、起動時のだいたいの位置でよいが、
/// 大きく外すと収束しない。

#include "sotoba_ros/objects.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>

namespace sotoba_ros {
	namespace {
		/// ロボット姿勢 (フィールド座標系) -> ICPに渡すオブジェクト姿勢。
		///
		/// ICPが欲しいのは「フィールドローカル -> LiDAR座標系」なので、
		/// 「LiDAR -> フィールド」を作って逆を取る。
		auto robot_pose_to_object_pose(const ObjectsConfig& config) -> SE3 {
			auto lidar_in_field =
				math::trans(Vec3{config.start_x, config.start_y, config.lidar_height})
				* math::rot(math::ypr(Vec3{0.f, 0.f, config.start_yaw}));

			// Z軸が床を向いている場合は、x軸まわりに180度回す。
			// x軸 (正面) はそのままで、y と z が反転する。
			if (config.lidar_upside_down) {
				lidar_in_field = lidar_in_field
					* math::rot(math::ypr(Vec3{std::numbers::pi_v<float>, 0.f, 0.f}));
			}

			return lidar_in_field.inverse();
		}

		/// ノーツ1個。床に置かれた一辺 config.note_size の立方体。
		/// 注意: lidar_height >= note_size だとノーツには1本も当たらない
		/// (sotoba_node が起動時に警告する)。
		///
		/// 引数はフィールド座標系での位置。
		/// initial_pose は他のオブジェクトと同じくLiDAR座標系なので、
		/// 初期ロボット姿勢を通して変換する。
		///
		/// 2スキャン目以降、ノーツをフィールドに追従させる (ロボットが動いても
		/// 見失わないようにする) のは、事前分布を出すノードの仕事。
		auto make_note(
			const ObjectsConfig& config,
			std::string name,
			const float x,
			const float y
		) -> ObjectDef {
			std::vector<Surface> surfaces{};
			surfaces.emplace_back(sotoba::surface::BoxOuter(
				Vec3{0.f, 0.f, 0.5f * config.note_size},
				SquareMat<3>::ide(),
				Vec3{0.5f * config.note_size, 0.5f * config.note_size, 0.5f * config.note_size},
				// 壁と同じく、走査面に効かない上面/底面は落とす
				std::array<bool, 6>{false, false, false, false, true, true}
			));

			return ObjectDef{
				.name = std::move(name),
				.surfaces = std::move(surfaces),
				.initial_pose = robot_pose_to_object_pose(config) * math::trans(Vec3{x, y, 0.f}),
			};
		}

		/// 床の上の箱 (ロボットは外にいる) を、高さ height の直方体にする。
		auto box_outer(const Box2& box, const float height) -> Surface {
			return sotoba::surface::BoxOuter(
				Vec3{0.5f * (box.xmin + box.xmax), 0.5f * (box.ymin + box.ymax), 0.5f * height},
				SquareMat<3>::ide(),
				Vec3{0.5f * (box.xmax - box.xmin), 0.5f * (box.ymax - box.ymin), 0.5f * height},
				// 天井と床は無い。2D LiDAR の点は走査面上にしか無いので、
				// 面外法線を持つ面を残すと誤対応でz方向に引っ張られるだけ。
				std::array<bool, 6>{false, false, false, false, true, true}
			);
		}

		/// フィールド全体。壁もビンゴ棚も一緒に動く剛体なので1オブジェクトにまとめる。
		/// 形はすべて config (パラメータの field.*) から作る。
		auto make_field(const ObjectsConfig& config) -> ObjectDef {
			std::vector<Surface> surfaces{};
			surfaces.reserve(config.walls.size() + 2);

			// --- 外周壁 ---
			// ロボットは内側にいるので、4枚まとめて BoxInner 1個で表す。
			const auto& b = config.bounds;
			surfaces.emplace_back(sotoba::surface::BoxInner(
				Vec3{0.5f * (b.xmin + b.xmax), 0.5f * (b.ymin + b.ymax), 0.5f * config.wall_height},
				SquareMat<3>::ide(),
				Vec3{0.5f * (b.xmax - b.xmin), 0.5f * (b.ymax - b.ymin), 0.5f * config.wall_height},
				std::array<bool, 6>{false, false, false, false, true, true}
			));

			// --- 内側の壁 ---
			for (const auto& wall : config.walls) {
				surfaces.emplace_back(box_outer(wall, config.wall_height));
			}

			// --- ビンゴ棚 ---
			// 実際は格子状で隙間だらけだが、走査面の高さでは正面が塞がっている前提で
			// 中実の箱として近似する。実機で合わないなら include_shelf: false にする。
			if (config.include_shelf) {
				surfaces.emplace_back(box_outer(config.shelf, config.shelf_height));
			}

			return ObjectDef{
				.name = "field",
				.surfaces = std::move(surfaces),
				.initial_pose = robot_pose_to_object_pose(config),
			};
		}
	} // namespace

	auto make_objects(const ObjectsConfig& config) -> std::vector<ObjectDef> {
		std::vector<ObjectDef> objects{};
		objects.emplace_back(make_field(config));

		if (!config.include_notes) { return objects; }

		// ノーツ。座標はフィールド座標系。+y 側 (notes_y) を orange、-y 側を blue と呼ぶ。
		//
		// 注意: 走査面 (lidar_height) はノーツの上端すれすれなので、
		// 当たる点は1個あたり10数点しかない。さらに、
		// - センターライン壁の向こう側 (反対チーム側) のノーツは完全に隠れる
		// - 面が1つしか見えないノーツは、その面に沿う方向とyawが観測できない
		//   (正規方程式がランク落ちして solve_failed になり、publish されない)
		// ので、全部が常に取れるとは思わないこと。詳しくはREADMEを参照。
		for (const auto& [prefix, y] : {std::pair{"note_orange_", config.notes_y}, std::pair{"note_blue_", -config.notes_y}}) {
			for (std::size_t i = 0; i < config.notes_x.size(); ++i) {
				objects.emplace_back(make_note(config, std::string{prefix} + std::to_string(i), config.notes_x[i], y));
			}
		}

		return objects;
	}
} // namespace sotoba_ros
