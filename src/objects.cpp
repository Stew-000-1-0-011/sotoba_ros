/// @file objects.cpp
/// 推定対象のオブジェクト定義。**ここを書き換えて使う**。
///
/// 現在の中身は千葉大ロボコン2026のフィールド
/// (YUKICHI6105/lio_localization_sim の config/robocon2026_field.json,
///  branch feature/eval-fidelity, schema_version 1) を写したもの。
/// 座標系はJSONと同じで、原点はフィールド中心の床面、+x がスタート->ビンゴ方向、
/// +y が左、+z が上。長さの単位は m。
///
/// initial_pose はフィールドローカル座標系をLiDAR座標系へ写すSE3
/// (= LiDARから見たフィールドの姿勢) で、ロボット姿勢の逆変換にあたる。
/// ICPは前回の推定値を次のシードにするので、起動時のだいたいの位置でよいが、
/// 大きく外すと収束しない。

#include "sotoba_ros/objects.hpp"

#include <array>
#include <cmath>
#include <utility>

namespace sotoba_ros {
	namespace {
		// --- robocon2026_field.json より ---

		/// 外周壁の芯の位置 [m] (JSON: walls.segments の start_end / bingo_end / *_side)
		constexpr float outer_wall_x = 2.8025f;
		constexpr float outer_wall_y = 1.6715f;

		/// ロボット姿勢 (フィールド座標系) -> ICPに渡すオブジェクト姿勢。
		///
		/// ICPが欲しいのは「フィールドローカル -> LiDAR座標系」なので、
		/// 「LiDAR -> フィールド」を作って逆を取る。
		auto robot_pose_to_object_pose(const ObjectsConfig& config) -> SE3 {
			const auto lidar_in_field =
				SE3::trans(Vec3{config.start_x, config.start_y, config.lidar_height})
				* SE3::rot(sotoba::math::quaternion::ypr(Vec3{0.f, 0.f, config.start_yaw}));
			return lidar_in_field.inv();
		}

		/// 壁1本 (JSONの walls.segments の1要素) を、厚みと高さを持つ直方体にする。
		///
		/// JSONの座標は壁の芯を通る線分なので、厚みは左右に半分ずつ、
		/// 高さは床 (z = 0) から上へ伸ばす。
		/// ロボットは壁の外側にいる (壁を外から見る) ので BoxOuter。
		auto wall_segment(
			const float x1,
			const float y1,
			const float x2,
			const float y2,
			const float thickness,
			const float height
		) -> Surface {
			const float dx = x2 - x1;
			const float dy = y2 - y1;
			const float length = std::sqrt(dx * dx + dy * dy);
			const float ux = dx / length;
			const float uy = dy / length;

			// rot は world -> local の回転で、行 i がローカル軸 i の world 表現。
			// ローカル x を壁に沿う向き、y を壁の法線向き、z を鉛直にとる。
			SquareMat<3> rot{};
			rot[0, 0] = ux;
			rot[0, 1] = uy;
			rot[0, 2] = 0.f;
			rot[1, 0] = -uy;
			rot[1, 1] = ux;
			rot[1, 2] = 0.f;
			rot[2, 0] = 0.f;
			rot[2, 1] = 0.f;
			rot[2, 2] = 1.f;

			return sotoba::surface::BoxOuter(
				Vec3{0.5f * (x1 + x2), 0.5f * (y1 + y2), 0.5f * height},
				rot,
				Vec3{0.5f * length, 0.5f * thickness, 0.5f * height},
				// 天板と底面 (ローカルz軸の±面) は無いことにする。
				// 走査面はこの箱のz範囲の内側を通るので側面しか当たらないはずだが、
				// 残しておくと端の点が天板に誤対応してz方向へ引っ張る。
				std::array<bool, 6>{false, false, false, false, true, true}
			);
		}

		/// ノーツ (JSON: rules.note_size, notes.blue / notes.orange)
		/// 注意: lidar_height >= note_size だとノーツには1本も当たらない
		/// (sotoba_node が起動時に警告する)。
		constexpr float note_size = 0.15f;

		/// ノーツ1個。床に置かれた立方体。
		///
		/// 引数はフィールド座標系での位置 (JSONの notes.* の座標そのまま)。
		/// initial_pose は他のオブジェクトと同じくLiDAR座標系なので、
		/// 初期ロボット姿勢を通して変換する。
		///
		/// 2スキャン目以降、ノーツをフィールドに追従させる (ロボットが動いても
		/// 見失わないようにする) のは事前予測の仕事で、
		/// field_note_predictor ノードがやる。
		auto make_note(
			const ObjectsConfig& config,
			const char* const name,
			const float x,
			const float y
		) -> ObjectDef {
			std::vector<Surface> surfaces{};
			surfaces.emplace_back(sotoba::surface::BoxOuter(
				Vec3{0.f, 0.f, 0.5f * note_size},
				SquareMat<3>::ide(),
				Vec3{0.5f * note_size, 0.5f * note_size, 0.5f * note_size},
				// 壁と同じく、走査面に効かない上面/底面は落とす
				std::array<bool, 6>{false, false, false, false, true, true}
			));

			return ObjectDef{
				.name = name,
				.surfaces = std::move(surfaces),
				.initial_pose = robot_pose_to_object_pose(config) * SE3::trans(Vec3{x, y, 0.f}),
			};
		}

		/// フィールド全体。壁もビンゴ棚も一緒に動く剛体なので1オブジェクトにまとめる。
		auto make_field(const ObjectsConfig& config) -> ObjectDef {
			const auto wall = [&](const float x1, const float y1, const float x2, const float y2) {
				return wall_segment(x1, y1, x2, y2, config.wall_thickness, config.wall_height);
			};

			std::vector<Surface> surfaces{};
			surfaces.reserve(16);

			// --- 外周壁 (start_end / bingo_end / left_side / right_side) ---
			// ロボットは内側にいるので、4枚まとめて BoxInner 1個で表す。
			// JSONの座標は芯なので、内側の面は半厚ぶん内側。
			surfaces.emplace_back(sotoba::surface::BoxInner(
				Vec3{0.f, 0.f, 0.5f * config.wall_height},
				SquareMat<3>::ide(),
				Vec3{
					outer_wall_x - 0.5f * config.wall_thickness,
					outer_wall_y - 0.5f * config.wall_thickness,
					0.5f * config.wall_height
				},
				// 天井と床は無い。2D LiDAR の点は走査面上にしか無いので、
				// 面外法線を持つ面を残すと誤対応でz方向に引っ張られるだけ。
				std::array<bool, 6>{false, false, false, false, true, true}
			));

			// --- 内側の壁 (JSON: walls.segments のうち外周以外) ---
			// センターライン
			surfaces.emplace_back(wall(-2.8025f, 0.0f, 1.5625f, 0.0f));
			// --- スラロームの壁 3列 ---
			// **JSONとは外壁/中央壁の付き方が反転している**。
			// 実フィールドを確認したところ、JSON (robocon2026_field.json) の
			// baffle_*_1 / baffle_*_2 / goal_slalom_boundary_* は
			// 互い違いの位相が逆だった。長さは各列のものを保ったまま、
			// 反対側の壁から生やしている。
			// JSONを直したら、こちらもJSONどおりに戻すこと。
			//
			// 列1 (JSON: baffle_*_1, x=-0.821, 長さ0.8715): 中央壁(y=0)から生える
			surfaces.emplace_back(wall(-0.821f, 0.019f, -0.821f, 0.8905f));
			surfaces.emplace_back(wall(-0.821f, -0.8905f, -0.821f, -0.019f));
			// ノーツ/スタートゾーンとスラロームの境界
			surfaces.emplace_back(wall(-1.579f, 1.069f, -1.579f, 1.669f));
			surfaces.emplace_back(wall(-1.579f, -1.669f, -1.579f, -1.069f));
			surfaces.emplace_back(wall(-1.579f, 0.0165f, -1.579f, 0.2665f));
			surfaces.emplace_back(wall(-1.579f, -0.2665f, -1.579f, -0.0165f));
			// 列2 (JSON: baffle_*_2, x=+0.021, 長さ0.8): 外壁(y=±1.6715)から生える
			surfaces.emplace_back(wall(0.021f, 0.8715f, 0.021f, 1.6715f));
			surfaces.emplace_back(wall(0.021f, -1.6715f, 0.021f, -0.8715f));
			// 列3 (JSON: goal_slalom_boundary_*, x=+0.821, 長さ0.8715): 中央壁から生える
			surfaces.emplace_back(wall(0.821f, 0.019f, 0.821f, 0.8905f));
			surfaces.emplace_back(wall(0.821f, -0.8905f, 0.821f, -0.019f));
			// ビンゴ棚裏のセンターライン
			surfaces.emplace_back(wall(2.5025f, 0.0f, 2.8025f, 0.0f));

			// --- ビンゴ棚 (JSON: bingo) ---
			// 実際は格子状で隙間だらけだが、走査面の高さでは正面が塞がっている前提で
			// 中実の箱として近似する。実機で合わないようならここを外すか、
			// 支柱を細い箱で並べる形に置き換えること。
			// centre = (2.0325, 0), width 0.94 (x方向), total_depth 0.6 (y方向), height 0.9
			surfaces.emplace_back(sotoba::surface::BoxOuter(
				Vec3{2.0325f, 0.f, 0.45f},
				SquareMat<3>::ide(),
				Vec3{0.47f, 0.30f, 0.45f},
				std::array<bool, 6>{false, false, false, false, true, true}
			));

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

		// ノーツ (JSON: notes.blue / notes.orange)。座標はフィールド座標系。
		//
		// 注意: 走査面 (lidar_height) はノーツの上端 (0.15m) すれすれなので、
		// 当たる点は1個あたり10数点しかない。さらに、
		// - 高さ0.3mのセンターライン壁の向こう側 (反対チーム側) のノーツは完全に隠れる
		// - 面が1つしか見えないノーツは、その面に沿う方向とyawが観測できない
		//   (正規方程式がランク落ちして solve_failed になり、publish されない)
		// ので、全部が常に取れるとは思わないこと。詳しくはREADMEを参照。
		objects.emplace_back(make_note(config, "note_orange_0", -2.699f, 0.125f));
		objects.emplace_back(make_note(config, "note_orange_1", -2.499f, 0.125f));
		objects.emplace_back(make_note(config, "note_orange_2", -2.299f, 0.125f));
		objects.emplace_back(make_note(config, "note_orange_3", -2.099f, 0.125f));
		objects.emplace_back(make_note(config, "note_orange_4", -1.899f, 0.125f));
		objects.emplace_back(make_note(config, "note_orange_5", -1.699f, 0.125f));
		objects.emplace_back(make_note(config, "note_blue_0", -2.699f, -0.125f));
		objects.emplace_back(make_note(config, "note_blue_1", -2.499f, -0.125f));
		objects.emplace_back(make_note(config, "note_blue_2", -2.299f, -0.125f));
		objects.emplace_back(make_note(config, "note_blue_3", -2.099f, -0.125f));
		objects.emplace_back(make_note(config, "note_blue_4", -1.899f, -0.125f));
		objects.emplace_back(make_note(config, "note_blue_5", -1.699f, -0.125f));

		return objects;
	}
} // namespace sotoba_ros
