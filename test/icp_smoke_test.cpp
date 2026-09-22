/// @file icp_smoke_test.cpp
/// ROSに依存しないスモークテスト。
///
/// objects.cpp のオブジェクト群を初期姿勢から少しずらした「真の姿勢」に置き、
/// 2D LiDAR を模したレイキャストで合成スキャンを作って、
/// 初期姿勢をシードにしたICPが真値へ戻るかを確認する。
///
/// オブジェクト定義には依存しないので、objects.cpp を書き換えてもそのまま使える。
/// 収束しなくなったら、それは形状かシードか (あるいは下のずらし量が
/// そのオブジェクトには大きすぎるか) の問題。

#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <span>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "sotoba_ros/icp_engine.hpp"
#include "sotoba_ros/objects.hpp"

namespace {
	using namespace sotoba_ros;
	using sotoba::math::UVec3;

	constexpr int ray_num = 720;
	/// 位置の許容誤差 [m]
	constexpr float position_tolerance = 0.03f;
	/// ロボットが動いたぶん (センサ座標系)。親を持たないオブジェクト全部に効く。
	constexpr float robot_shift_x = 0.10f;
	constexpr float robot_shift_y = -0.06f;
	constexpr float robot_shift_yaw = 0.03f; // [rad]
	/// 子オブジェクトが親の中で動いたぶん (自分の座標系)。
	constexpr float child_shift_x = 0.03f;
	constexpr float child_shift_y = 0.015f;
	constexpr float child_shift_yaw = 0.10f; // [rad]
	/// 何スキャンぶん回すか。子のシードは親の推定から作り直されるので、
	/// 親が収束した次のスキャンで子が合う。
	constexpr int scan_num = 3;

	/// ObjectDef::parent を index に直す (親なしは objects.size())。
	auto resolve_parents(std::span<const ObjectDef> objects) -> std::vector<std::size_t> {
		std::unordered_map<std::string, std::size_t> index_of{};
		for (std::size_t i = 0; i < objects.size(); ++i) { index_of.emplace(objects[i].name, i); }

		std::vector<std::size_t> parents(objects.size(), objects.size());
		for (std::size_t i = 0; i < objects.size(); ++i) {
			if (objects[i].parent.empty()) { continue; }
			if (const auto it = index_of.find(objects[i].parent); it != index_of.end()) {
				parents[i] = it->second;
			}
		}
		return parents;
	}

	/// 真の姿勢で全オブジェクトへレイキャストし、センサ座標系の点群を作る。
	/// 走査面はセンサ座標系の z = 0 平面。
	auto simulate_scan(std::span<const ObjectDef> objects, std::span<const SE3> truth)
		-> std::vector<Vec3> {
		std::vector<Vec3> points{};
		points.reserve(ray_num);

		for (int i = 0; i < ray_num; ++i) {
			const float angle = -std::numbers::pi_v<float>
				+ 2.f * std::numbers::pi_v<float> * static_cast<float>(i) / ray_num;
			const UVec3 ray{std::cos(angle), std::sin(angle), 0.f};

			float nearest2 = std::numeric_limits<float>::infinity();
			for (std::size_t iobj = 0; iobj < objects.size(); ++iobj) {
				for (const auto& surface : objects[iobj].surfaces) {
					std::visit(
						[&](auto moved) {
							moved.apply_se3(truth[iobj]);
							const float d2 = moved.ray_collision(ray);
							if (d2 < nearest2) { nearest2 = d2; }
						},
						surface
					);
				}
			}
			if (!std::isfinite(nearest2)) { continue; }

			const float d = std::sqrt(nearest2);
			points.emplace_back(d * ray.x(), d * ray.y(), 0.f);
		}

		return points;
	}
} // namespace

auto main() -> int {
	const auto objects = make_objects();
	if (objects.empty()) {
		std::fprintf(stderr, "make_objects() returned nothing.\n");
		return 1;
	}

	// 真値を作る。
	// 親なし: ロボットが動いたぶんをセンサ座標系で掛ける。
	// 子: 親の真値 × (親から見た初期姿勢 × 自分の座標系での移動)。
	const auto parents = resolve_parents(std::span{objects});
	const auto robot_shift =
		SE3::rot(sotoba::math::quaternion::ypr(Vec3{0.f, 0.f, robot_shift_yaw}))
		* SE3::trans(Vec3{robot_shift_x, robot_shift_y, 0.f});
	const auto child_shift = SE3::trans(Vec3{child_shift_x, child_shift_y, 0.f})
		* SE3::rot(sotoba::math::quaternion::ypr(Vec3{0.f, 0.f, child_shift_yaw}));

	std::vector<SE3> truth(objects.size(), SE3::ide());
	for (std::size_t iobj = 0; iobj < objects.size(); ++iobj) {
		const auto iparent = parents[iobj];
		truth[iobj] = iparent < objects.size()
			? truth[iparent] * objects[iobj].initial_pose * child_shift
			: robot_shift * objects[iobj].initial_pose;
	}

	const auto points = simulate_scan(std::span{objects}, std::span{truth});
	if (points.size() < IcpEngine::min_correspondences()) {
		std::fprintf(stderr, "simulated scan has too few points: %zu\n", points.size());
		return 1;
	}
	std::printf("simulated scan: %zu points\n", points.size());

	IcpEngine engine{std::span<const ObjectDef>{objects}, points.size()};
	for (const auto& warning : engine.warnings()) {
		std::fprintf(stderr, "warning: %s\n", warning.c_str());
	}

	IcpParams params{};
	params.max_loop_num = 30;
	// 小さいオブジェクトは緩いゲートだと隣の形状を掴んで飛んでいくので、
	// ゲートは絞る (スケジュールは使わない)。
	params.accept_distance = 0.1f;
	params.accept_distance_begin = 0.f;
	params.tikhonov = {1.f, 1.f, 0.01f, 0.f, 0.f, 1.f};

	// 同じスキャンを複数回流して、ノード上の連続スキャンを模す。
	for (int iscan = 0; iscan < scan_num; ++iscan) {
		if (const auto status = engine.run(std::span{points}, params); status != RunStatus::ok) {
			std::fprintf(stderr, "run_icp failed: %s\n", to_string(status));
			return 1;
		}
	}

	int failed = 0;
	int updated = 0;
	int invisible = 0;
	for (std::size_t iobj = 0; iobj < objects.size(); ++iobj) {
		const auto estimated = engine.pose(iobj);
		const auto& expected = truth[iobj];
		const float dx = estimated.p.x() - expected.p.x();
		const float dy = estimated.p.y() - expected.p.y();
		const float dz = estimated.p.z() - expected.p.z();
		const float error = std::sqrt(dx * dx + dy * dy + dz * dz);

		// 見えていないオブジェクト (陰に隠れている等) は評価しない。
		// publish されたもの (= updated) が正しいことと、
		// 十分な数が推定できていることを見る。
		const bool visible = engine.correspondence_count(iobj) > 0;
		const bool is_updated = engine.status(iobj) == ObjectStatus::updated;
		const char* tag = "--";
		if (is_updated) {
			++updated;
			if (error <= position_tolerance) {
				tag = "ok";
			} else {
				tag = "NG";
				++failed;
			}
		} else if (!visible) {
			++invisible;
		}

		std::printf(
			"[%s] object %zu '%s': status = %s, correspondences = %zu, position error = %.4f m\n",
			tag,
			iobj,
			objects[iobj].name.c_str(),
			to_string(engine.status(iobj)),
			engine.correspondence_count(iobj),
			static_cast<double>(error)
		);
	}

	std::printf(
		"updated %d / %zu object(s) (%d invisible), %d wrong\n",
		updated,
		objects.size(),
		invisible,
		failed
	);

	// publish される姿勢が全部正しく、かつ見えているものの過半が取れていること。
	const auto visible_num = objects.size() - static_cast<std::size_t>(invisible);
	if (failed != 0) { return 1; }
	if (static_cast<std::size_t>(updated) * 2 < visible_num) {
		std::fprintf(stderr, "too few objects were updated.\n");
		return 1;
	}
	return 0;
}
