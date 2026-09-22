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
	/// 初期姿勢からのずらし量。センサ座標系での平面運動。
	constexpr float shift_x = 0.10f;
	constexpr float shift_y = -0.06f;
	constexpr float shift_yaw = 0.03f; // [rad]

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

	// 初期姿勢を少しずらしたものを真値とする。
	const auto shift = SE3::rot(sotoba::math::quaternion::ypr(Vec3{0.f, 0.f, shift_yaw}))
		* SE3::trans(Vec3{shift_x, shift_y, 0.f});
	std::vector<SE3> truth{};
	truth.reserve(objects.size());
	for (const auto& object : objects) { truth.emplace_back(shift * object.initial_pose); }

	const auto points = simulate_scan(std::span{objects}, std::span{truth});
	if (points.size() < IcpEngine::min_correspondences()) {
		std::fprintf(stderr, "simulated scan has too few points: %zu\n", points.size());
		return 1;
	}
	std::printf("simulated scan: %zu points\n", points.size());

	IcpEngine engine{std::span<const ObjectDef>{objects}, points.size()};
	IcpParams params{};
	params.max_loop_num = 30;
	params.accept_distance = 0.2f;
	params.accept_distance_begin = 0.8f;
	params.tikhonov = {1.f, 1.f, 0.01f, 0.f, 0.f, 1.f};

	if (const auto status = engine.run(std::span{points}, params); status != RunStatus::ok) {
		std::fprintf(stderr, "run_icp failed: %s\n", to_string(status));
		return 1;
	}

	int failed = 0;
	for (std::size_t iobj = 0; iobj < objects.size(); ++iobj) {
		const auto estimated = engine.pose(iobj);
		const auto& expected = truth[iobj];
		const float dx = estimated.p.x() - expected.p.x();
		const float dy = estimated.p.y() - expected.p.y();
		const float dz = estimated.p.z() - expected.p.z();
		const float error = std::sqrt(dx * dx + dy * dy + dz * dz);

		const bool ok =
			engine.status(iobj) == ObjectStatus::updated && error <= position_tolerance;
		if (!ok) { ++failed; }

		std::printf(
			"[%s] object %zu '%s': status = %s, correspondences = %zu, position error = %.4f m\n",
			ok ? "ok" : "NG",
			iobj,
			objects[iobj].name.c_str(),
			to_string(engine.status(iobj)),
			engine.correspondence_count(iobj),
			static_cast<double>(error)
		);
	}

	return failed == 0 ? 0 : 1;
}
