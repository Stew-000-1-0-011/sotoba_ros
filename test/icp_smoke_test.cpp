/// @file icp_smoke_test.cpp
/// ROSに依存しないスモークテスト。
///
/// objects.cpp のオブジェクト群を真値の姿勢に置き、2D LiDAR を模したレイキャストで
/// 合成スキャンを作り、初期姿勢をシードにしたICPが真値へ戻るかを確認する。
/// オブジェクト定義を書き換えたら、このテストの truth もそれに合わせること。

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
	constexpr float position_tolerance = 0.05f;

	/// 真の姿勢で全オブジェクトへレイキャストし、センサ座標系の点群を作る。
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
	if (objects.size() < 2) {
		std::fprintf(stderr, "this test assumes the default objects.cpp (field + pole).\n");
		return 1;
	}

	// 初期姿勢からずらした真値。
	// field: 並進 (0.4, -0.2) + yaw 5deg、pole: 横に 0.08m。
	// pole を視線方向へずらさないのは、円柱の可視判定で対応点が消えるのを避けるため。
	std::vector<SE3> truth{};
	truth.emplace_back(
		SE3::rot(sotoba::math::quaternion::ypr(Vec3{0.f, 0.f, 0.087f}))
		* SE3::trans(Vec3{0.4f, -0.2f, 0.f}) * objects[0].initial_pose
	);
	truth.emplace_back(SE3::trans(Vec3{0.f, 0.08f, 0.f}) * objects[1].initial_pose);
	for (std::size_t iobj = 2; iobj < objects.size(); ++iobj) {
		truth.emplace_back(objects[iobj].initial_pose);
	}

	const auto points = simulate_scan(std::span{objects}, std::span{truth});
	if (points.size() < IcpEngine::min_correspondences()) {
		std::fprintf(stderr, "simulated scan has too few points: %zu\n", points.size());
		return 1;
	}

	IcpEngine engine{std::span<const ObjectDef>{objects}, points.size()};
	IcpParams params{};
	params.max_loop_num = 30;
	params.accept_distance = 0.5f;
	params.accept_distance_begin = 1.5f;
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
