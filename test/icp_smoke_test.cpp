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

#include <Eigen/Dense>
#include <limits>
#include <numbers>
#include <span>
#include <variant>
#include <vector>

#include "sotoba_ros/belief.hpp"
#include "sotoba_ros/icp_engine.hpp"
#include "sotoba_ros/objects.hpp"

namespace {
	using namespace sotoba_ros;
	using sotoba::math::UVec3;

	constexpr int ray_num = 720;
	/// 位置の許容誤差 [m]。事前が支配的なオブジェクト (観測できない方向がある) は
	/// この値を超えうるので、下の NEES でも見る。
	constexpr float position_tolerance = 0.03f;
	/// 正規化推定誤差二乗 (並進3自由度) の上限。
	/// 推定が自分の申告する共分散と整合しているかを見る。chi^2(3) の 99% は 11.3。
	constexpr float nees_limit = 11.3f;
	/// ロボットが動いたぶん (センサ座標系)。全オブジェクトが同じだけずれる。
	///
	/// ここは**小さめ**にしてある。大きく動かすと、ノーツのような小さい
	/// オブジェクトはシードが自分の影から外れて対応点を失うが、それは
	/// 事前予測 (field_note_predictor) の担当であって、ICP単体の問題ではない。
	constexpr float robot_shift_x = 0.02f;
	constexpr float robot_shift_y = -0.012f;
	constexpr float robot_shift_yaw = 0.006f; // [rad]
	/// 何スキャンぶん回すか。
	constexpr int scan_num = 2;

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

	// 真値を作る。ロボットが動いたぶんをセンサ座標系で掛ける
	// (静止した世界をロボットが見ているので、全オブジェクトに同じ変換がかかる)。
	const auto robot_shift = math::rot(math::ypr(Vec3{0.f, 0.f, robot_shift_yaw}))
		* math::trans(Vec3{robot_shift_x, robot_shift_y, 0.f});

	std::vector<SE3> truth{};
	truth.reserve(objects.size());
	for (const auto& object : objects) {
		truth.emplace_back(robot_shift * object.initial_pose);
	}

	const auto points = simulate_scan(std::span{objects}, std::span{truth});
	if (points.size() < IcpEngine::min_correspondences()) {
		std::fprintf(stderr, "simulated scan has too few points: %zu\n", points.size());
		return 1;
	}
	std::printf("simulated scan: %zu points\n", points.size());

	IcpEngine engine{std::span<const ObjectDef>{objects}, points.size()};

	IcpParams params{};
	params.max_loop_num = 30;
	// 小さいオブジェクトは緩いゲートだと隣の形状を掴んで飛んでいくので、
	// ゲートは絞る (スケジュールは使わない)。
	params.accept_distance = 0.1f;
	params.accept_distance_begin = 0.f;
	// (並進, 回転) の順。2D LiDAR で見えない z / roll / pitch を抑える
	params.tikhonov = {0.f, 0.f, 1.f, 1.f, 1.f, 0.01f};
	// 事前を使うので sotoba 側が noise を要求する
	params.noise = NoiseParams{0.03f, 0.005f};

	// 事前分布: 初期姿勢まわり。面外 (z, roll, pitch) は取付で決まっていて動かない。
	std::vector<Belief> priors{};
	priors.reserve(objects.size());
	const auto prior_information =
		diagonal_information({0.05f, 0.05f, 0.005f, 0.005f, 0.005f, 0.05f});
	for (const auto& object : objects) {
		priors.emplace_back(Belief{object.initial_pose, prior_information});
	}

	// 同じスキャンを複数回流して、ノード上の連続スキャンを模す。
	for (int iscan = 0; iscan < scan_num; ++iscan) {
		// 事前の平均は前回の推定に合わせて動かす (持続予測に相当)
		for (std::size_t iobj = 0; iobj < objects.size(); ++iobj) {
			priors[iobj].mean = engine.pose(iobj);
		}
		const auto status = engine.run(std::span{points}, params, std::span{priors});
		if (status != RunStatus::ok) {
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
		const Eigen::Vector3f error_vector = estimated.translation() - expected.translation();
		const float error = error_vector.norm();

		// 事後共分散の並進ブロックから NEES を出す
		const Eigen::Matrix3f covariance =
			engine.posterior_information(iobj).inverse().topLeftCorner<3, 3>();
		const float nees = error > 1e-9f
			? error_vector.dot(covariance.inverse() * error_vector)
			: 0.f;

		// 見えていないオブジェクト (陰に隠れている等) は評価しない。
		// publish されたもの (= updated) が正しいことと、
		// 十分な数が推定できていることを見る。
		const bool visible = engine.correspondence_count(iobj) > 0;
		const bool is_updated = engine.status(iobj) == ObjectStatus::updated;
		const char* tag = "--";
		if (is_updated) {
			++updated;
			// 位置が十分近いか、共分散と整合していればよい。
			// 事前で埋めた方向は誤差が残るが、そのぶん共分散も大きいはず。
			if (error <= position_tolerance || nees <= nees_limit) {
				tag = "ok";
			} else {
				tag = "NG";
				++failed;
			}
		} else if (!visible) {
			++invisible;
		}

		std::printf(
			"[%s] object %zu '%s': status = %s, correspondences = %zu, "
			"position error = %.4f m, NEES = %.1f\n",
			tag,
			iobj,
			objects[iobj].name.c_str(),
			to_string(engine.status(iobj)),
			engine.correspondence_count(iobj),
			static_cast<double>(error),
			static_cast<double>(nees)
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
