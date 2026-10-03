#pragma once

/// @file belief.hpp
/// オブジェクトの信念分布 (ガウス) と、その時間伝播。
///
/// sotoba_node の内蔵予測と、外部の予測ノードの両方が使う。
/// ROSにもICPにも依存しないので単体でテストできる。
///
/// 接空間の成分順序は sotoba (Sophus::SE3f::Tangent) と同じ **(並進, 回転)**。
/// 添字0..2が並進、3..5が回転。摂動は左 (センサ座標系) で T = exp(ξ) mean。

#include <array>
#include <span>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace sotoba_ros {
	using SE3 = Sophus::SE3f;

	/// 情報行列 (共分散の逆)。平均まわりの局所座標で、ゼロは「全く分からない」。
	/// sotoba の ObjPrior::information / posterior_information と同じ成分順序。
	using Information = Eigen::Matrix<float, 6, 6>;

	struct Belief final {
		SE3 mean{};
		Information information{Information::Zero()};
	};

	/// プロセスノイズ。単位時間あたりの分散。
	///
	/// 走査面 (センサ座標系の z=0 平面) の内と外を分けている。
	/// 2D LiDAR では面外の3自由度 (z, roll, pitch) は**観測できない**ので、
	/// ここを等方に扱うと、事前も観測もゼロ情報になって推定が破綻する。
	/// 実際には LiDAR の取付高も傾きも動かないので、面外はほぼ0でよい。
	struct ProcessNoise final {
		float angular_in_plane{0.f}; ///< yaw [rad^2/s]
		float angular_out_of_plane{0.f}; ///< roll/pitch [rad^2/s]
		float linear_in_plane{0.f}; ///< x, y [m^2/s]
		float linear_out_of_plane{0.f}; ///< z [m^2/s]
	};

	/// 標準偏差 (添字 0..2 が並進 [m]、3..5 が回転 [rad]) から対角の情報行列を作る。
	/// sigma <= 0 の成分は「完全に既知」として大きな情報を入れる。
	auto diagonal_information(const std::array<float, 6>& sigma) -> Information;

	/// 姿勢についての持続予測。
	///
	/// 平均はそのまま (静止仮定)、不確かさだけ増やす:
	///   Σ ← Σ + Q dt,  すなわち  Λ ← (I + Λ Q dt)^{-1} Λ
	/// 後者の形で計算するので、Λ = 0 (何も分からない) でも破綻しない。
	auto propagate(const Information& information, const ProcessNoise& noise, float dt)
		-> Information;

	/// 情報形式での融合。同じ平均のまわりで線形化されていること。
	inline auto fuse(const Information& a, const Information& b) -> Information {
		return a + b;
	}
} // namespace sotoba_ros
