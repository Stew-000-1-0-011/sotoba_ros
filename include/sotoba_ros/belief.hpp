#pragma once

/// @file belief.hpp
/// オブジェクトの信念分布 (ガウス) と、その時間伝播。
///
/// sotoba_node の内蔵予測と、外部の予測ノードの両方が使う。
/// ROSにもICPにも依存しないので単体でテストできる。

#include <span>
#include <vector>

#include <sotoba/math/se3.hpp>
#include <sotoba/math/sym_mat.hpp>

namespace sotoba_ros {
	using sotoba::math::SE3;
	/// 情報行列 (共分散の逆)。添字 0..2 が回転、3..5 が並進。
	/// 平均まわりの局所座標で、摂動は左 (センサ座標系)。
	/// ゼロは「全く分からない」を表せる (共分散では書けない)。
	using Information = sotoba::math::SymMat<6>;

	struct Belief final {
		SE3 mean{SE3::ide()};
		Information information{};
	};

	/// 等方なプロセスノイズ。単位時間あたりの分散。
	struct ProcessNoise final {
		float angular{0.f}; ///< [rad^2/s]
		float linear{0.f}; ///< [m^2/s]
	};

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
