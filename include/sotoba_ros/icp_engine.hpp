#pragma once

/// @file icp_engine.hpp
/// sotoba の ICP をノードから切り離して包むラッパ。
///
/// sotoba の ICP はテンプレート (曲面の型リストで実体化される) なので、
/// ヘッダに出すとインクルードした全TUで重い実体化が走る。
/// ここでは pimpl にして実体化を src/icp_engine.cpp の1TUに閉じ込める。
/// そのため、このヘッダは sotoba/icp_resource/* を一切インクルードしない。

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <sotoba/math/vec.hpp>

#include "sotoba_ros/belief.hpp"
#include "sotoba_ros/objects.hpp"

namespace sotoba_ros {
	/// 点対面残差の分散モデル。sotoba::icp_resource::NoiseModel と同じ意味。
	struct NoiseParams final {
		float sigma_range; ///< [m]
		float sigma_angle; ///< [rad]
	};

	/// run() 1回分のパラメータ。
	struct IcpParams final {
		/// 最大反復回数。
		std::uint32_t max_loop_num{20};
		/// 対応点として採用する距離 [m] (最終反復でのゲート)。
		float accept_distance{0.30f};
		/// 0より大きければ、1回目をこの距離、最終反復を accept_distance として
		/// 等比でゲートを絞る。初期ずれが大きいときに効く。
		float accept_distance_begin{0.f};
		/// 姿勢更新量がこれ未満になったら打ち切る [m] 相当。0なら打ち切らない。
		float convergence_delta{0.f};
		/// Tikhonov正則化 (LM減衰。事前分布とは別概念)。
		/// 添字 0..2 が並進、3..5 が回転 (sotoba の接空間と同じ順序)。
		std::array<float, 6> tikhonov{};
		/// 指定すると点ごとの重みを距離依存の分散で決める。
		std::optional<NoiseParams> noise{};
		/// 正規化残差に対するHuberの閾値。noise 無指定なら単位は [m]。
		std::optional<float> huber_k{};
	};

	/// run() 自体の成否。sotoba::icp_resource::IcpError と1対1。
	enum class RunStatus : std::uint8_t {
		ok = 0,
		too_many_points,
		invalid_weighting,
		invalid_accept_schedule,
		invalid_accept_distance,
		invalid_loop_num,
		prior_size_mismatch,
		/// 非ゼロの事前分布があるのに noise が設定されていない。
		/// 情報行列は物理単位を持つので、重みも 1/σ² でなければ足せない。
		prior_requires_noise_model,
		invalid_prior_information,
	};

	/// オブジェクトごとの直近の更新結果。sotoba::icp_resource::ObjStatus と1対1。
	enum class ObjectStatus : std::uint8_t {
		not_run = 0,
		updated,
		too_few_correspondences,
		solve_failed,
	};

	auto to_string(RunStatus status) noexcept -> const char*;
	auto to_string(ObjectStatus status) noexcept -> const char*;

	/// ICPの資源 (点群バッファ、曲面バッファ、姿勢) を保持して使い回す。
	///
	/// 点群容量は構築時に固定され、run() はそれを超える点群を受け取ると
	/// 何もせずに too_many_points を返す (再確保しない)。
	/// 容量を増やしたければ作り直すこと。

	class IcpEngine final {
	public:
		/// @param objects 推定対象。空でないこと。曲面が1つも無いオブジェクトは
		///        対応点が取れないので、呼び出し側で弾くのが望ましい。
		/// @param points_capacity 1スキャンで受け付ける最大点数。
		IcpEngine(std::span<const ObjectDef> objects, std::size_t points_capacity);
		~IcpEngine();

		IcpEngine(IcpEngine&&) noexcept;
		auto operator=(IcpEngine&&) noexcept -> IcpEngine&;
		IcpEngine(const IcpEngine&) = delete;
		auto operator=(const IcpEngine&) -> IcpEngine& = delete;

		/// 次の run() のシードとなる姿勢を設定する (LiDAR座標系)。
		void set_pose(std::size_t iobj, const SE3& pose) noexcept;

		/// ObjectDef::initial_pose へ戻す。
		void reset_pose(std::size_t iobj) noexcept;
		/// 直近の推定姿勢 (オブジェクトローカル -> センサ座標系)。
		auto pose(std::size_t iobj) const noexcept -> SE3;

		/// センサ座標系の点群でICPを1回走らせる。
		///
		/// @param priors オブジェクトごとの事前分布。空なら事前なし。
		///        非空なら object_count() と同じ長さであること。
		///        事前は**正規方程式に入る**ので、観測できない方向は事前のまま残り、
		///        1面しか見えないオブジェクトも解ける (sotoba が1点から解いてくれる)。
		auto run(
			std::span<const sotoba::math::Vec3> points,
			const IcpParams& params,
			std::span<const Belief> priors = {}
		) noexcept -> RunStatus;

		auto status(std::size_t iobj) const noexcept -> ObjectStatus;
		auto correspondence_count(std::size_t iobj) const noexcept -> std::size_t;
		auto last_loop_count() const noexcept -> std::uint32_t;

		auto object_count() const noexcept -> std::size_t;
		auto points_capacity() const noexcept -> std::size_t;

		/// 直近の run() における、オブジェクトごとの観測の情報行列 A = Σ w JᵀJ。
		/// 添字 0..2 が並進 (センサ座標系)、3..5 が回転。
		auto observation_information(std::size_t iobj) const noexcept -> Information;

		/// 直近の run() で解いた事後の情報行列 H = A + JinvᵀΛJinv + diag(tikhonov)。
		/// 事前を渡していればそれも含んだ値。
		///
		/// 注意: 楽観的な値。対応付けの誤りと地図の誤差を含まず、点が独立に
		/// 正規分布するという仮定のもとの Fisher 情報でしかない。
		auto posterior_information(std::size_t iobj) const noexcept -> Information;

		/// SE3 の6自由度を決めるのに最低限必要な対応点数 (事前なしのとき)。
		static auto min_correspondences() noexcept -> std::size_t;

		/// 非ゼロの事前分布があるときの必要対応点数。
		/// 事前が正定値なら H も正定値なので、1点から解ける。
		static auto min_correspondences_with_prior() noexcept -> std::size_t;

	private:
		struct Impl;
		std::unique_ptr<Impl> impl_;
	};
} // namespace sotoba_ros
