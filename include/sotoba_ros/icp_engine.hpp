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

#include <sotoba/math/se3.hpp>
#include <sotoba/math/vec.hpp>

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
		/// Tikhonov正則化。添字 0..2 が回転 (w)、3..5 が並進 (t)。
		/// 2D LiDAR では z / roll / pitch が観測できないので、
		/// 対応する成分に正の値を入れて解が暴れないようにすること。
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
	///
	/// ObjectDef::parent が指定されたオブジェクトの面倒もここで見る。
	/// run() のたびに「親の姿勢 × 親から見た相対姿勢」でシードを作り直し、
	/// 推定が成功したら相対姿勢を更新する。
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
		/// 親を持つオブジェクトに対しては、親から見た相対姿勢もここで更新される。
		void set_pose(std::size_t iobj, const sotoba::math::SE3& pose) noexcept;

		/// 初期姿勢へ戻す。親を持つオブジェクトは相対姿勢のほうを初期値へ戻す。
		void reset_pose(std::size_t iobj) noexcept;
		/// 直近の推定姿勢 (オブジェクトローカル -> センサ座標系)。
		auto pose(std::size_t iobj) const noexcept -> sotoba::math::SE3;

		/// センサ座標系の点群でICPを1回走らせる。
		auto run(std::span<const sotoba::math::Vec3> points, const IcpParams& params) noexcept
			-> RunStatus;

		auto status(std::size_t iobj) const noexcept -> ObjectStatus;
		auto correspondence_count(std::size_t iobj) const noexcept -> std::size_t;
		auto last_loop_count() const noexcept -> std::uint32_t;

		auto object_count() const noexcept -> std::size_t;
		auto points_capacity() const noexcept -> std::size_t;

		/// 親の指定が解決できなかったオブジェクトについての説明。
		/// (未知の名前、自分自身、入れ子が2段以上)。構築時に確定する。
		auto warnings() const noexcept -> std::span<const std::string>;

		/// SE3 の6自由度を決めるのに最低限必要な対応点数。
		static auto min_correspondences() noexcept -> std::size_t;

	private:
		struct Impl;
		std::unique_ptr<Impl> impl_;
	};
} // namespace sotoba_ros
