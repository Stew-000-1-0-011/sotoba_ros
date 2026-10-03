/// @file icp_engine.cpp
/// sotoba の ICP テンプレートを実体化する唯一のTU。
/// ここだけが sotoba/icp_resource/* をインクルードする。

#include "sotoba_ros/icp_engine.hpp"

#include <stdexcept>
#include <utility>

#include <sotoba/icp_resource/normal_known_icp.hpp>
#include <sotoba/icp_resource/resource.hpp>
#include <sotoba/math/scalar_functions.hpp>

#include <vector>

namespace sotoba_ros {
	namespace {
		using sotoba::math::Vec3;

		using Resource = sotoba::icp_resource::NormalKnownResource<
			sotoba::surface::BoxInner,
			sotoba::surface::BoxOuter,
			sotoba::surface::Rectangle,
			sotoba::surface::CylinderOuter>;

		auto from_icp_error(const sotoba::icp_resource::IcpError e) noexcept -> RunStatus {
			switch (e) {
			case sotoba::icp_resource::IcpError::none: return RunStatus::ok;
			case sotoba::icp_resource::IcpError::too_many_points:
				return RunStatus::too_many_points;
			case sotoba::icp_resource::IcpError::invalid_weighting:
				return RunStatus::invalid_weighting;
			case sotoba::icp_resource::IcpError::invalid_accept_schedule:
				return RunStatus::invalid_accept_schedule;
			case sotoba::icp_resource::IcpError::invalid_accept_distance:
				return RunStatus::invalid_accept_distance;
			case sotoba::icp_resource::IcpError::invalid_loop_num:
				return RunStatus::invalid_loop_num;
			case sotoba::icp_resource::IcpError::prior_size_mismatch:
				return RunStatus::prior_size_mismatch;
			case sotoba::icp_resource::IcpError::prior_requires_noise_model:
				return RunStatus::prior_requires_noise_model;
			case sotoba::icp_resource::IcpError::invalid_prior_information:
				return RunStatus::invalid_prior_information;
			}
			return RunStatus::ok;
		}

		auto from_obj_status(const sotoba::icp_resource::ObjStatus s) noexcept -> ObjectStatus {
			switch (s) {
			case sotoba::icp_resource::ObjStatus::not_run: return ObjectStatus::not_run;
			case sotoba::icp_resource::ObjStatus::updated: return ObjectStatus::updated;
			case sotoba::icp_resource::ObjStatus::too_few_correspondences:
				return ObjectStatus::too_few_correspondences;
			case sotoba::icp_resource::ObjStatus::solve_failed: return ObjectStatus::solve_failed;
			}
			return ObjectStatus::not_run;
		}

		/// ObjectDef の曲面群を to_resource が食える形 (span の列) に均す。
		auto to_resource_from(std::span<const ObjectDef> objects, const std::size_t points_capacity)
			-> Resource {
			if (objects.empty()) { throw std::runtime_error{"sotoba_ros: no objects given"}; }
			if (points_capacity == 0) {
				throw std::runtime_error{"sotoba_ros: points_capacity must be positive"};
			}

			std::vector<std::span<const Surface>> spans{};
			spans.reserve(objects.size());
			for (const auto& obj : objects) { spans.emplace_back(std::span{obj.surfaces}); }

			return sotoba::icp_resource::to_resource<sotoba::icp_resource::NormalKnownResource>(
				points_capacity,
				std::span<const std::span<const Surface>>{spans}
			);
		}
	} // namespace

	auto to_string(const RunStatus status) noexcept -> const char* {
		switch (status) {
		case RunStatus::ok: return "ok";
		case RunStatus::too_many_points: return "too_many_points";
		case RunStatus::invalid_weighting: return "invalid_weighting";
		case RunStatus::invalid_accept_schedule: return "invalid_accept_schedule";
		case RunStatus::invalid_accept_distance: return "invalid_accept_distance";
		case RunStatus::invalid_loop_num: return "invalid_loop_num";
		case RunStatus::prior_size_mismatch: return "prior_size_mismatch";
		case RunStatus::prior_requires_noise_model: return "prior_requires_noise_model";
		case RunStatus::invalid_prior_information: return "invalid_prior_information";
		}
		return "unknown";
	}

	auto to_string(const ObjectStatus status) noexcept -> const char* {
		switch (status) {
		case ObjectStatus::not_run: return "not_run";
		case ObjectStatus::updated: return "updated";
		case ObjectStatus::too_few_correspondences: return "too_few_correspondences";
		case ObjectStatus::solve_failed: return "solve_failed";
		}
		return "unknown";
	}

	struct IcpEngine::Impl final {
		Resource resource;
		/// ObjectDef::initial_pose (リセット用)。
		std::vector<SE3> initial_poses;
		/// run() のたびに組み直す事前分布 (span の寿命を持たせるため)。
		std::vector<sotoba::icp_resource::ObjPrior> priors;

		Impl(std::span<const ObjectDef> objects, const std::size_t points_capacity)
			: resource{to_resource_from(objects, points_capacity)} {
			this->initial_poses.reserve(objects.size());
			for (std::size_t iobj = 0; iobj < objects.size(); ++iobj) {
				this->initial_poses.emplace_back(objects[iobj].initial_pose);
				this->resource.obj_pose(static_cast<sotoba::u8>(iobj)) =
					objects[iobj].initial_pose;
			}
		}
	};

	IcpEngine::IcpEngine(std::span<const ObjectDef> objects, const std::size_t points_capacity)
		: impl_{std::make_unique<Impl>(objects, points_capacity)} {}

	IcpEngine::~IcpEngine() = default;
	IcpEngine::IcpEngine(IcpEngine&&) noexcept = default;
	auto IcpEngine::operator=(IcpEngine&&) noexcept -> IcpEngine& = default;

	void IcpEngine::set_pose(const std::size_t iobj, const SE3& pose) noexcept {
		this->impl_->resource.obj_pose(static_cast<sotoba::u8>(iobj)) = pose;
	}

	void IcpEngine::reset_pose(const std::size_t iobj) noexcept {
		this->impl_->resource.obj_pose(static_cast<sotoba::u8>(iobj)) =
			this->impl_->initial_poses[iobj];
	}

	auto IcpEngine::pose(const std::size_t iobj) const noexcept -> SE3 {
		return this->impl_->resource.obj_pose(static_cast<sotoba::u8>(iobj));
	}

	auto IcpEngine::run(
		std::span<const Vec3> points,
		const IcpParams& params,
		std::span<const Belief> priors
	) noexcept -> RunStatus {
		// sotoba の接空間と同じ (並進, 回転) の順
		Sophus::SE3f::Tangent tikhonov{};
		for (int i = 0; i < 6; ++i) { tikhonov[i] = params.tikhonov[static_cast<std::size_t>(i)]; }

		sotoba::icp_resource::IcpWeighting weighting{};
		if (params.noise) {
			weighting.noise = sotoba::icp_resource::NoiseModel{
				params.noise->sigma_range,
				params.noise->sigma_angle
			};
		}
		weighting.huber_k = params.huber_k;

		// 事前分布を sotoba の形へ。平均まわりの左摂動という規約はそのまま。
		this->impl_->priors.clear();
		if (!priors.empty()) {
			this->impl_->priors.reserve(priors.size());
			for (const auto& prior : priors) {
				this->impl_->priors.emplace_back(
					sotoba::icp_resource::ObjPrior{prior.mean, prior.information}
				);
			}
		}

		const sotoba::icp_resource::IcpParams sotoba_params{
			.max_loop_num = params.max_loop_num,
			// sotoba 側は距離の二乗で受け取る
			.accept_distance2 = sotoba::math::pow2(params.accept_distance),
			.convergence_delta2 = sotoba::math::pow2(params.convergence_delta),
			.accept_distance2_begin = params.accept_distance_begin > 0.f
				? sotoba::math::pow2(params.accept_distance_begin)
				: 0.f,
			.tikhonov = tikhonov,
			.weighting = weighting,
			.priors = std::span<const sotoba::icp_resource::ObjPrior>{this->impl_->priors}
		};

		return from_icp_error(this->impl_->resource.run_icp(points, sotoba_params));
	}

	auto IcpEngine::status(const std::size_t iobj) const noexcept -> ObjectStatus {
		return from_obj_status(this->impl_->resource.obj_status(static_cast<sotoba::u8>(iobj)));
	}

	auto IcpEngine::correspondence_count(const std::size_t iobj) const noexcept -> std::size_t {
		return this->impl_->resource.correspondence_count(static_cast<sotoba::u8>(iobj));
	}

	auto IcpEngine::last_loop_count() const noexcept -> std::uint32_t {
		return this->impl_->resource.last_loop_count();
	}

	auto IcpEngine::object_count() const noexcept -> std::size_t {
		return this->impl_->resource.obj_num;
	}

	auto IcpEngine::points_capacity() const noexcept -> std::size_t {
		return this->impl_->resource.points_capacity();
	}

	auto IcpEngine::observation_information(const std::size_t iobj) const noexcept -> Information {
		return this->impl_->resource.information_matrix(static_cast<sotoba::u8>(iobj));
	}

	auto IcpEngine::posterior_information(const std::size_t iobj) const noexcept -> Information {
		return this->impl_->resource.posterior_information(static_cast<sotoba::u8>(iobj));
	}

	auto IcpEngine::min_correspondences() noexcept -> std::size_t {
		return Resource::min_correspondences;
	}

	auto IcpEngine::min_correspondences_with_prior() noexcept -> std::size_t {
		return Resource::min_correspondences_with_prior;
	}
} // namespace sotoba_ros
