/// @file icp_engine.cpp
/// sotoba の ICP テンプレートを実体化する唯一のTU。
/// ここだけが sotoba/icp_resource/* をインクルードする。

#include "sotoba_ros/icp_engine.hpp"

#include <stdexcept>
#include <utility>

#include <sotoba/icp_resource/normal_known_icp.hpp>
#include <sotoba/icp_resource/resource.hpp>
#include <sotoba/math/scalar_functions.hpp>

namespace sotoba_ros {
	namespace {
		using sotoba::math::SE3;
		using sotoba::math::Vec3;
		using Vec6 = sotoba::math::Vec<6>;

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

		Impl(std::span<const ObjectDef> objects, const std::size_t points_capacity)
			: resource{to_resource_from(objects, points_capacity)} {
			for (std::size_t iobj = 0; iobj < objects.size(); ++iobj) {
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

	auto IcpEngine::pose(const std::size_t iobj) const noexcept -> SE3 {
		return this->impl_->resource.obj_pose(static_cast<sotoba::u8>(iobj));
	}

	auto IcpEngine::run(std::span<const Vec3> points, const IcpParams& params) noexcept
		-> RunStatus {
		const Vec6 tikhonov{
			params.tikhonov[0],
			params.tikhonov[1],
			params.tikhonov[2],
			params.tikhonov[3],
			params.tikhonov[4],
			params.tikhonov[5]
		};

		sotoba::icp_resource::IcpWeighting weighting{};
		if (params.noise) {
			weighting.noise = sotoba::icp_resource::NoiseModel{
				params.noise->sigma_range,
				params.noise->sigma_angle
			};
		}
		weighting.huber_k = params.huber_k;

		// sotoba 側は距離の二乗で受け取る。
		const auto err = this->impl_->resource.run_icp(
			points,
			tikhonov,
			params.max_loop_num,
			sotoba::math::pow2(params.accept_distance),
			sotoba::math::pow2(params.convergence_delta),
			weighting,
			params.accept_distance_begin > 0.f ? sotoba::math::pow2(params.accept_distance_begin)
											   : 0.f
		);

		return from_icp_error(err);
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

	auto IcpEngine::min_correspondences() noexcept -> std::size_t {
		return Resource::min_correspondences;
	}
} // namespace sotoba_ros
