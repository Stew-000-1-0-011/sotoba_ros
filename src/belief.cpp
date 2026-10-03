/// @file belief.cpp

#include "sotoba_ros/belief.hpp"

#include <Eigen/Dense>

namespace sotoba_ros {
	auto diagonal_information(const std::array<float, 6>& sigma) -> Information {
		Information information = Information::Zero();
		for (int i = 0; i < 6; ++i) {
			// sigma <= 0 は「完全に既知」。無限大は入れられないので十分大きな値。
			const float s = sigma[static_cast<std::size_t>(i)] > 0.f
				? sigma[static_cast<std::size_t>(i)]
				: 1e-4f;
			information(i, i) = 1.f / (s * s);
		}
		return information;
	}

	auto propagate(const Information& information, const ProcessNoise& noise, const float dt)
		-> Information {
		if (!(dt > 0.f)) { return information; }

		// 添字 0,1 = x,y (面内)、2 = z (面外)、3,4 = roll/pitch (面外)、5 = yaw (面内)
		Information q = Information::Zero();
		q(0, 0) = noise.linear_in_plane * dt;
		q(1, 1) = noise.linear_in_plane * dt;
		q(2, 2) = noise.linear_out_of_plane * dt;
		q(3, 3) = noise.angular_out_of_plane * dt;
		q(4, 4) = noise.angular_out_of_plane * dt;
		q(5, 5) = noise.angular_in_plane * dt;
		if (q.isZero(0.f)) { return information; }

		// Λ⁺ = (I + Λ Q)^{-1} Λ。Λ = 0 なら 0 のまま。
		const Information a = Information::Identity() + information * q;
		const Information propagated = a.partialPivLu().solve(information);
		// 数値誤差で崩れた対称性をここで均す
		return 0.5f * (propagated + propagated.transpose());
	}
} // namespace sotoba_ros
