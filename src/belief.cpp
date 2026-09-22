/// @file belief.cpp

#include "sotoba_ros/belief.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Dense>

namespace sotoba_ros {
	namespace {
		using Matrix6f = Eigen::Matrix<float, 6, 6>;

		auto to_eigen(const Information& information) -> Matrix6f {
			Matrix6f m{};
			for (std::uint8_t i = 0; i < 6; ++i) {
				for (std::uint8_t j = 0; j < 6; ++j) { m(i, j) = information[i, j]; }
			}
			return m;
		}

		auto from_eigen(const Matrix6f& m) -> Information {
			Information information{};
			for (std::uint8_t i = 0; i < 6; ++i) {
				for (std::uint8_t j = i; j < 6; ++j) {
					// 数値誤差で崩れた対称性をここで均す
					information[i, j] = 0.5f * (m(i, j) + m(j, i));
				}
			}
			return information;
		}
	} // namespace

	auto propagate(const Information& information, const ProcessNoise& noise, const float dt)
		-> Information {
		if (!(dt > 0.f)) { return information; }
		if (!(noise.angular > 0.f) && !(noise.linear > 0.f)) { return information; }

		const auto lambda = to_eigen(information);

		Matrix6f q = Matrix6f::Zero();
		for (std::uint8_t i = 0; i < 3; ++i) {
			q(i, i) = noise.angular * dt;
			q(i + 3, i + 3) = noise.linear * dt;
		}

		// Λ⁺ = (I + Λ Q)^{-1} Λ。Λ = 0 なら 0 のまま。
		const Matrix6f a = Matrix6f::Identity() + lambda * q;
		const Eigen::PartialPivLU<Matrix6f> lu(a);
		return from_eigen(lu.solve(lambda));
	}
} // namespace sotoba_ros
