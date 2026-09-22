/// @file fake_scan_publisher.cpp
/// objects.cpp の形状から合成 LaserScan を作って流すテスト用ノード。
///
/// 各オブジェクトを「初期姿勢を少しずらした姿勢」に置き、
/// センサ原点からレイキャストしてスキャンを作る。
/// 置いた姿勢 (真値) は ~/truth_poses にも出すので、
/// sotoba_node の推定と直接比べられる。
///
/// objects.cpp にしか依存しないので、フィールド定義を書き換えてもそのまま使える。

#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <variant>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <sotoba/math/quaternion.hpp>
#include <sotoba/math/se3.hpp>
#include <sotoba/math/vec.hpp>

#include "sotoba_ros/objects.hpp"

namespace {
	using sotoba::math::SE3;
	using sotoba::math::UVec3;
	using sotoba::math::Vec3;
	using sotoba_ros::ObjectDef;

	class FakeScanPublisher final : public rclcpp::Node {
	public:
		FakeScanPublisher() : rclcpp::Node{"fake_scan_publisher"} {
			this->objects_ = sotoba_ros::make_objects();

			this->frame_id_ = this->declare_parameter<std::string>("frame_id", "laser");
			this->ray_num_ = static_cast<std::size_t>(
				std::max<std::int64_t>(2, this->declare_parameter<std::int64_t>("ray_num", 720))
			);
			this->angle_min_ = static_cast<float>(this->declare_parameter<double>(
				"angle_min",
				-static_cast<double>(std::numbers::pi_v<float>)
			));
			this->angle_max_ = static_cast<float>(this->declare_parameter<double>(
				"angle_max",
				static_cast<double>(std::numbers::pi_v<float>)
			));
			this->range_min_ =
				static_cast<float>(this->declare_parameter<double>("range_min", 0.05));
			this->range_max_ =
				static_cast<float>(this->declare_parameter<double>("range_max", 30.0));
			this->range_noise_ =
				static_cast<float>(this->declare_parameter<double>("range_noise_stddev", 0.0));

			// 起動時のずれ (センサ座標系での平面運動)。
			// sotoba_node の初期シードは objects.cpp の初期姿勢なので、これが初期誤差になる。
			this->shift_x_ = static_cast<float>(this->declare_parameter<double>("shift_x", 0.02));
			this->shift_y_ =
				static_cast<float>(this->declare_parameter<double>("shift_y", -0.012));
			this->shift_yaw_ =
				static_cast<float>(this->declare_parameter<double>("shift_yaw", 0.006));

			// ロボットの運動。静止した世界をロボットが動きながら見るので、
			// 全オブジェクトに同じセンサ座標系の変換がかかる。
			// 0 なら静止 (起動時のずれだけ)。
			this->motion_amplitude_ =
				static_cast<float>(this->declare_parameter<double>("motion_amplitude", 0.0));
			this->motion_yaw_amplitude_ =
				static_cast<float>(this->declare_parameter<double>("motion_yaw_amplitude", 0.0));
			this->motion_period_ =
				static_cast<float>(this->declare_parameter<double>("motion_period", 6.0));

			const auto hz = this->declare_parameter<double>("scan_hz", 10.0);

			this->scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
				"/scan",
				rclcpp::SensorDataQoS{}
			);
			this->truth_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
				"~/truth_poses",
				rclcpp::QoS{10}
			);
			this->timer_ = this->create_wall_timer(
				std::chrono::duration<double>(1.0 / std::max(0.1, hz)),
				[this]() { this->tick(); }
			);

			RCLCPP_INFO(
				this->get_logger(),
				"publishing fake /scan (%zu rays) for %zu object(s)",
				this->ray_num_,
				this->objects_.size()
			);
		}

	private:
		/// 時刻 t におけるロボットの運動 (センサ座標系での変換)。
		auto motion(const float t) const -> SE3 {
			if (!(this->motion_amplitude_ > 0.f) && !(this->motion_yaw_amplitude_ > 0.f)) {
				return SE3::ide();
			}
			const float w = 2.f * std::numbers::pi_v<float> / std::max(0.1f, this->motion_period_);
			return SE3::rot(sotoba::math::quaternion::ypr(
					   Vec3{0.f, 0.f, this->motion_yaw_amplitude_ * std::sin(w * t)}
				   ))
				* SE3::trans(Vec3{
					  this->motion_amplitude_ * std::sin(w * t),
					  this->motion_amplitude_ * (1.f - std::cos(w * t)),
					  0.f
				  });
		}

		/// このスキャンでの真の姿勢を作り直す。
		void update_truth(const float t) {
			const auto shift =
				SE3::rot(sotoba::math::quaternion::ypr(Vec3{0.f, 0.f, this->shift_yaw_}))
				* SE3::trans(Vec3{this->shift_x_, this->shift_y_, 0.f});
			const auto pose = this->motion(t) * shift;

			this->truth_.clear();
			this->truth_.reserve(this->objects_.size());
			for (const auto& object : this->objects_) {
				this->truth_.emplace_back(pose * object.initial_pose);
			}
		}

		/// 真の姿勢に置いた全曲面へのレイキャスト。距離の二乗を返す。
		auto cast(const UVec3& ray) const -> float {
			float nearest2 = std::numeric_limits<float>::infinity();
			for (std::size_t iobj = 0; iobj < this->objects_.size(); ++iobj) {
				for (const auto& surface : this->objects_[iobj].surfaces) {
					std::visit(
						[&](auto moved) {
							moved.apply_se3(this->truth_[iobj]);
							const float d2 = moved.ray_collision(ray);
							if (d2 < nearest2) { nearest2 = d2; }
						},
						surface
					);
				}
			}
			return nearest2;
		}

		void tick() {
			const auto stamp = this->now();
			if (this->start_.nanoseconds() == 0) { this->start_ = stamp; }
			this->update_truth(static_cast<float>((stamp - this->start_).seconds()));

			sensor_msgs::msg::LaserScan scan{};
			scan.header.stamp = stamp;
			scan.header.frame_id = this->frame_id_;
			scan.angle_min = this->angle_min_;
			scan.angle_max = this->angle_max_;
			scan.angle_increment = (this->angle_max_ - this->angle_min_)
				/ static_cast<float>(this->ray_num_);
			scan.range_min = this->range_min_;
			scan.range_max = this->range_max_;
			scan.ranges.reserve(this->ray_num_);

			for (std::size_t i = 0; i < this->ray_num_; ++i) {
				const float angle =
					scan.angle_min + scan.angle_increment * static_cast<float>(i);
				const UVec3 ray{std::cos(angle), std::sin(angle), 0.f};
				const float d2 = this->cast(ray);
				if (!std::isfinite(d2)) {
					scan.ranges.emplace_back(std::numeric_limits<float>::infinity());
					continue;
				}
				float d = std::sqrt(d2);
				if (this->range_noise_ > 0.f) {
					d += std::normal_distribution<float>{0.f, this->range_noise_}(this->rng_);
				}
				scan.ranges.emplace_back(d);
			}
			this->scan_pub_->publish(scan);

			geometry_msgs::msg::PoseArray truth{};
			truth.header.stamp = stamp;
			truth.header.frame_id = this->frame_id_;
			truth.poses.reserve(this->truth_.size());
			for (const auto& pose : this->truth_) {
				geometry_msgs::msg::Pose msg{};
				msg.position.x = static_cast<double>(pose.p.x());
				msg.position.y = static_cast<double>(pose.p.y());
				msg.position.z = static_cast<double>(pose.p.z());
				msg.orientation.x = static_cast<double>(pose.uq.v.x());
				msg.orientation.y = static_cast<double>(pose.uq.v.y());
				msg.orientation.z = static_cast<double>(pose.uq.v.z());
				msg.orientation.w = static_cast<double>(pose.uq.v.w());
				truth.poses.emplace_back(msg);
			}
			this->truth_pub_->publish(truth);
		}

		std::vector<ObjectDef> objects_{};
		std::vector<SE3> truth_{};
		std::string frame_id_{};
		std::size_t ray_num_{720};
		float angle_min_{};
		float angle_max_{};
		float range_min_{};
		float range_max_{};
		float range_noise_{};
		float shift_x_{};
		float shift_y_{};
		float shift_yaw_{};
		float motion_amplitude_{};
		float motion_yaw_amplitude_{};
		float motion_period_{6.f};
		rclcpp::Time start_{0, 0, RCL_ROS_TIME};
		std::mt19937 rng_{0};

		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_{};
		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr truth_pub_{};
		rclcpp::TimerBase::SharedPtr timer_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FakeScanPublisher>());
	rclcpp::shutdown();
	return 0;
}
