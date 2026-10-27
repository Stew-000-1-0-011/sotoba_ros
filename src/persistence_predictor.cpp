/// @file persistence_predictor.cpp
/// sotoba_node に事前分布を出す、いちばん単純なノード: 持続予測。
///
/// 「各オブジェクトは LiDAR から見て静止している」と仮定して、直近の事後の平均をそのまま、
/// 不確かさだけ経過時間に応じて増やして出す。sotoba_node は自分では予測をしないので、
/// 単体で動かすときはこのノードを一緒に立てる。
///
/// ロボットが動くなら、機体の動きを知っているノード (状態推定など) が事前を出すこと。
/// このノードは機体の動きを知らないので、速く動くと初期値がずれて見失う。

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sotoba_ros/belief.hpp"
#include "sotoba_ros/belief_msg.hpp"

namespace {
	using sotoba_ros::Belief;
	using sotoba_ros::ProcessNoise;

	class PersistencePredictor final : public rclcpp::Node {
	public:
		PersistencePredictor() : rclcpp::Node{"persistence_predictor"} {
			const double rate = this->declare_parameter<double>("publish_rate");
			if (!(rate > 0.0)) { throw std::invalid_argument("publish_rate must be positive"); }

			// 持続予測で1秒あたりに増やす不確かさ。走査面内 (x, y, yaw) と面外 (z, roll, pitch) を分ける。
			// 面外は 2D LiDAR では観測できないので、大きくすると情報の無い方向ができて推定が壊れる
			this->noise_.angular_in_plane =
				static_cast<float>(this->declare_parameter<double>("process_noise_angular"));
			this->noise_.linear_in_plane =
				static_cast<float>(this->declare_parameter<double>("process_noise_linear"));
			this->noise_.angular_out_of_plane = static_cast<float>(
				this->declare_parameter<double>("process_noise_angular_out_of_plane")
			);
			this->noise_.linear_out_of_plane = static_cast<float>(
				this->declare_parameter<double>("process_noise_linear_out_of_plane")
			);
			// 連続してこの回数だけ更新できなかったオブジェクトを、定義上の初期姿勢に戻す (0 で無効)
			this->reset_after_failures_ = static_cast<std::size_t>(
				std::max<std::int64_t>(0, this->declare_parameter<std::int64_t>("reset_after_failures"))
			);

			this->prior_pub_ =
				this->create_publisher<sotoba_ros::msg::BeliefArray>("prior_beliefs", rclcpp::QoS{10});
			// 定義上の初期姿勢 (transient local)。ここから始め、リセットでもここへ戻す
			this->initial_sub_ = this->create_subscription<sotoba_ros::msg::BeliefArray>(
				"initial_beliefs",
				rclcpp::QoS{1}.transient_local(),
				[this](const sotoba_ros::msg::BeliefArray::ConstSharedPtr message) {
					this->on_initial(*message);
				}
			);
			this->posterior_sub_ = this->create_subscription<sotoba_ros::msg::BeliefArray>(
				"posterior_beliefs",
				rclcpp::QoS{10},
				[this](const sotoba_ros::msg::BeliefArray::ConstSharedPtr message) {
					this->on_posterior(*message);
				}
			);
			this->timer_ = this->create_wall_timer(
				std::chrono::duration<double>(1.0 / rate), [this] { this->publish_prior(); }
			);
		}

	private:
		void on_initial(const sotoba_ros::msg::BeliefArray& message) {
			this->names_ = message.names;
			this->initial_.assign(this->names_.size(), Belief{});
			sotoba_ros::from_belief_msg(
				message, std::span<const std::string>{this->names_}, std::span<Belief>{this->initial_}
			);
			this->beliefs_ = this->initial_;
			this->stamp_.reset();
			this->failure_counts_.assign(this->names_.size(), 0);
			this->ever_updated_.assign(this->names_.size(), 0);
			RCLCPP_INFO(this->get_logger(), "got %zu object(s) from sotoba", this->names_.size());
		}

		void on_posterior(const sotoba_ros::msg::BeliefArray& message) {
			if (this->names_.empty()) { return; }
			const rclcpp::Time stamp{message.header.stamp, this->get_clock()->get_clock_type()};
			// 古い事後 (順序の入れ替わり) は捨てる
			if (this->stamp_ && stamp < *this->stamp_) { return; }

			sotoba_ros::from_belief_msg(
				message, std::span<const std::string>{this->names_}, std::span<Belief>{this->beliefs_}
			);
			this->stamp_ = stamp;

			constexpr std::uint8_t updated = 1;
			for (std::size_t i = 0; i < message.names.size() && i < message.status.size(); ++i) {
				const auto it = std::ranges::find(this->names_, message.names[i]);
				if (it == this->names_.end()) { continue; }
				const auto k = static_cast<std::size_t>(it - this->names_.begin());
				if (message.status[i] == updated) {
					this->failure_counts_[k] = 0;
					this->ever_updated_[k] = 1;
					continue;
				}
				// 一度も取れていないものは「見えていない」だけなので戻さない
				if (++this->failure_counts_[k] >= this->reset_after_failures_ && this->reset_after_failures_ != 0
					&& this->ever_updated_[k] != 0) {
					this->beliefs_[k] = this->initial_[k];
					this->failure_counts_[k] = 0;
					RCLCPP_WARN(
						this->get_logger(), "object '%s' reset to its initial pose.", this->names_[k].c_str()
					);
				}
			}
		}

		void publish_prior() {
			if (this->names_.empty()) { return; }
			const auto now = this->now();
			// まだ事後が無いうちは、初期の不確かさのまま出す
			const float dt =
				this->stamp_ ? static_cast<float>(std::max(0.0, (now - *this->stamp_).seconds())) : 0.f;

			auto prior = this->beliefs_;
			for (auto& belief : prior) {
				belief.information = sotoba_ros::propagate(belief.information, this->noise_, dt);
			}
			this->prior_pub_->publish(sotoba_ros::to_belief_msg(
				std::span<const std::string>{this->names_}, std::span<const Belief>{prior}, now, ""
			));
		}

		ProcessNoise noise_{};
		std::size_t reset_after_failures_{0};

		std::vector<std::string> names_{};
		std::vector<Belief> initial_{};
		/// 直近の事後 (まだ無ければ初期姿勢) と、その時刻
		std::vector<Belief> beliefs_{};
		std::optional<rclcpp::Time> stamp_{};
		std::vector<std::size_t> failure_counts_{};
		/// 一度でも推定できたか
		std::vector<std::uint8_t> ever_updated_{};

		rclcpp::Publisher<sotoba_ros::msg::BeliefArray>::SharedPtr prior_pub_{};
		rclcpp::Subscription<sotoba_ros::msg::BeliefArray>::SharedPtr initial_sub_{};
		rclcpp::Subscription<sotoba_ros::msg::BeliefArray>::SharedPtr posterior_sub_{};
		rclcpp::TimerBase::SharedPtr timer_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	try {
		rclcpp::spin(std::make_shared<PersistencePredictor>());
	} catch (const std::exception& e) {
		// パラメータが足りないときなど (既定値は持たないので、全部パラメータファイルで与える)
		RCLCPP_FATAL(rclcpp::get_logger("persistence_predictor"), "%s", e.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::shutdown();
	return 0;
}
