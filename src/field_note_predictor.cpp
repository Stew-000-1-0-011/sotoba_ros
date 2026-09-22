/// @file field_note_predictor.cpp
/// sotoba_node の事前分布を外から与えるノード。**アプリ固有の例**。
///
/// sotoba_node 内蔵の持続予測は「各オブジェクトが独立に静止している」としか
/// 仮定しないので、ロボットが動くとノーツのような小さいオブジェクトは
/// 1スキャンでシードがズレて見失う (0.15m のノーツに対し、5 m/s・10 Hz なら
/// 1スキャンで 0.5 m 動く)。
///
/// このノードは「ノーツはフィールドに付いている」という関係を入れる:
///   ノーツの事前 = フィールドの推定姿勢 × (フィールドから見たノーツの姿勢)
/// 相対姿勢は、ノーツの推定が成功するたびに覚え直すので、
/// 試合中にノーツが動かされてもついていける。
///
/// 自分のアプリに合わせて書き換えるか、丸ごと差し替えて使うことを想定している。
/// sotoba_node 側は prior_source を external か external_or_internal にすること。

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sotoba_ros/belief.hpp"
#include "sotoba_ros/belief_msg.hpp"

namespace {
	using sotoba_ros::Belief;
	using sotoba_ros::Information;
	using sotoba_ros::SE3;

	/// "note_*" のような末尾ワイルドカード1個だけ対応する簡易マッチ。
	auto matches(const std::string& pattern, const std::string& name) -> bool {
		if (pattern.empty()) { return false; }
		if (pattern.back() != '*') { return pattern == name; }
		const auto prefix = pattern.substr(0, pattern.size() - 1);
		return name.size() >= prefix.size() && name.compare(0, prefix.size(), prefix) == 0;
	}

	class FieldNotePredictor final : public rclcpp::Node {
	public:
		FieldNotePredictor() : rclcpp::Node{"field_note_predictor"} {
			// "<子のパターン>:<親の名前>" の列。例: ["note_*:field"]
			this->rules_ = this->declare_parameter<std::vector<std::string>>(
				"attachments",
				std::vector<std::string>{"note_*:field"}
			);
			// 親の推定が信用できないときは事前を出さない
			this->require_parent_updated_ =
				this->declare_parameter<bool>("require_parent_updated", true);

			this->prior_pub_ =
				this->create_publisher<sotoba_ros::msg::BeliefArray>("prior_beliefs", rclcpp::QoS{10});
			this->posterior_sub_ = this->create_subscription<sotoba_ros::msg::BeliefArray>(
				"posterior_beliefs",
				rclcpp::QoS{10},
				[this](const sotoba_ros::msg::BeliefArray::ConstSharedPtr message) {
					this->on_posterior(*message);
				}
			);

			RCLCPP_INFO(
				this->get_logger(),
				"attachment rules: %zu (e.g. \"note_*:field\")",
				this->rules_.size()
			);
		}

	private:
		/// 名前 -> 親の名前。ルールから解決してキャッシュする。
		auto parent_of(const std::string& name) -> const std::string* {
			if (const auto it = this->parent_cache_.find(name); it != this->parent_cache_.end()) {
				return it->second.empty() ? nullptr : &it->second;
			}

			std::string parent{};
			for (const auto& rule : this->rules_) {
				const auto colon = rule.rfind(':');
				if (colon == std::string::npos) { continue; }
				const auto pattern = rule.substr(0, colon);
				const auto candidate = rule.substr(colon + 1);
				if (candidate == name) { continue; } // 自分自身は親にしない
				if (matches(pattern, name)) {
					parent = candidate;
					break;
				}
			}
			const auto [it, _] = this->parent_cache_.emplace(name, std::move(parent));
			return it->second.empty() ? nullptr : &it->second;
		}

		void on_posterior(const sotoba_ros::msg::BeliefArray& message) {
			const auto object_num = message.names.size();
			if (object_num == 0 || message.means.size() != object_num) { return; }

			std::vector<Belief> beliefs(object_num);
			const auto matched = sotoba_ros::from_belief_msg(
				message,
				std::span<const std::string>{message.names},
				std::span<Belief>{beliefs}
			);
			if (matched != object_num) { return; }

			std::unordered_map<std::string, std::size_t> index_of{};
			for (std::size_t i = 0; i < object_num; ++i) { index_of.emplace(message.names[i], i); }

			const auto status_of = [&](const std::size_t i) -> std::uint8_t {
				return i < message.status.size() ? message.status[i] : std::uint8_t{1};
			};
			constexpr std::uint8_t updated = 1;

			auto prior = beliefs;
			for (std::size_t i = 0; i < object_num; ++i) {
				const auto* const parent_name = this->parent_of(message.names[i]);
				if (parent_name == nullptr) { continue; } // 根はそのまま

				const auto it = index_of.find(*parent_name);
				if (it == index_of.end()) { continue; }
				const auto iparent = it->second;

				// 子の推定が成功していたら、親から見た相対姿勢を覚え直す
				// (= 親の中で動いたことを追う)
				if (status_of(i) == updated && status_of(iparent) == updated) {
					this->relative_[message.names[i]] =
						beliefs[iparent].mean.inv() * beliefs[i].mean;
				}

				const auto relative = this->relative_.find(message.names[i]);
				if (relative == this->relative_.end()) {
					// まだ一度も相対姿勢が取れていないので、初期値として今の姿勢から作る
					this->relative_.emplace(
						message.names[i],
						beliefs[iparent].mean.inv() * beliefs[i].mean
					);
					continue;
				}

				if (this->require_parent_updated_ && status_of(iparent) != updated) { continue; }

				// これが本題: 子の事前は「親の推定 × 相対姿勢」
				prior[i].mean = beliefs[iparent].mean * relative->second;
				// 注意: 親の不確かさは子に伝えていない (非対角ブロックが要る。Phase 3)
			}

			this->prior_pub_->publish(sotoba_ros::to_belief_msg(
				std::span<const std::string>{message.names},
				std::span<const Belief>{prior},
				message.header.stamp,
				message.header.frame_id
			));
		}

		std::vector<std::string> rules_{};
		bool require_parent_updated_{true};
		std::unordered_map<std::string, std::string> parent_cache_{};
		std::unordered_map<std::string, SE3> relative_{};

		rclcpp::Publisher<sotoba_ros::msg::BeliefArray>::SharedPtr prior_pub_{};
		rclcpp::Subscription<sotoba_ros::msg::BeliefArray>::SharedPtr posterior_sub_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FieldNotePredictor>());
	rclcpp::shutdown();
	return 0;
}
