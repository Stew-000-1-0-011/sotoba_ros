/// @file belief_msg.cpp

#include "sotoba_ros/belief_msg.hpp"

#include <cstdint>
#include <unordered_map>

namespace sotoba_ros {
	namespace {
		auto to_pose_msg(const SE3& pose) -> geometry_msgs::msg::Pose {
			geometry_msgs::msg::Pose message{};
			const auto& t = pose.translation();
			const auto& q = pose.unit_quaternion();
			message.position.x = static_cast<double>(t.x());
			message.position.y = static_cast<double>(t.y());
			message.position.z = static_cast<double>(t.z());
			message.orientation.x = static_cast<double>(q.x());
			message.orientation.y = static_cast<double>(q.y());
			message.orientation.z = static_cast<double>(q.z());
			message.orientation.w = static_cast<double>(q.w());
			return message;
		}

		auto from_pose_msg(const geometry_msgs::msg::Pose& message) -> SE3 {
			Eigen::Quaternionf q{
				static_cast<float>(message.orientation.w),
				static_cast<float>(message.orientation.x),
				static_cast<float>(message.orientation.y),
				static_cast<float>(message.orientation.z)
			};
			q.normalize();
			return SE3{
				Sophus::SO3f{q},
				Eigen::Vector3f{
					static_cast<float>(message.position.x),
					static_cast<float>(message.position.y),
					static_cast<float>(message.position.z)
				}
			};
		}
	} // namespace

	auto to_belief_msg(
		std::span<const std::string> names,
		std::span<const Belief> beliefs,
		const builtin_interfaces::msg::Time& stamp,
		const std::string& frame_id,
		std::span<const std::uint8_t> status
	) -> msg::BeliefArray {
		msg::BeliefArray message{};
		message.header.stamp = stamp;
		message.header.frame_id = frame_id;
		message.names.reserve(names.size());
		message.means.reserve(beliefs.size());
		message.blocks.reserve(beliefs.size());

		for (std::size_t iobj = 0; iobj < beliefs.size() && iobj < names.size(); ++iobj) {
			message.names.emplace_back(names[iobj]);
			message.means.emplace_back(to_pose_msg(beliefs[iobj].mean));

			msg::InformationBlock block{};
			block.i = static_cast<std::uint8_t>(iobj);
			block.j = static_cast<std::uint8_t>(iobj);
			for (int i = 0; i < 6; ++i) {
				for (int j = 0; j < 6; ++j) {
					block.information[i * 6 + j] =
						static_cast<double>(beliefs[iobj].information(i, j));
				}
			}
			message.blocks.emplace_back(block);
		}
		message.status.assign(status.begin(), status.end());

		return message;
	}

	auto from_belief_msg(
		const msg::BeliefArray& message,
		std::span<const std::string> names,
		std::span<Belief> beliefs
	) -> std::size_t {
		std::unordered_map<std::string, std::size_t> index_of{};
		for (std::size_t i = 0; i < names.size(); ++i) { index_of.emplace(names[i], i); }

		// 名前 -> メッセージ内の添字
		std::vector<std::size_t> message_to_local(message.names.size(), names.size());
		std::size_t matched = 0;
		for (std::size_t imsg = 0; imsg < message.names.size(); ++imsg) {
			const auto it = index_of.find(message.names[imsg]);
			if (it == index_of.end()) { continue; }
			message_to_local[imsg] = it->second;
			if (imsg < message.means.size()) {
				beliefs[it->second].mean = from_pose_msg(message.means[imsg]);
				beliefs[it->second].information = Information::Zero();
				++matched;
			}
		}

		// 対角ブロックのみ取り込む (非対角は Phase 3 で対応)
		for (const auto& block : message.blocks) {
			if (block.i != block.j) { continue; }
			if (block.i >= message_to_local.size()) { continue; }
			const auto ilocal = message_to_local[block.i];
			if (ilocal >= names.size()) { continue; }

			Information information = Information::Zero();
			for (int i = 0; i < 6; ++i) {
				for (int j = 0; j < 6; ++j) {
					information(i, j) = 0.5f
						* static_cast<float>(block.information[i * 6 + j] + block.information[j * 6 + i]);
				}
			}
			beliefs[ilocal].information = information;
		}

		return matched;
	}
} // namespace sotoba_ros
