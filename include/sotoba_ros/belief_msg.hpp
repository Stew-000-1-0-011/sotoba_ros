#pragma once

/// @file belief_msg.hpp
/// Belief と BeliefArray メッセージの相互変換。
///
/// 情報行列の添字順は **[回転3, 並進3]** で、ROS の PoseWithCovariance
/// ([x, y, z, rx, ry, rz]) とは逆。変換をここ1箇所に閉じ込める。

#include <span>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>

#include "sotoba_ros/belief.hpp"
#include "sotoba_ros/msg/belief_array.hpp"

namespace sotoba_ros {
	/// 対角ブロックだけを詰めた BeliefArray を作る。
	/// (オブジェクト間の相互相関は未対応。Phase 3 で非対角ブロックを足す)
	auto to_belief_msg(
		std::span<const std::string> names,
		std::span<const Belief> beliefs,
		const builtin_interfaces::msg::Time& stamp,
		const std::string& frame_id,
		std::span<const std::uint8_t> status = {}
	) -> msg::BeliefArray;

	/// 名前で突き合わせて beliefs を上書きする。
	/// メッセージに無い名前の要素は触らない。上書きできた数を返す。
	auto from_belief_msg(
		const msg::BeliefArray& message,
		std::span<const std::string> names,
		std::span<Belief> beliefs
	) -> std::size_t;
} // namespace sotoba_ros
