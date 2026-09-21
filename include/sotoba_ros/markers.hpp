#pragma once

/// @file markers.hpp
/// オブジェクトの形状と姿勢を RViz2 で見るための MarkerArray を組み立てる。
///
/// ICPとは独立なので、ここも別TU (src/markers.cpp) に分かれている。

#include <cstdint>
#include <span>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sotoba/math/se3.hpp>

#include "sotoba_ros/objects.hpp"

namespace sotoba_ros {
	/// 見た目の設定。
	struct MarkerStyle final {
		/// 線の太さ [m]
		float line_width{0.03f};
		/// 直近のスキャンで更新できたオブジェクトの色 (RGBA)
		std::array<float, 4> color_fresh{{0.1f, 0.9f, 0.3f, 1.0f}};
		/// 更新できなかった (姿勢が古い) オブジェクトの色 (RGBA)
		std::array<float, 4> color_stale{{0.6f, 0.6f, 0.6f, 0.6f}};
		/// Marker の寿命 [s]。0なら消えない
		float lifetime{0.f};
		/// 曲面の法線を描く長さ [m]。0なら描かない
		float normal_length{0.2f};
		/// オブジェクト名をテキストで描くか
		bool show_labels{true};
	};

	/// オブジェクト群の形状を、与えられた姿勢に置いた MarkerArray にする。
	///
	/// @param objects 形状 (オブジェクトローカル座標系)
	/// @param poses   各オブジェクトの姿勢。objects と同じ長さであること
	/// @param fresh   直近のスキャンで更新できたか。objects と同じ長さであること
	/// @param frame_id マーカーを置くフレーム (通常はLiDARのフレーム)
	auto build_object_markers(
		std::span<const ObjectDef> objects,
		std::span<const sotoba::math::SE3> poses,
		std::span<const std::uint8_t> fresh,
		const std::string& frame_id,
		const builtin_interfaces::msg::Time& stamp,
		const MarkerStyle& style = {}
	) -> visualization_msgs::msg::MarkerArray;
} // namespace sotoba_ros
