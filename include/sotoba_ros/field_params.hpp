#pragma once

/// @file field_params.hpp
/// フィールドの寸法 (パラメータ field.*) を ObjectsConfig に読む。sotoba_node と、同じフィールドを使う
/// ノード (oumuamua_sim の robot_sim など) で共有する。既定値は持たない。
///
/// | パラメータ | 型 | 意味 |
/// | --- | --- | --- |
/// | `field.bounds` | double[4] | 外周壁の内側の面 [xmin, ymin, xmax, ymax] |
/// | `field.wall_height` | double | 壁の高さ [m] |
/// | `field.walls` | double[4n] | 内側の壁 (厚み込みの外形) を [xmin, ymin, xmax, ymax] で並べたもの |
/// | `field.shelf` / `field.shelf_height` | double[4] / double | ビンゴ棚の箱と高さ |
/// | `field.notes_x` / `field.notes_y` / `field.note_size` | double[] / double / double | 自チームのノーツの中心と一辺 (相手は y を反転) |

#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sotoba_ros/objects.hpp"

namespace sotoba_ros {
	inline auto box_from(const std::vector<double>& v, const std::string& what) -> Box2 {
		if (v.size() != 4) {
			throw std::invalid_argument(what + " must be [xmin, ymin, xmax, ymax]");
		}
		return Box2{static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2]), static_cast<float>(v[3])};
	}

	/// field.* を宣言して config の bounds / wall_height / walls / shelf / shelf_height / notes_* / note_size を埋める
	inline void read_field(rclcpp::Node& node, ObjectsConfig& config) {
		config.bounds = box_from(node.declare_parameter<std::vector<double>>("field.bounds"), "field.bounds");
		config.wall_height = static_cast<float>(node.declare_parameter<double>("field.wall_height"));
		const auto walls = node.declare_parameter<std::vector<double>>("field.walls");
		if (walls.size() % 4 != 0) {
			throw std::invalid_argument("field.walls must be [xmin, ymin, xmax, ymax, ...] (a multiple of 4 numbers)");
		}
		config.walls.clear();
		for (std::size_t i = 0; i < walls.size(); i += 4) {
			config.walls.push_back(box_from({walls[i], walls[i + 1], walls[i + 2], walls[i + 3]}, "field.walls"));
		}
		config.shelf = box_from(node.declare_parameter<std::vector<double>>("field.shelf"), "field.shelf");
		config.shelf_height = static_cast<float>(node.declare_parameter<double>("field.shelf_height"));
		config.notes_x.clear();
		for (const double x : node.declare_parameter<std::vector<double>>("field.notes_x")) {
			config.notes_x.push_back(static_cast<float>(x));
		}
		config.notes_y = static_cast<float>(node.declare_parameter<double>("field.notes_y"));
		config.note_size = static_cast<float>(node.declare_parameter<double>("field.note_size"));
	}
} // namespace sotoba_ros
