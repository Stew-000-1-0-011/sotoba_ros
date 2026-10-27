/// @file sotoba_node.cpp
/// 実行ファイルのエントリポイント。
/// objects.hpp 由来のオブジェクト群をここで受け取り、ノードへ渡す。
/// フィールドの形 (field.*)、LiDAR の取付、初期姿勢はすべて ROS パラメータから読む
/// (既定値は持たない)。ここがこのファイルの主な仕事。

#include <cmath>
#include <cstdio>
#include <exception>
#include <memory>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sotoba_ros/objects.hpp"
#include "sotoba_ros/field_params.hpp"
#include "sotoba_ros/sotoba_node.hpp"

namespace {
	/// ノードのパラメータから ObjectsConfig を作る。
	auto read_config(rclcpp::Node& node) -> sotoba_ros::ObjectsConfig {
		sotoba_ros::ObjectsConfig config{};

		config.lidar_height = static_cast<float>(node.declare_parameter<double>("lidar_height"));
		// フィールドの形 (field.*)
		sotoba_ros::read_field(node, config);
		// 初期シード = LiDAR の初期姿勢。機体のスタート位置 (field.start_pose) に LiDAR の取付 (lidar_x / lidar_y / lidar_yaw) を足す
		const auto start = node.declare_parameter<std::vector<double>>("field.start_pose");
		if (start.size() != 3) {
			throw std::invalid_argument("field.start_pose must be [x, y, yaw]");
		}
		const double lidar_x = node.declare_parameter<double>("lidar_x");
		const double lidar_y = node.declare_parameter<double>("lidar_y");
		const double lidar_yaw = node.declare_parameter<double>("lidar_yaw");
		config.start_x = static_cast<float>(start[0] + std::cos(start[2]) * lidar_x - std::sin(start[2]) * lidar_y);
		config.start_y = static_cast<float>(start[1] + std::sin(start[2]) * lidar_x + std::cos(start[2]) * lidar_y);
		config.start_yaw = static_cast<float>(start[2] + lidar_yaw);
		config.include_notes =
			node.declare_parameter<bool>("include_notes");
		config.include_shelf =
			node.declare_parameter<bool>("include_shelf");
		config.lidar_upside_down =
			node.declare_parameter<bool>("lidar_upside_down");

		// 実機で一番やらかしやすいところなので、起動時に検算しておく。
		if (config.lidar_height >= config.wall_height) {
			RCLCPP_ERROR(
				node.get_logger(),
				"lidar_height (%.3f m) >= wall_height (%.3f m): the scan plane passes above "
				"the walls, so nothing will be seen.",
				static_cast<double>(config.lidar_height),
				static_cast<double>(config.wall_height)
			);
		}
		if (config.include_notes && config.lidar_height >= config.note_size) {
			RCLCPP_WARN(
				node.get_logger(),
				"lidar_height (%.3f m) is not below the note size (%.3f m): notes will be "
				"invisible to the scan.",
				static_cast<double>(config.lidar_height),
				static_cast<double>(config.note_size)
			);
		}

		RCLCPP_INFO(
			node.get_logger(),
			"objects: lidar_height=%.3f wall_height=%.3f start=(%.3f, %.3f, %.3f rad) "
			"upside_down=%s notes=%s shelf=%s",
			static_cast<double>(config.lidar_height),
			static_cast<double>(config.wall_height),
			static_cast<double>(config.start_x),
			static_cast<double>(config.start_y),
			static_cast<double>(config.start_yaw),
			config.lidar_upside_down ? "yes" : "no",
			config.include_notes ? "yes" : "no",
			config.include_shelf ? "yes" : "no"
		);

		return config;
	}
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);

	int ret = 0;
	try {
		auto node = std::make_shared<sotoba_ros::SotobaNode>(
			[](rclcpp::Node& self) { return sotoba_ros::make_objects(read_config(self)); }
		);
		rclcpp::spin(node);
	} catch (const std::exception& e) {
		std::fprintf(stderr, "sotoba_node: %s\n", e.what());
		ret = 1;
	}

	rclcpp::shutdown();
	return ret;
}
