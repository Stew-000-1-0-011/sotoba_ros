/// @file sotoba_node.cpp
/// 実行ファイルのエントリポイント。
/// objects.hpp 由来のオブジェクト群をここで受け取り、ノードへ渡す。
/// ノード自体はオブジェクトの中身を知らないので、形状を変えたいときは
/// objects.cpp だけを直せばよい。
///
/// 実機に合わせて変えたい寸法 (LiDAR取付高、壁の高さ、初期姿勢) は
/// ROSパラメータから読む。ここがこのファイルの主な仕事。

#include <cstdio>
#include <exception>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sotoba_ros/objects.hpp"
#include "sotoba_ros/sotoba_node.hpp"

namespace {
	/// ノードのパラメータから ObjectsConfig を作る。
	auto read_config(rclcpp::Node& node) -> sotoba_ros::ObjectsConfig {
		const sotoba_ros::ObjectsConfig defaults{};
		sotoba_ros::ObjectsConfig config{};

		config.lidar_height = static_cast<float>(node.declare_parameter<double>(
			"lidar_height",
			static_cast<double>(defaults.lidar_height)
		));
		config.wall_height = static_cast<float>(node.declare_parameter<double>(
			"wall_height",
			static_cast<double>(defaults.wall_height)
		));
		config.wall_thickness = static_cast<float>(node.declare_parameter<double>(
			"wall_thickness",
			static_cast<double>(defaults.wall_thickness)
		));
		config.start_x = static_cast<float>(
			node.declare_parameter<double>("start_x", static_cast<double>(defaults.start_x))
		);
		config.start_y = static_cast<float>(
			node.declare_parameter<double>("start_y", static_cast<double>(defaults.start_y))
		);
		config.start_yaw = static_cast<float>(
			node.declare_parameter<double>("start_yaw", static_cast<double>(defaults.start_yaw))
		);
		config.include_notes =
			node.declare_parameter<bool>("include_notes", defaults.include_notes);

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
		if (config.include_notes && config.lidar_height >= 0.15f) {
			RCLCPP_WARN(
				node.get_logger(),
				"lidar_height (%.3f m) is not below the note size (0.15 m): notes will be "
				"invisible to the scan.",
				static_cast<double>(config.lidar_height)
			);
		}

		RCLCPP_INFO(
			node.get_logger(),
			"objects: lidar_height=%.3f wall_height=%.3f start=(%.3f, %.3f, %.3f rad) notes=%s",
			static_cast<double>(config.lidar_height),
			static_cast<double>(config.wall_height),
			static_cast<double>(config.start_x),
			static_cast<double>(config.start_y),
			static_cast<double>(config.start_yaw),
			config.include_notes ? "yes" : "no"
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
