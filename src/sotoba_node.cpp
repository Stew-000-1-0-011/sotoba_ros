/// @file sotoba_node.cpp
/// 実行ファイルのエントリポイント。
/// objects.hpp 由来のオブジェクト群をここで受け取り、ノードへ渡す。
/// ノード自体はオブジェクトの中身を知らないので、形状を変えたいときは
/// objects.cpp だけを直せばよい。

#include <cstdio>
#include <exception>
#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "sotoba_ros/objects.hpp"
#include "sotoba_ros/sotoba_node.hpp"

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);

	int ret = 0;
	try {
		auto objects = sotoba_ros::make_objects();
		auto node = std::make_shared<sotoba_ros::SotobaNode>(std::move(objects));
		rclcpp::spin(node);
	} catch (const std::exception& e) {
		std::fprintf(stderr, "sotoba_node: %s\n", e.what());
		ret = 1;
	}

	rclcpp::shutdown();
	return ret;
}
