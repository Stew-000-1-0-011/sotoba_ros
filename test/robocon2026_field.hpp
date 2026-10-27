#pragma once

/// @file robocon2026_field.hpp
/// テスト用 (icp_smoke_test, fake_scan_publisher) のフィールド。sotoba_ros 自体は既定値を持たないので、
/// テストではここの値を使う。千葉大ロボコン2026のフィールドで、oumuamua の
/// src/oumuamua_bringup/config/field.yaml と同じ寸法。

#include "sotoba_ros/objects.hpp"

namespace sotoba_ros::test {
	inline auto robocon2026_config() -> ObjectsConfig {
		return ObjectsConfig{
			.lidar_height = 0.14f,
			.wall_height = 0.3f,
			.bounds = {-2.786f, -1.655f, 2.786f, 1.655f},
			.walls = {
				{-2.8025f, -0.0165f, 1.5625f, 0.0165f},
				{-0.8375f, 0.019f, -0.8045f, 0.8905f},
				{-0.8375f, -0.8905f, -0.8045f, -0.019f},
				{-1.5955f, 1.069f, -1.5625f, 1.669f},
				{-1.5955f, -1.669f, -1.5625f, -1.069f},
				{-1.5955f, 0.0165f, -1.5625f, 0.2665f},
				{-1.5955f, -0.2665f, -1.5625f, -0.0165f},
				{0.0045f, 0.8715f, 0.0375f, 1.6715f},
				{0.0045f, -1.6715f, 0.0375f, -0.8715f},
				{0.8045f, 0.019f, 0.8375f, 0.8905f},
				{0.8045f, -0.8905f, 0.8375f, -0.019f},
				{2.5025f, -0.0165f, 2.8025f, 0.0165f},
			},
			.shelf = {1.5625f, -0.30f, 2.5025f, 0.30f},
			.shelf_height = 0.9f,
			.notes_x = {-2.699f, -2.499f, -2.299f, -2.099f, -1.899f, -1.699f},
			.notes_y = 0.125f,
			.note_size = 0.15f,
			// 棚から2つ目のスラローム壁 (x=+0.021) の延長線上、センターライン壁までの隙間の中央で +y を向く
			.start_x = 0.021f,
			.start_y = 0.444f,
			.start_yaw = 1.5707963f,
			.lidar_upside_down = true,
			.include_notes = true,
			.include_shelf = true,
		};
	}
} // namespace sotoba_ros::test
