#pragma once

/// @file sotoba_node.hpp
/// /scan (sensor_msgs::msg::LaserScan) を受けて、与えられたオブジェクト群の
/// LiDAR座標系での姿勢をICPで推定し Pose として publish するノード。
///
/// オブジェクト群はこのヘッダでは決め打ちにせず、コンストラクタで受け取る。
/// 実際に何を渡すかは sotoba_node.cpp 側 (objects.hpp の make_objects()) の仕事。
///
/// ICPの実体 (テンプレート展開) は IcpEngine の pimpl の向こうにあるので、
/// このヘッダをインクルードしても sotoba の ICP ヘッダは引きずられない。

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sotoba_ros/objects.hpp"

namespace sotoba_ros {
	/// publish するトピック (いずれもノード名前空間下)
	/// - `~/objects/<name>/pose` : geometry_msgs::msg::PoseStamped (オブジェクトごと)
	/// - `~/object_poses`        : geometry_msgs::msg::PoseArray   (全オブジェクトまとめ)
	///
	/// frame_id は入力 LaserScan の frame_id、stamp も入力のものをそのまま使う。
	/// publish される姿勢は「オブジェクトローカル座標系をLiDAR座標系へ写すSE3」、
	/// すなわちLiDAR座標系から見たオブジェクトの位置姿勢である。
	///
	/// 主なパラメータ (詳細は config/sotoba_node.yaml と README を参照):
	/// - `scan_topic`            : 購読するトピック (既定 "/scan")
	/// - `max_points`            : 1スキャンで扱う最大点数 (ICPバッファの容量)
	/// - `point_stride`          : 点の間引き。1なら間引かない
	/// - `range_min` / `range_max` : 採用する距離の範囲 [m] (0以下ならスキャン側の値)
	/// - `max_loop_num`, `accept_distance`, `accept_distance_begin`,
	///   `convergence_delta`, `tikhonov` (6要素), `sigma_range`, `sigma_angle`, `huber_k`
	/// - `reset_after_failures`  : 連続失敗がこの回数を超えたら初期姿勢に戻す (0で無効)
	class SotobaNode final : public rclcpp::Node {
	public:
		/// @param objects 推定対象のオブジェクト群。空ならICPは走らず警告を出し続ける。
		/// @param options 通常の rclcpp::NodeOptions。
		explicit SotobaNode(
			std::vector<ObjectDef> objects,
			const rclcpp::NodeOptions& options = rclcpp::NodeOptions{}
		);
		~SotobaNode() override;

		SotobaNode(const SotobaNode&) = delete;
		auto operator=(const SotobaNode&) -> SotobaNode& = delete;

	private:
		struct Impl;
		std::unique_ptr<Impl> impl_;
	};
} // namespace sotoba_ros
