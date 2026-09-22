/// @file sotoba_node_impl.cpp
/// SotobaNode の実装。ROS側の入出力とパラメータの面倒を見て、
/// 推定そのものは IcpEngine に投げる。

#include "sotoba_ros/sotoba_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sotoba/math/se3.hpp>
#include <sotoba/math/vec.hpp>

#include "sotoba_ros/belief.hpp"
#include "sotoba_ros/belief_msg.hpp"
#include "sotoba_ros/icp_engine.hpp"
#include "sotoba_ros/markers.hpp"

namespace sotoba_ros {
	namespace {
		using sotoba::math::SE3;
		using sotoba::math::Vec3;

		auto to_msg(const SE3& pose) -> geometry_msgs::msg::Pose {
			geometry_msgs::msg::Pose msg{};
			msg.position.x = static_cast<double>(pose.p.x());
			msg.position.y = static_cast<double>(pose.p.y());
			msg.position.z = static_cast<double>(pose.p.z());
			// sotoba の UQuaternion は v = (x, y, z, w)。
			msg.orientation.x = static_cast<double>(pose.uq.v.x());
			msg.orientation.y = static_cast<double>(pose.uq.v.y());
			msg.orientation.z = static_cast<double>(pose.uq.v.z());
			msg.orientation.w = static_cast<double>(pose.uq.v.w());
			return msg;
		}

		/// トピック名に使えない文字を潰す。
		auto sanitize(const std::string& name, const std::size_t iobj) -> std::string {
			std::string ret{};
			ret.reserve(name.size());
			for (const char c : name) {
				const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
					|| (c >= '0' && c <= '9') || c == '_';
				ret.push_back(ok ? c : '_');
			}
			if (ret.empty() || (ret.front() >= '0' && ret.front() <= '9')) {
				ret.insert(ret.begin(), 'o');
			}
			if (ret.empty()) { ret = "object_" + std::to_string(iobj); }
			return ret;
		}
	} // namespace

	struct SotobaNode::Impl final {
		rclcpp::Node& node;

		std::vector<ObjectDef> objects;
		std::vector<std::string> names;
		std::vector<std::string> topic_names;
		std::vector<std::size_t> failure_counts;

		/// 直近の事後分布。次スキャンの事前のもとになる。
		std::vector<Belief> beliefs{};
		/// 外部から来た事前 (まだ使っていなければ has_external_prior が true)。
		std::vector<Belief> external_prior{};
		bool has_external_prior{false};
		rclcpp::Time external_prior_stamp{};
		/// 前スキャンの時刻。dt を出すのに使う。
		rclcpp::Time last_scan_stamp{};
		bool has_last_scan{false};

		/// 点群容量が足りなくなったら作り直すので optional。
		std::unique_ptr<IcpEngine> engine{};

		std::vector<Vec3> points{};

		std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs{};
		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub{};
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub{};
		rclcpp::Publisher<msg::BeliefArray>::SharedPtr posterior_pub{};
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub{};
		rclcpp::Subscription<msg::BeliefArray>::SharedPtr prior_sub{};

		// --- パラメータ ---
		std::size_t max_points{2048};
		std::size_t point_stride{1};
		float range_min{0.f};
		float range_max{0.f};
		std::size_t reset_after_failures{0};
		IcpParams icp_params{};
		bool publish_markers{true};
		MarkerStyle marker_style{};
		/// internal / external / external_or_internal
		std::string prior_source{"internal"};
		ProcessNoise process_noise{};
		double prior_timeout{0.5};

		explicit Impl(rclcpp::Node& node, std::vector<ObjectDef>&& objects)
			: node{node}, objects{std::move(objects)} {
			this->declare_params();
			this->prepare_objects();
			this->create_pubs();
			this->create_sub();
		}

		void declare_params() {
			auto& n = this->node;

			this->max_points = static_cast<std::size_t>(
				std::max<std::int64_t>(1, n.declare_parameter<std::int64_t>("max_points", 2048))
			);
			this->point_stride = static_cast<std::size_t>(
				std::max<std::int64_t>(1, n.declare_parameter<std::int64_t>("point_stride", 1))
			);
			this->range_min = static_cast<float>(n.declare_parameter<double>("range_min", 0.0));
			this->range_max = static_cast<float>(n.declare_parameter<double>("range_max", 0.0));
			this->reset_after_failures = static_cast<std::size_t>(std::max<std::int64_t>(
				0,
				n.declare_parameter<std::int64_t>("reset_after_failures", 0)
			));

			this->icp_params.max_loop_num = static_cast<std::uint32_t>(
				std::max<std::int64_t>(0, n.declare_parameter<std::int64_t>("max_loop_num", 20))
			);
			this->icp_params.accept_distance =
				static_cast<float>(n.declare_parameter<double>("accept_distance", 0.3));
			this->icp_params.accept_distance_begin =
				static_cast<float>(n.declare_parameter<double>("accept_distance_begin", 0.0));
			this->icp_params.convergence_delta =
				static_cast<float>(n.declare_parameter<double>("convergence_delta", 0.0));

			// 2D LiDAR では z / roll / pitch が観測できないので、既定で強めに拘束する。
			// yaw にも僅かに入れてあるのは、円柱のような回転対称オブジェクトで
			// 正規方程式がランク落ちして solve_failed になるのを防ぐため。
			const auto tikhonov = n.declare_parameter<std::vector<double>>(
				"tikhonov",
				std::vector<double>{1.0, 1.0, 0.01, 0.0, 0.0, 1.0}
			);
			if (tikhonov.size() == 6) {
				for (std::size_t i = 0; i < 6; ++i) {
					this->icp_params.tikhonov[i] = static_cast<float>(tikhonov[i]);
				}
			} else {
				RCLCPP_WARN(
					n.get_logger(),
					"parameter 'tikhonov' must have 6 elements (got %zu). using zeros.",
					tikhonov.size()
				);
			}

			const auto sigma_range = n.declare_parameter<double>("sigma_range", 0.0);
			const auto sigma_angle = n.declare_parameter<double>("sigma_angle", 0.0);
			if (sigma_range > 0.0 || sigma_angle > 0.0) {
				this->icp_params.noise =
					NoiseParams{static_cast<float>(sigma_range), static_cast<float>(sigma_angle)};
			}
			const auto huber_k = n.declare_parameter<double>("huber_k", 0.0);
			if (huber_k > 0.0) { this->icp_params.huber_k = static_cast<float>(huber_k); }

			// --- 事前分布 ---
			this->prior_source = n.declare_parameter<std::string>("prior_source", "internal");
			if (this->prior_source != "internal" && this->prior_source != "external"
				&& this->prior_source != "external_or_internal") {
				RCLCPP_WARN(
					n.get_logger(),
					"unknown prior_source '%s'. falling back to 'internal'.",
					this->prior_source.c_str()
				);
				this->prior_source = "internal";
			}
			this->process_noise.angular =
				static_cast<float>(n.declare_parameter<double>("process_noise_angular", 0.5));
			this->process_noise.linear =
				static_cast<float>(n.declare_parameter<double>("process_noise_linear", 0.5));
			this->prior_timeout = n.declare_parameter<double>("prior_timeout", 0.5);

			// --- RViz2 表示 ---
			this->publish_markers = n.declare_parameter<bool>("publish_markers", true);
			this->marker_style.line_width =
				static_cast<float>(n.declare_parameter<double>("marker_line_width", 0.03));
			this->marker_style.lifetime =
				static_cast<float>(n.declare_parameter<double>("marker_lifetime", 0.0));
			this->marker_style.normal_length =
				static_cast<float>(n.declare_parameter<double>("marker_normal_length", 0.2));
			this->marker_style.show_labels =
				n.declare_parameter<bool>("marker_show_labels", true);
		}

		void prepare_objects() {
			auto& n = this->node;

			this->topic_names.reserve(this->objects.size());
			this->names.reserve(this->objects.size());
			this->beliefs.assign(this->objects.size(), Belief{});
			this->external_prior.assign(this->objects.size(), Belief{});

			std::unordered_map<std::string, std::size_t> used{};
			for (std::size_t iobj = 0; iobj < this->objects.size(); ++iobj) {
				const auto& obj = this->objects[iobj];
				if (obj.surfaces.empty()) {
					RCLCPP_WARN(
						n.get_logger(),
						"object '%s' has no surface. it will never be updated.",
						obj.name.c_str()
					);
				}

				auto topic = sanitize(obj.name, iobj);
				// 同名オブジェクトがあるとトピックが衝突するので連番で逃がす。
				if (const auto [it, inserted] = used.emplace(topic, iobj); !inserted) {
					topic += "_" + std::to_string(iobj);
					used.emplace(topic, iobj);
				}
				this->topic_names.emplace_back(std::move(topic));
				this->names.emplace_back(obj.name);
				// 初期姿勢が最初の事前。情報行列ゼロ = 「姿勢以外に何も知らない」
				this->beliefs[iobj].mean = obj.initial_pose;
			}
			this->failure_counts.assign(this->objects.size(), 0);
		}

		void create_pubs() {
			auto& n = this->node;
			this->pose_pubs.reserve(this->topic_names.size());
			for (const auto& topic : this->topic_names) {
				this->pose_pubs.emplace_back(
					n.create_publisher<geometry_msgs::msg::PoseStamped>(
						"~/objects/" + topic + "/pose",
						rclcpp::QoS{10}
					)
				);
			}
			this->pose_array_pub =
				n.create_publisher<geometry_msgs::msg::PoseArray>("~/object_poses", rclcpp::QoS{10});

			this->posterior_pub =
				n.create_publisher<msg::BeliefArray>("~/posterior_beliefs", rclcpp::QoS{10});

			if (this->publish_markers) {
				// RViz2 を後から起動しても形状が見えるよう transient local にする。
				this->marker_pub = n.create_publisher<visualization_msgs::msg::MarkerArray>(
					"~/object_markers",
					rclcpp::QoS{1}.transient_local()
				);
			}
		}

		void create_sub() {
			auto& n = this->node;
			const auto topic = n.declare_parameter<std::string>("scan_topic", "/scan");
			this->scan_sub = n.create_subscription<sensor_msgs::msg::LaserScan>(
				topic,
				rclcpp::SensorDataQoS{},
				[this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
					this->on_scan(*msg);
				}
			);
			if (this->prior_source != "internal") {
				this->prior_sub = n.create_subscription<msg::BeliefArray>(
					"~/prior_beliefs",
					rclcpp::QoS{10},
					[this](const msg::BeliefArray::ConstSharedPtr message) {
						this->on_prior(*message);
					}
				);
			}

			RCLCPP_INFO(
				n.get_logger(),
				"subscribing '%s' for %zu object(s), prior_source = %s",
				topic.c_str(),
				this->objects.size(),
				this->prior_source.c_str()
			);
		}

		/// LaserScan を LiDAR 座標系の点群へ。z は 0 (スキャン平面上)。
		void fill_points(const sensor_msgs::msg::LaserScan& scan) {
			this->points.clear();

			const float lo = this->range_min > 0.f ? std::max(this->range_min, scan.range_min)
												   : scan.range_min;
			const float hi =
				this->range_max > 0.f ? std::min(this->range_max, scan.range_max) : scan.range_max;

			for (std::size_t i = 0; i < scan.ranges.size(); i += this->point_stride) {
				const float r = scan.ranges[i];
				if (!std::isfinite(r) || r < lo || r > hi) { continue; }

				const float angle =
					scan.angle_min + scan.angle_increment * static_cast<float>(i);
				this->points.emplace_back(r * std::cos(angle), r * std::sin(angle), 0.f);
				if (this->points.size() >= this->max_points) { break; }
			}
		}

		/// 点群容量が足りていればそのまま、足りなければ作り直す。
		auto ensure_engine(const std::size_t points_num) -> bool {
			if (this->engine && this->engine->points_capacity() >= points_num) { return true; }

			const std::size_t capacity = std::max<std::size_t>(points_num, 1);
			try {
				auto engine = std::make_unique<IcpEngine>(
					std::span<const ObjectDef>{this->objects},
					capacity
				);
				// 作り直しでも信念は引き継ぐ (シードは毎スキャン入れ直すので姿勢は任意)。
				this->engine = std::move(engine);
			} catch (const std::exception& e) {
				RCLCPP_ERROR_THROTTLE(
					this->node.get_logger(),
					*this->node.get_clock(),
					5000,
					"failed to build ICP engine: %s",
					e.what()
				);
				return false;
			}
			RCLCPP_INFO(
				this->node.get_logger(),
				"ICP engine (re)built with points_capacity = %zu",
				capacity
			);
			return true;
		}

		/// 外部予測ノードからの事前。ここでは溜めるだけで、
		/// スキャン時刻までの差分は build_prior() が持続予測で埋める。
		void on_prior(const msg::BeliefArray& message) {
			const auto matched = from_belief_msg(
				message,
				std::span<const std::string>{this->names},
				std::span<Belief>{this->external_prior}
			);
			if (matched == 0) {
				RCLCPP_WARN_THROTTLE(
					this->node.get_logger(),
					*this->node.get_clock(),
					5000,
					"prior_beliefs matched no object by name. check the publisher."
				);
				return;
			}
			this->external_prior_stamp = rclcpp::Time{message.header.stamp};
			this->has_external_prior = true;
		}

		/// このスキャンに対する事前分布を作る。
		///
		/// 内蔵の持続予測は「平均そのまま、不確かさだけ増やす」。
		/// 外部の事前があれば、その時刻からスキャン時刻までの差分だけ
		/// 同じ持続予測で前進させてから使う (外部はスキャン周期を知らなくてよい)。
		/// 戻り値が false ならこのスキャンは捨てる。
		auto build_prior(const rclcpp::Time& stamp, std::vector<Belief>& prior) -> bool {
			const bool want_external = this->prior_source != "internal";
			bool used_external = false;

			if (want_external && this->has_external_prior) {
				const auto age = (stamp - this->external_prior_stamp).seconds();
				if (age >= 0.0 && age <= this->prior_timeout) {
					prior = this->external_prior;
					// 外部の時刻からスキャン時刻までを埋める
					for (auto& belief : prior) {
						belief.information = propagate(
							belief.information,
							this->process_noise,
							static_cast<float>(age)
						);
					}
					used_external = true;
				} else {
					RCLCPP_WARN_THROTTLE(
						this->node.get_logger(),
						*this->node.get_clock(),
						5000,
						"external prior is %.3f s old (timeout %.3f s).",
						age,
						this->prior_timeout
					);
				}
			} else if (want_external) {
				RCLCPP_WARN_THROTTLE(
					this->node.get_logger(),
					*this->node.get_clock(),
					5000,
					"no external prior received yet."
				);
			}

			if (!used_external) {
				if (this->prior_source == "external") { return false; }

				// 内蔵の持続予測
				const float dt = this->has_last_scan
					? static_cast<float>((stamp - this->last_scan_stamp).seconds())
					: 0.f;
				prior = this->beliefs;
				for (auto& belief : prior) {
					belief.information =
						propagate(belief.information, this->process_noise, dt);
				}
			}

			return true;
		}

		void on_scan(const sensor_msgs::msg::LaserScan& scan) {
			if (this->objects.empty()) {
				RCLCPP_WARN_THROTTLE(
					this->node.get_logger(),
					*this->node.get_clock(),
					5000,
					"no object is given. nothing to estimate."
				);
				return;
			}

			this->fill_points(scan);
			if (this->points.size() < IcpEngine::min_correspondences()) {
				RCLCPP_WARN_THROTTLE(
					this->node.get_logger(),
					*this->node.get_clock(),
					5000,
					"too few valid points in scan (%zu).",
					this->points.size()
				);
				return;
			}

			if (!this->ensure_engine(this->points.size())) { return; }

			const rclcpp::Time stamp{scan.header.stamp};
			std::vector<Belief> prior{};
			if (!this->build_prior(stamp, prior)) { return; }

			// 事前の平均をICPのシードにする。
			// (情報行列そのものを solve に入れるのは sotoba 側の対応待ち)
			for (std::size_t iobj = 0; iobj < this->objects.size(); ++iobj) {
				this->engine->set_pose(iobj, prior[iobj].mean);
			}

			const auto run_status = this->engine->run(std::span{this->points}, this->icp_params);
			if (run_status != RunStatus::ok) {
				RCLCPP_ERROR_THROTTLE(
					this->node.get_logger(),
					*this->node.get_clock(),
					5000,
					"run_icp failed: %s",
					to_string(run_status)
				);
				return;
			}

			this->update_beliefs(prior);
			this->last_scan_stamp = stamp;
			this->has_last_scan = true;
			this->has_external_prior = false;

			this->publish(scan);
		}

		/// 事後 = 事前 + 観測。
		///
		/// 平均は ICP の出力をそのまま使う (現状のICPは事前を解に入れていないので、
		/// 厳密にはMAPでなく最尤推定)。情報行列は事前と観測の和。
		void update_beliefs(const std::vector<Belief>& prior) {
			for (std::size_t iobj = 0; iobj < this->objects.size(); ++iobj) {
				this->beliefs[iobj].mean = this->engine->pose(iobj);
				if (this->engine->status(iobj) == ObjectStatus::updated) {
					this->beliefs[iobj].information =
						fuse(prior[iobj].information, this->engine->observation_information(iobj));
				} else {
					// 観測が無かったので事前のまま
					this->beliefs[iobj].information = prior[iobj].information;
				}
			}
		}

		void publish(const sensor_msgs::msg::LaserScan& scan) {
			geometry_msgs::msg::PoseArray array{};
			array.header.stamp = scan.header.stamp;
			array.header.frame_id = scan.header.frame_id;
			array.poses.reserve(this->objects.size());

			// マーカー用。更新できなかったオブジェクトも、最後の姿勢で色を変えて出す。
			std::vector<SE3> poses{};
			std::vector<std::uint8_t> fresh{};
			poses.reserve(this->objects.size());
			fresh.reserve(this->objects.size());

			for (std::size_t iobj = 0; iobj < this->objects.size(); ++iobj) {
				const auto status = this->engine->status(iobj);
				poses.emplace_back(this->engine->pose(iobj));
				fresh.emplace_back(status == ObjectStatus::updated ? 1 : 0);

				if (status != ObjectStatus::updated) {
					++this->failure_counts[iobj];
					RCLCPP_WARN_THROTTLE(
						this->node.get_logger(),
						*this->node.get_clock(),
						5000,
						"object '%s' not updated: %s (correspondences = %zu)",
						this->objects[iobj].name.c_str(),
						to_string(status),
						this->engine->correspondence_count(iobj)
					);

					if (this->reset_after_failures != 0
						&& this->failure_counts[iobj] >= this->reset_after_failures) {
						this->engine->reset_pose(iobj);
						this->beliefs[iobj] = Belief{this->objects[iobj].initial_pose, {}};
						poses.back() = this->engine->pose(iobj);
						this->failure_counts[iobj] = 0;
						RCLCPP_WARN(
							this->node.get_logger(),
							"object '%s' reset to its initial pose.",
							this->objects[iobj].name.c_str()
						);
					}
					// 更新できなかったオブジェクトは publish しない。
					continue;
				}

				this->failure_counts[iobj] = 0;

				geometry_msgs::msg::PoseStamped msg{};
				msg.header.stamp = scan.header.stamp;
				msg.header.frame_id = scan.header.frame_id;
				msg.pose = to_msg(this->engine->pose(iobj));
				this->pose_pubs[iobj]->publish(msg);
				array.poses.emplace_back(msg.pose);
			}

			if (!array.poses.empty()) { this->pose_array_pub->publish(array); }

			// 信念分布は全オブジェクトぶん出す (更新できなかったものも含む)。
			// 外部の予測ノードはこれを見て次の事前を作る。
			std::vector<std::uint8_t> status{};
			status.reserve(this->objects.size());
			for (std::size_t iobj = 0; iobj < this->objects.size(); ++iobj) {
				status.emplace_back(static_cast<std::uint8_t>(this->engine->status(iobj)));
			}
			this->posterior_pub->publish(to_belief_msg(
				std::span<const std::string>{this->names},
				std::span<const Belief>{this->beliefs},
				scan.header.stamp,
				scan.header.frame_id,
				std::span<const std::uint8_t>{status}
			));

			if (this->marker_pub) {
				this->marker_pub->publish(build_object_markers(
					std::span{this->objects},
					std::span{poses},
					std::span{fresh},
					scan.header.frame_id,
					scan.header.stamp,
					this->marker_style
				));
			}
		}
	};

	SotobaNode::SotobaNode(std::vector<ObjectDef> objects, const rclcpp::NodeOptions& options)
		: rclcpp::Node{"sotoba_node", options}
		, impl_{std::make_unique<Impl>(*this, std::move(objects))} {}

	SotobaNode::~SotobaNode() = default;
} // namespace sotoba_ros
