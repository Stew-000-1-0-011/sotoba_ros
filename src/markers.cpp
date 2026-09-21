/// @file markers.cpp
/// 形状 -> Marker の変換。
///
/// 曲面はオブジェクトローカル座標系で持っているので、
/// sotoba の apply_se3 でセンサ座標系へ写してから点を取り出す。
/// こうしておけば、曲面の内部表現の解釈はライブラリと1箇所で揃う。

#include "sotoba_ros/markers.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <sotoba/math/square_mat.hpp>
#include <sotoba/math/vec.hpp>

namespace sotoba_ros {
	namespace {
		using sotoba::math::SquareMat;
		using sotoba::math::UVec3;
		using sotoba::math::Vec3;
		using visualization_msgs::msg::Marker;

		auto to_point(const Vec3& v) -> geometry_msgs::msg::Point {
			geometry_msgs::msg::Point p{};
			p.x = static_cast<double>(v.x());
			p.y = static_cast<double>(v.y());
			p.z = static_cast<double>(v.z());
			return p;
		}

		void push_edge(Marker& marker, const Vec3& a, const Vec3& b) {
			marker.points.emplace_back(to_point(a));
			marker.points.emplace_back(to_point(b));
		}

		/// box のローカル座標 -> センサ座標系。
		/// rot は world -> local (行 i がローカル軸 i の world 表現) なので、戻すには転置。
		template <class Box_>
		auto box_local_to_world(const Box_& box, const Vec3& local) -> Vec3 {
			return box.center + box.rot.transpose() * local;
		}

		/// 存在する壁だけを枠線で描く。
		/// wall_exist のビット割り当ては sotoba 側と同じで、
		/// 軸 i の -hlens 面が (1 << 2i)、+hlens 面が (1 << 2i+1)。
		template <class Box_>
		void add_box_edges(Marker& marker, const Box_& box) {
			for (std::uint8_t axis = 0; axis < 3; ++axis) {
				const std::uint8_t u = (axis + 1) % 3;
				const std::uint8_t v = (axis + 2) % 3;

				for (std::uint8_t side = 0; side < 2; ++side) {
					const auto bit = std::uint32_t{1} << (2 * axis + side);
					if (!(box.wall_exist & bit)) { continue; }

					const float sign = side == 0 ? -1.f : 1.f;
					// 面の4隅をローカル座標で作る
					std::array<Vec3, 4> corners{};
					const std::array<std::pair<float, float>, 4> signs{
						{{-1.f, -1.f}, {1.f, -1.f}, {1.f, 1.f}, {-1.f, 1.f}}
					};
					for (std::uint8_t i = 0; i < 4; ++i) {
						Vec3 local{};
						local[axis] = sign * box.hlens[axis];
						local[u] = signs[i].first * box.hlens[u];
						local[v] = signs[i].second * box.hlens[v];
						corners[i] = box_local_to_world(box, local);
					}
					for (std::uint8_t i = 0; i < 4; ++i) {
						push_edge(marker, corners[i], corners[(i + 1) % 4]);
					}
				}
			}
		}

		void add_rectangle_edges(
			Marker& marker,
			const sotoba::surface::Rectangle& rect,
			const float normal_length
		) {
			const auto u = rect.u_axis_and_hlen.w() * rect.u_axis_and_hlen.xyz();
			const auto v = rect.v_axis_and_hlen.w() * rect.v_axis_and_hlen.xyz();

			const std::array<Vec3, 4> corners{
				rect.center - u - v,
				rect.center + u - v,
				rect.center + u + v,
				rect.center - u + v
			};
			for (std::uint8_t i = 0; i < 4; ++i) {
				push_edge(marker, corners[i], corners[(i + 1) % 4]);
			}

			if (normal_length > 0.f) {
				push_edge(
					marker,
					rect.center,
					rect.center + normal_length * Vec3{rect.normal}
				);
			}
		}

		/// +z を axis へ向ける回転。
		auto orientation_from_axis(const UVec3& axis) -> geometry_msgs::msg::Quaternion {
			geometry_msgs::msg::Quaternion q{};
			const float dot = axis.z(); // dot(+z, axis)
			if (dot > 1.f - 1e-6f) {
				q.w = 1.0;
				return q;
			}
			if (dot < -1.f + 1e-6f) {
				// 180度反転。x軸まわりに回せばよい
				q.x = 1.0;
				q.w = 0.0;
				return q;
			}
			// cross(+z, axis) = (-axis.y, axis.x, 0)
			const float cx = -axis.y();
			const float cy = axis.x();
			const float w = 1.f + dot;
			const float len = std::sqrt(cx * cx + cy * cy + w * w);
			q.x = static_cast<double>(cx / len);
			q.y = static_cast<double>(cy / len);
			q.z = 0.0;
			q.w = static_cast<double>(w / len);
			return q;
		}

		auto make_base_marker(
			const std::string& frame_id,
			const builtin_interfaces::msg::Time& stamp,
			const std::string& ns,
			const std::int32_t id,
			const std::int32_t type,
			const std::array<float, 4>& color,
			const float lifetime
		) -> Marker {
			Marker marker{};
			marker.header.frame_id = frame_id;
			marker.header.stamp = stamp;
			marker.ns = ns;
			marker.id = id;
			marker.type = type;
			marker.action = Marker::ADD;
			marker.pose.orientation.w = 1.0;
			marker.color.r = color[0];
			marker.color.g = color[1];
			marker.color.b = color[2];
			marker.color.a = color[3];
			// rclcpp に依存しないよう、Duration は直接組み立てる。
			const float clamped = lifetime > 0.f ? lifetime : 0.f;
			marker.lifetime.sec = static_cast<std::int32_t>(clamped);
			marker.lifetime.nanosec =
				static_cast<std::uint32_t>((clamped - std::floor(clamped)) * 1e9f);
			return marker;
		}
	} // namespace

	auto build_object_markers(
		std::span<const ObjectDef> objects,
		std::span<const sotoba::math::SE3> poses,
		std::span<const std::uint8_t> fresh,
		const std::string& frame_id,
		const builtin_interfaces::msg::Time& stamp,
		const MarkerStyle& style
	) -> visualization_msgs::msg::MarkerArray {
		visualization_msgs::msg::MarkerArray array{};
		if (poses.size() < objects.size() || fresh.size() < objects.size()) { return array; }

		for (std::size_t iobj = 0; iobj < objects.size(); ++iobj) {
			const auto& object = objects[iobj];
			const auto& pose = poses[iobj];
			const auto& color = fresh[iobj] != 0 ? style.color_fresh : style.color_stale;

			// 線で描ける曲面 (箱・長方形) は1つの LINE_LIST にまとめる。
			auto lines = make_base_marker(
				frame_id,
				stamp,
				object.name,
				0,
				Marker::LINE_LIST,
				color,
				style.lifetime
			);
			lines.scale.x = static_cast<double>(style.line_width);

			std::int32_t cylinder_id = 0;
			for (const auto& surface_variant : object.surfaces) {
				std::visit(
					[&](auto surface) {
						// センサ座標系へ
						surface.apply_se3(pose);

						using S = std::decay_t<decltype(surface)>;
						if constexpr (std::is_same_v<S, sotoba::surface::BoxInner>
									  || std::is_same_v<S, sotoba::surface::BoxOuter>) {
							add_box_edges(lines, surface);
						} else if constexpr (std::is_same_v<S, sotoba::surface::Rectangle>) {
							add_rectangle_edges(lines, surface, style.normal_length);
						} else if constexpr (std::is_same_v<S, sotoba::surface::CylinderOuter>) {
							auto cylinder = make_base_marker(
								frame_id,
								stamp,
								object.name + "/cylinder",
								cylinder_id++,
								Marker::CYLINDER,
								color,
								style.lifetime
							);
							cylinder.pose.position = to_point(surface.center);
							cylinder.pose.orientation = orientation_from_axis(surface.axis);
							cylinder.scale.x = static_cast<double>(2.f * surface.radius);
							cylinder.scale.y = static_cast<double>(2.f * surface.radius);
							cylinder.scale.z = static_cast<double>(2.f * surface.hheight);
							// 中身が見えるよう少し透ける
							cylinder.color.a *= 0.6f;
							array.markers.emplace_back(std::move(cylinder));
						}
					},
					surface_variant
				);
			}

			if (!lines.points.empty()) { array.markers.emplace_back(std::move(lines)); }

			if (style.show_labels) {
				auto label = make_base_marker(
					frame_id,
					stamp,
					object.name + "/label",
					0,
					Marker::TEXT_VIEW_FACING,
					color,
					style.lifetime
				);
				label.text = object.name;
				label.pose.position = to_point(pose.p);
				label.scale.z = 0.3;
				array.markers.emplace_back(std::move(label));
			}
		}

		return array;
	}
} // namespace sotoba_ros
