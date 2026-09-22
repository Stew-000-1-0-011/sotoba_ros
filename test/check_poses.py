"""fake_scan_publisher の真値と sotoba_node の推定を突き合わせる手動テスト。

    ros2 launch sotoba_ros sotoba_node.launch.py fake_scan:=true
    python3 test/check_poses.py

objects.cpp の中身には依存しない (真値は fake_scan_publisher が publish する)。
colcon test では走らない (ノードを別に起動する必要があるため)。
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

# 許容誤差
POSITION_TOLERANCE = 0.03  # [m]
YAW_TOLERANCE = 0.02  # [rad]
WATCH_SECONDS = 8.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Checker(Node):
    def __init__(self):
        super().__init__("pose_checker")
        self.truth = None
        self.estimated = None
        self.markers = None
        self.create_subscription(
            PoseArray, "/fake_scan_publisher/truth_poses", self.on_truth, 10
        )
        self.create_subscription(
            PoseArray, "/sotoba_node/object_poses", self.on_estimated, 10
        )
        self.create_subscription(
            MarkerArray, "/sotoba_node/object_markers", self.on_markers, 10
        )

    def on_truth(self, msg):
        self.truth = msg

    def on_estimated(self, msg):
        self.estimated = msg

    def on_markers(self, msg):
        self.markers = msg


def main() -> int:
    rclpy.init()
    node = Checker()
    end = node.get_clock().now().nanoseconds + WATCH_SECONDS * 1e9
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    ok = True
    if node.truth is None:
        print("NG: no truth poses (is fake_scan_publisher running?)")
        ok = False
    elif node.estimated is None:
        print("NG: no estimated poses (is sotoba_node running?)")
        ok = False
    elif len(node.truth.poses) != len(node.estimated.poses):
        print(
            f"NG: {len(node.estimated.poses)} of {len(node.truth.poses)} object(s) "
            "were published (some failed to update)"
        )
        ok = False
    else:
        if node.estimated.header.frame_id != node.truth.header.frame_id:
            print(
                f"NG: frame_id mismatch: {node.estimated.header.frame_id!r} "
                f"vs {node.truth.header.frame_id!r}"
            )
            ok = False
        for i, (t, e) in enumerate(zip(node.truth.poses, node.estimated.poses)):
            err = math.dist(
                (t.position.x, t.position.y, t.position.z),
                (e.position.x, e.position.y, e.position.z),
            )
            yaw_err = abs(yaw_of(t.orientation) - yaw_of(e.orientation))
            good = err <= POSITION_TOLERANCE and yaw_err <= YAW_TOLERANCE
            ok = ok and good
            print(
                f"[{'ok' if good else 'NG'}] object {i}: "
                f"est=({e.position.x:.3f}, {e.position.y:.3f}, {e.position.z:.3f}) "
                f"truth=({t.position.x:.3f}, {t.position.y:.3f}, {t.position.z:.3f}) "
                f"pos_err={err:.4f} yaw_err={yaw_err:.4f}"
            )

    if node.markers is None:
        print("NG: no markers published")
        ok = False
    else:
        namespaces = sorted({m.ns for m in node.markers.markers})
        points = sum(len(m.points) for m in node.markers.markers)
        print(f"markers: {len(node.markers.markers)} in {namespaces}, {points} line points")
        if not node.markers.markers:
            ok = False

    node.destroy_node()
    rclpy.shutdown()
    print("RUNTIME OK" if ok else "RUNTIME FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
