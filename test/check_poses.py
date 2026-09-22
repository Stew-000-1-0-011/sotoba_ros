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
from sotoba_ros.msg import BeliefArray
from visualization_msgs.msg import MarkerArray

STATUS_NAMES = {0: "not_run", 1: "updated", 2: "too_few", 3: "solve_failed"}

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
        # 全オブジェクトぶん名前付きで出る信念分布を見る
        # (~/object_poses は更新できたものしか出ないので突き合わせに使えない)
        self.create_subscription(
            BeliefArray, "/sotoba_node/posterior_beliefs", self.on_estimated, 10
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
    elif len(node.truth.poses) != len(node.estimated.means):
        print(
            f"NG: {len(node.estimated.means)} beliefs vs {len(node.truth.poses)} truth poses"
        )
        ok = False
    else:
        est = node.estimated
        if est.header.frame_id != node.truth.header.frame_id:
            print(
                f"NG: frame_id mismatch: {est.header.frame_id!r} "
                f"vs {node.truth.header.frame_id!r}"
            )
            ok = False

        updated = 0
        wrong = 0
        for i, (t, e) in enumerate(zip(node.truth.poses, est.means)):
            err = math.dist(
                (t.position.x, t.position.y, t.position.z),
                (e.position.x, e.position.y, e.position.z),
            )
            yaw_err = abs(yaw_of(t.orientation) - yaw_of(e.orientation))
            status = est.status[i] if i < len(est.status) else 1
            name = est.names[i] if i < len(est.names) else f"#{i}"
            good = err <= POSITION_TOLERANCE and yaw_err <= YAW_TOLERANCE
            if status == 1:
                updated += 1
                if not good:
                    wrong += 1
            tag = "ok" if (status == 1 and good) else ("NG" if status == 1 else "--")
            print(
                f"[{tag}] {name}: {STATUS_NAMES.get(status, status)} "
                f"pos_err={err:.4f} yaw_err={yaw_err:.4f}"
            )

        print(f"updated {updated} / {len(node.truth.poses)}, wrong {wrong}")
        # publish された姿勢が全部正しく、フィールド (先頭) が取れていること
        if wrong != 0 or updated == 0:
            ok = False
        if est.status and est.status[0] != 1:
            print("NG: the first object (field) was not updated")
            ok = False

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
