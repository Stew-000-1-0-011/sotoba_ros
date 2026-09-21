"""合成 LaserScan を流して、sotoba_node の出す Pose を検証する手動テスト。

objects.cpp の既定のオブジェクト (field + pole) を前提に、フィールド内の
既知の姿勢に置いたロボットから見た2Dスキャンを作って /scan へ流し、
publish された Pose からロボット姿勢を復元して真値と比べる。

    # 別端末で
    ros2 run sotoba_ros sotoba_node
    # こちらで
    python3 test/scan_sim.py

colcon test では走らない (ノードを別に起動する必要があるため)。
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

# フィールド: 原点中心 12m x 12m (half 6)。ポール: (-1, 1) 半径 0.15
HALF = 6.0
# シードは laser 系で (2.0, 1.0)。真値をその 0.05m 奥に置く
# (円柱は「シードより手前」にあると可視判定で対応点が消えるため)
POLE = (-0.7026, 1.2512, 0.15)
# 真のロボット姿勢 (フィールド座標系): 位置と yaw
TRUE_XY = (-2.7, 0.15)
TRUE_YAW = 0.05
N = 720


def cast(px, py, a):
    dx, dy = math.cos(a), math.sin(a)
    best = float("inf")
    for sign in (1.0, -1.0):
        if abs(dx) > 1e-9:
            t = (sign * HALF - px) / dx
            if t > 0 and abs(py + t * dy) <= HALF:
                best = min(best, t)
        if abs(dy) > 1e-9:
            t = (sign * HALF - py) / dy
            if t > 0 and abs(px + t * dx) <= HALF:
                best = min(best, t)
    # ポール (円)
    cx, cy, r = POLE
    ox, oy = px - cx, py - cy
    b = 2 * (ox * dx + oy * dy)
    c = ox * ox + oy * oy - r * r
    disc = b * b - 4 * c
    if disc >= 0:
        for t in ((-b - math.sqrt(disc)) / 2, (-b + math.sqrt(disc)) / 2):
            if t > 0:
                best = min(best, t)
    return best


class Sim(Node):
    def __init__(self):
        super().__init__("scan_sim")
        self.pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "/sotoba_node/objects/field/pose", self.on_field, 10
        )
        self.create_subscription(
            PoseStamped, "/sotoba_node/objects/pole/pose", self.on_pole, 10
        )
        self.create_subscription(
            MarkerArray, "/sotoba_node/object_markers", self.on_markers, 10
        )
        self.create_timer(0.1, self.tick)
        self.field_msgs = []
        self.pole_msgs = []
        self.markers = None

    def tick(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2 * math.pi / N
        msg.range_min = 0.05
        msg.range_max = 30.0
        px, py = TRUE_XY
        msg.ranges = [
            float(cast(px, py, TRUE_YAW + msg.angle_min + msg.angle_increment * i))
            for i in range(N)
        ]
        self.pub.publish(msg)

    def on_field(self, msg):
        self.field_msgs.append(msg)

    def on_pole(self, msg):
        self.pole_msgs.append(msg)

    def on_markers(self, msg):
        self.markers = msg


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = Sim()
    end = node.get_clock().now().nanoseconds + 8e9
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    ok = True
    print(f"field poses: {len(node.field_msgs)}, pole poses: {len(node.pole_msgs)}")
    if not node.field_msgs:
        print("NG: no field pose published")
        ok = False
    else:
        p = node.field_msgs[-1]
        # publish されるのは field -> laser。逆に取るとフィールド座標系でのロボット姿勢。
        th = yaw_of(p.pose.orientation)
        x, y = p.pose.position.x, p.pose.position.y
        rx = -(math.cos(-th) * x - math.sin(-th) * y)
        ry = -(math.sin(-th) * x + math.cos(-th) * y)
        ryaw = -th
        err = math.hypot(rx - TRUE_XY[0], ry - TRUE_XY[1])
        yerr = abs(ryaw - TRUE_YAW)
        print(f"frame_id={p.header.frame_id!r} z={p.pose.position.z:.4f}")
        print(
            f"robot (field): est=({rx:.3f}, {ry:.3f}, yaw {ryaw:.4f}) "
            f"truth=({TRUE_XY[0]}, {TRUE_XY[1]}, yaw {TRUE_YAW}) "
            f"pos_err={err:.4f} yaw_err={yerr:.4f}"
        )
        if err > 0.05 or yerr > 0.02 or p.header.frame_id != "laser":
            ok = False
        if abs(p.pose.position.z) > 1e-3:
            print("NG: z drifted")
            ok = False
    th, (tx, ty) = TRUE_YAW, TRUE_XY
    ex = math.cos(-th) * (POLE[0] - tx) - math.sin(-th) * (POLE[1] - ty)
    ey = math.sin(-th) * (POLE[0] - tx) + math.cos(-th) * (POLE[1] - ty)
    if not node.pole_msgs:
        print("NG: no pole pose published")
        ok = False
    else:
        p = node.pole_msgs[-1].pose.position
        err = math.hypot(p.x - ex, p.y - ey)
        print(f"pole (laser): est=({p.x:.3f}, {p.y:.3f}) truth=({ex:.3f}, {ey:.3f}) err={err:.4f}")
        if err > 0.1:
            ok = False

    # --- マーカー ---
    if node.markers is None:
        print("NG: no markers published")
        ok = False
    else:
        names = sorted({m.ns for m in node.markers.markers})
        lines = [m for m in node.markers.markers if m.ns == "field"]
        cyl = [m for m in node.markers.markers if m.ns == "pole/cylinder"]
        print(f"marker namespaces: {names}")
        if not lines or not cyl:
            print("NG: missing field wireframe or pole cylinder marker")
            ok = False
        else:
            pts = lines[0].points
            # laser系 -> field系 に戻して、全点が壁の上 (|x|=6 か |y|=6) に乗るか
            th, (tx, ty) = TRUE_YAW, TRUE_XY
            bad = 0
            for q in pts:
                fx = math.cos(th) * q.x - math.sin(th) * q.y + tx
                fy = math.sin(th) * q.x + math.cos(th) * q.y + ty
                on_wall = abs(abs(fx) - HALF) < 1e-2 or abs(abs(fy) - HALF) < 1e-2
                if not on_wall or abs(abs(q.z) - 1.0) > 1e-2:
                    bad += 1
            print(f"field wireframe: {len(pts)} points, off-wall points: {bad}")
            # 天井/床を消しているので、壁4枚 x 4辺 x 2点 = 32点のはず
            if bad or len(pts) != 32:
                ok = False
            c = cyl[0].pose.position
            derr = math.hypot(c.x - ex, c.y - ey)
            print(
                f"pole cylinder marker: pos=({c.x:.3f}, {c.y:.3f}) "
                f"scale=({cyl[0].scale.x:.2f}, {cyl[0].scale.z:.2f}) err={derr:.4f}"
            )
            if derr > 0.1 or abs(cyl[0].scale.x - 0.3) > 1e-3 or abs(cyl[0].scale.z - 1.0) > 1e-3:
                ok = False

    node.destroy_node()
    rclpy.shutdown()
    print("RUNTIME OK" if ok else "RUNTIME FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
