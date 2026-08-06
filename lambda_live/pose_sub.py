import json
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

rclpy.init()
node = Node("umi_teleop_pose_tap")

target = None
deadline = time.time() + 120
while target is None and time.time() < deadline:
    candidates = [n for n, t in node.get_topic_names_and_types() if "nav_msgs/msg/Odometry" in t]
    if candidates:
        candidates.sort(key=lambda n: ("corrected" not in n, "scanend" in n, n))
        target = candidates[0]
    if target is None:
        rclpy.spin_once(node, timeout_sec=0.5)
if target is None:
    print(json.dumps({"error": "no odometry topic found"}), flush=True)
    sys.exit(1)
print(json.dumps({"info": f"subscribed {target}"}), flush=True)


def cb(m: Odometry):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    print(
        json.dumps(
            {
                "t": m.header.stamp.sec + m.header.stamp.nanosec / 1e9,
                "x": round(p.x, 4),
                "y": round(p.y, 4),
                "z": round(p.z, 4),
                "qx": round(q.x, 4),
                "qy": round(q.y, 4),
                "qz": round(q.z, 4),
                "qw": round(q.w, 4),
            }
        ),
        flush=True,
    )


node.create_subscription(Odometry, target, cb, 10)
rclpy.spin(node)
