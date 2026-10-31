"""Check live sensor data, the SLAM map, and active navigation lifecycle nodes."""

import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2


def main() -> None:
    rclpy.init()
    node = Node("mobile_health")
    received = set()
    subscriptions = [
        node.create_subscription(
            PointCloud2,
            "/points",
            lambda m: received.add("points") if m.width > 0 and m.data else None,
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            LaserScan,
            "/scan",
            lambda m: received.add("scan") if len(m.ranges) == 360 else None,
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            Image,
            "/camera/image_raw",
            lambda m: received.add("camera") if m.width > 0 and m.data else None,
            1,
        ),
        node.create_subscription(
            OccupancyGrid,
            "/map",
            lambda m: received.add("map") if m.info.width > 0 and m.data else None,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        ),
    ]
    clients = [
        node.create_client(GetState, f"/{name}/get_state")
        for name in ("bt_navigator", "controller_server")
    ]
    futures = {}
    deadline = time.monotonic() + 12
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            for client in clients:
                if client not in futures and client.service_is_ready():
                    futures[client] = client.call_async(GetState.Request())
            active = len(futures) == len(clients) and all(
                future.done() and not future.exception() and future.result().current_state.id == 3
                for future in futures.values()
            )
            if received == {"scan", "points", "camera", "map"} and active:
                return
        raise SystemExit(
            f"Mobile stack not ready: received {sorted(received)}; check lifecycle logs"
        )
    finally:
        # Retain subscription handles until after spinning has finished.
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
