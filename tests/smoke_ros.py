"""Exercise a running stack in an isolated domain; this test commands robot motion."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, LaserScan

rclpy.init()
node = Node("mobile_integration_test")
latest = {}


def record(name):
    return lambda message: latest.update({name: message})


retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscriptions = [
    node.create_subscription(LaserScan, "/scan", record("scan"), qos_profile_sensor_data),
    node.create_subscription(Image, "/camera/image_raw", record("camera"), 1),
    node.create_subscription(Imu, "/imu/data", record("imu"), qos_profile_sensor_data),
    node.create_subscription(Odometry, "/odom", record("odom"), 10),
    node.create_subscription(Clock, "/clock", record("clock"), 10),
    node.create_subscription(OccupancyGrid, "/map", record("map"), retained),
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", record("local"), retained),
]
publisher = node.create_publisher(Twist, "/cmd_vel", 1)


def until(predicate, timeout=120):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError(f"Timed out; received topics: {sorted(latest)}")


def simulated_time():
    stamp = latest["clock"].clock
    return stamp.sec + stamp.nanosec * 1e-9


try:
    until(lambda: len(latest) == 7)
    assert len(latest["scan"].ranges) == 360
    assert latest["camera"].width == 320 and latest["camera"].height == 240
    assert min(latest["map"].data) <= 0 and max(latest["map"].data) == 100
    assert math.isclose(
        latest["local"].info.width * latest["local"].info.resolution, 3.0, abs_tol=1e-6
    )
    start_x = latest["odom"].pose.pose.position.x
    origin_x = latest["local"].info.origin.position.x
    target_time = simulated_time() + 2
    command = Twist()
    command.linear.x = 0.2
    deadline = time.monotonic() + 120
    while simulated_time() < target_time:
        assert time.monotonic() < deadline, "Simulation stalled during driving"
        publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.05)
    # Stop publishing; the policy must settle into standing after command expiry.
    stopped_time = simulated_time() + 2
    time.sleep(0.6)
    until(lambda: simulated_time() > stopped_time)
    assert latest["odom"].pose.pose.position.x > start_x + 0.1
    standing = latest["odom"].pose.pose.position
    standing_x, standing_y = standing.x, standing.y
    stopped_time = simulated_time() + 2
    until(lambda: simulated_time() > stopped_time)
    current = latest["odom"].pose.pose.position
    assert math.hypot(current.x - standing_x, current.y - standing_y) < 0.15
    assert latest["local"].info.origin.position.x > origin_x + 0.05
    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    assert client.wait_for_server(timeout_sec=10)
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = latest["clock"].clock
    goal.pose.pose.position.x = 0.8
    goal.pose.pose.orientation.w = 1.0
    sent = client.send_goal_async(goal)
    until(sent.done)
    handle = sent.result()
    assert handle.accepted
    result = handle.get_result_async()
    until(result.done, timeout=180)
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED, result.result()
    print(
        {
            "lidar_rays": 360,
            "camera": "320x240 RGB",
            "map_cells": len(latest["map"].data),
            "rolling_costmap": True,
            "watchdog_stopped": True,
            "navigation": "succeeded",
            "simulator_odom_x_m": round(latest["odom"].pose.pose.position.x, 3),
            "simulated_seconds": round(simulated_time(), 2),
        }
    )
finally:
    publisher.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()
