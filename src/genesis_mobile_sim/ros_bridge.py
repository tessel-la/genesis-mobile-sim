"""Publish Go2 sensors, joints, and explicitly ideal simulator odometry over ROS 2."""

import json
import math
import os
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .lidar import (
    MAX_RANGE,
    MIN_RANGE,
    cloud_bytes,
    scan_ranges,
)
from .mesh_server import serve_meshes
from .robot_model import BASE_FRAME
from .timing import next_tick
from .world import CAMERA_FOV, CAMERA_HEIGHT, CAMERA_WIDTH, DT, World

SCAN_STEPS, CLOUD_STEPS, CAMERA_STEPS = 5, 10, 10


class Simulator(Node):
    def __init__(self) -> None:
        super().__init__("genesis_mobile_sim")
        self.status_pub = self.create_publisher(String, "/go2/status", 1)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 1)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 1)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, "/points", qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", qos_profile_sensor_data)
        # Robo-Boy's web_video_server subscribes reliably.
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 1)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", 1)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", 1)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 1)
        self.tf = TransformBroadcaster(self, qos=QoSProfile(depth=1))
        self.world = World(os.environ.get("GENESIS_DEVICE", "cpu"))
        self.get_logger().info(
            f"Genesis backend: {self.world.gs.backend}; device: {self.world.gs.device}"
        )
        self.command_sub = self.create_subscription(Twist, "/cmd_vel", self.command, 1)
        self.motion_sub = self.create_subscription(String, "/go2/motion_cmd", self.motion, 1)

    def command(self, message: Twist) -> None:
        try:
            self.world.command(
                message.linear.x, message.linear.y, message.angular.z, time.monotonic()
            )
        except ValueError as error:
            self.get_logger().warning(str(error))

    def motion(self, message: String) -> None:
        try:
            self.world.request_motion(message.data)
        except ValueError as error:
            self.get_logger().warning(str(error))

    def stamp(self) -> Time:
        # Integer steps keep /clock and every sensor timestamp on the same timeline.
        nanoseconds = self.world.steps * round(DT * 1_000_000_000)
        return Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)

    def publish_state(self) -> None:
        world, stamp = self.world, self.stamp()
        position = world.array(world.base.get_pos())
        quaternion = world.array(world.base.get_quat())
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, "odom"
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(
            float, position
        )
        orientation = odom.pose.pose.orientation
        orientation.w, orientation.x, orientation.y, orientation.z = map(float, quaternion)
        from genesis.utils.geom import inv_quat, transform_by_quat

        body_velocity = world.array(
            transform_by_quat(world.base.get_vel(), inv_quat(world.base.get_quat()))
        )
        reading = world.imu.read()
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(
            float, body_velocity
        )
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(
            float, world.array(reading.ang_vel)
        )
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = map(float, position)
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = list(world.joint_names)
        joints.position = list(map(float, world.array(world.robot.get_dofs_position(world.motors))))
        self.joint_pub.publish(joints)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, BASE_FRAME
        imu.orientation_covariance[0] = -1.0
        for output, values in (
            (imu.linear_acceleration, world.array(reading.lin_acc)),
            (imu.angular_velocity, world.array(reading.ang_vel)),
        ):
            output.x, output.y, output.z = (float(value) for value in values)
        self.imu_pub.publish(imu)

    def publish_status(self) -> None:
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "mode": self.world.motion.mode,
                        "reason": self.world.motion.reason,
                        "backend": str(self.world.gs.device),
                        "environment": "warehouse",
                        "policies": ["walk", "stand", "recover"],
                    }
                )
            )
        )

    def publish_lidar(self) -> None:
        # A tilted planar scan is not a floor-plan measurement. The 3D cloud is
        # still valid during recovery because TF carries the actual sensor pose.
        planar = self.world.scan_valid
        cloud = self.world.steps % CLOUD_STEPS == 0 and self.cloud_pub.get_subscription_count() > 0
        if not planar and not cloud:
            return
        stamp = self.stamp()
        distances = self.world.array(self.world.lidar.read().distances)
        if planar:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, "laser"
            scan.angle_min, scan.angle_max = -math.pi, math.radians(179)
            scan.angle_increment, scan.scan_time = math.pi / 180, SCAN_STEPS * DT
            scan.range_min, scan.range_max = MIN_RANGE, MAX_RANGE
            scan.ranges = scan_ranges(distances)
            self.scan_pub.publish(scan)
        if cloud:
            data = cloud_bytes(distances, self.world.lidar_directions)
            points = PointCloud2()
            points.header.stamp, points.header.frame_id = stamp, "laser"
            points.height, points.width = 1, len(data) // 12
            points.fields = [
                PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                for name, offset in (("x", 0), ("y", 4), ("z", 8))
            ]
            points.is_bigendian, points.is_dense = False, True
            points.point_step, points.row_step = 12, len(data)
            points.data = data
            self.cloud_pub.publish(points)

    def publish_camera(self) -> None:
        # Calibration is cheap and remains available without an image viewer.
        stamp = self.stamp()
        color_requested = self.image_pub.get_subscription_count() > 0
        depth_requested = self.depth_pub.get_subscription_count() > 0
        if color_requested or depth_requested:
            rgb, depth = self.world.read_camera(rgb=color_requested, depth=depth_requested)
        if color_requested:
            image = Image()
            image.header.stamp, image.header.frame_id = stamp, "camera_optical_frame"
            image.height, image.width = CAMERA_HEIGHT, CAMERA_WIDTH
            image.encoding, image.step, image.data = (
                "rgb8",
                CAMERA_WIDTH * 3,
                rgb.astype("uint8", copy=False).tobytes(),
            )
            self.image_pub.publish(image)
        if depth_requested:
            image = Image()
            image.header.stamp, image.header.frame_id = stamp, "camera_optical_frame"
            image.height, image.width = CAMERA_HEIGHT, CAMERA_WIDTH
            image.encoding, image.is_bigendian = "32FC1", False
            image.step, image.data = CAMERA_WIDTH * 4, depth.astype("<f4", copy=False).tobytes()
            self.depth_pub.publish(image)
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, "camera_optical_frame"
        info.width, info.height = CAMERA_WIDTH, CAMERA_HEIGHT
        focal = CAMERA_HEIGHT / (2 * math.tan(math.radians(CAMERA_FOV) / 2))
        # Genesis samples pixel centres at (u + 0.5, v + 0.5); ROS indexes centres at integers.
        cx, cy = (CAMERA_WIDTH - 1) / 2, (CAMERA_HEIGHT - 1) / 2
        info.distortion_model, info.d = "plumb_bob", [0.0] * 5
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)
        self.depth_info_pub.publish(info)


def main() -> None:
    rclpy.init()
    node = None
    try:
        with serve_meshes():
            node = Simulator()
            deadline = time.monotonic()
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0)
                node.world.step(time.monotonic())
                node.clock_pub.publish(Clock(clock=node.stamp()))
                node.publish_state()
                if node.world.steps % SCAN_STEPS == 0:
                    node.publish_lidar()
                if node.world.steps % CAMERA_STEPS == CAMERA_STEPS // 2:
                    node.publish_camera()
                if node.world.steps % CLOUD_STEPS == 0:
                    node.publish_status()
                deadline, delay = next_tick(deadline, time.monotonic(), DT)
                time.sleep(delay)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.world.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
