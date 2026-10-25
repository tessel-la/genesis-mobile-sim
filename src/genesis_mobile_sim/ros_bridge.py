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
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .mesh_server import serve_meshes
from .robot_model import BASE_FRAME
from .world import CAMERA_FOV, CAMERA_HEIGHT, CAMERA_WIDTH, DT, World


class Simulator(Node):
    def __init__(self) -> None:
        super().__init__("genesis_mobile_sim")
        self.status_pub = self.create_publisher(String, "/go2/status", 1)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", qos_profile_sensor_data)
        # Robo-Boy's web_video_server subscribes reliably.
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 1)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 1)
        self.tf = TransformBroadcaster(self)
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

    def publish_sensors(self) -> None:
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
        # A tilted planar scan is not a floor-plan measurement. Preserve mapping
        # continuity by pausing it during falls/recovery; camera and odometry stay live.
        if self.world.scan_valid:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = self.stamp(), "laser"
            scan.angle_min, scan.angle_max = -math.pi, math.radians(179)
            scan.angle_increment, scan.scan_time = math.pi / 180, 0.2
            scan.range_min, scan.range_max = 0.08, 10.0
            distances = self.world.array(self.world.lidar.read().distances).reshape(-1)
            scan.ranges = [float(d) if 0.08 <= d < 10 else math.inf for d in distances]
            self.scan_pub.publish(scan)
        image = Image()
        image.header.stamp, image.header.frame_id = self.stamp(), "camera_optical_frame"
        rgb = self.world.array(self.world.camera.read().rgb).astype("uint8", copy=False)
        image.height, image.width = CAMERA_HEIGHT, CAMERA_WIDTH
        image.encoding, image.step, image.data = "rgb8", CAMERA_WIDTH * 3, rgb.tobytes()
        self.image_pub.publish(image)
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = image.width, image.height
        focal = CAMERA_HEIGHT / (2 * math.tan(math.radians(CAMERA_FOV) / 2))
        cx, cy = CAMERA_WIDTH / 2, CAMERA_HEIGHT / 2
        info.distortion_model, info.d = "plumb_bob", [0.0] * 5
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)


def main() -> None:
    rclpy.init()
    node = None
    try:
        with serve_meshes():
            node = Simulator()
            while rclpy.ok():
                started = time.monotonic()
                rclpy.spin_once(node, timeout_sec=0)
                node.world.step(time.monotonic())
                node.clock_pub.publish(Clock(clock=node.stamp()))
                if node.world.steps % 2 == 0:
                    node.publish_state()
                if node.world.steps % 10 == 0:
                    node.publish_sensors()
                time.sleep(max(0, DT - (time.monotonic() - started)))
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
