"""ROS 2 bridge from Nav2 Twist commands to the Pi UDP motor service.

Odometry is deliberately labeled and covaried as open-loop command integration.
It is a calibration/bootstrap source, not a substitute for encoder or visual
metric feedback.
"""

from __future__ import annotations

import json
import math
import secrets
import socket
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .base_kinematics import (
    DriveCalibration,
    calibrated_wheel_command,
    integrate_pose,
    wheels_to_twist,
)


def yaw_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw_rad * 0.5), math.cos(yaw_rad * 0.5)


def rpy_quaternion(
    roll_rad: float, pitch_rad: float, yaw_rad: float
) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class HouseBotBase(Node):
    def __init__(self) -> None:
        super().__init__("house_bot_base")
        self.declare_parameter("calibrated", False)
        self.declare_parameter("proportional_control_verified", False)
        self.declare_parameter("pi_host", "192.168.0.241")
        self.declare_parameter("pi_port", 8765)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.25)
        self.declare_parameter("ack_timeout_s", 0.50)
        self.declare_parameter("motor_duty_deadband", 0.05)
        self.declare_parameter("wheel_separation_m", 0.0)
        self.declare_parameter("left_forward_mps", 0.0)
        self.declare_parameter("left_reverse_mps", 0.0)
        self.declare_parameter("right_forward_mps", 0.0)
        self.declare_parameter("right_reverse_mps", 0.0)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("camera_optical_frame", "camera_optical_frame")
        self.declare_parameter("camera_x_m", 0.0)
        self.declare_parameter("camera_y_m", 0.0)
        self.declare_parameter("camera_z_m", 0.0)
        self.declare_parameter("camera_roll_rad", 0.0)
        self.declare_parameter("camera_pitch_rad", 0.0)
        self.declare_parameter("camera_yaw_rad", 0.0)

        if not bool(self.get_parameter("calibrated").value):
            raise RuntimeError(
                "base calibration is not approved; generate config/local/base_calibration.yaml"
            )
        self.calibration = DriveCalibration(
            wheel_separation_m=float(self.get_parameter("wheel_separation_m").value),
            left_forward_mps=float(self.get_parameter("left_forward_mps").value),
            left_reverse_mps=float(self.get_parameter("left_reverse_mps").value),
            right_forward_mps=float(self.get_parameter("right_forward_mps").value),
            right_reverse_mps=float(self.get_parameter("right_reverse_mps").value),
        )
        self.calibration.validate()
        self.proportional_control_verified = bool(
            self.get_parameter("proportional_control_verified").value
        )

        pi_host = str(self.get_parameter("pi_host").value)
        self.pi_address = (
            socket.gethostbyname(pi_host),
            int(self.get_parameter("pi_port").value),
        )
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.ack_timeout_s = float(self.get_parameter("ack_timeout_s").value)
        self.motor_duty_deadband = float(
            self.get_parameter("motor_duty_deadband").value
        )
        if not 10.0 <= self.control_rate_hz <= 50.0:
            raise ValueError("control_rate_hz must be between 10 and 50")
        if not 0.10 <= self.command_timeout_s < 0.35:
            raise ValueError("command_timeout_s must be at least 0.10 and below Pi watchdog")
        if not 0.0 <= self.motor_duty_deadband <= 0.25:
            raise ValueError("motor_duty_deadband must be between 0 and 0.25")

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.camera_optical_frame = str(self.get_parameter("camera_optical_frame").value)
        self.session = secrets.token_hex(8)
        self.sequence = 0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.enabled = False
        self.estopped = False
        self.enabled_at: float | None = None
        self.last_command_at: float | None = None
        self.last_ack_at: float | None = None
        self.requested_linear = 0.0
        self.requested_angular = 0.0
        self.applied_left = 0.0
        self.applied_right = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_update_at = time.monotonic()
        self.last_status_at = 0.0

        self.odom_publisher = self.create_publisher(Odometry, "/odom", 20)
        self.status_publisher = self.create_publisher(String, "/house_bot/base/status", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Bool, "/house_bot/estop", self.on_estop, 10)
        self.create_service(SetBool, "/house_bot/base/enable", self.on_enable)
        self.publish_camera_transform()
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_tick)
        self.get_logger().warn(
            "Base is DISABLED and odometry is open-loop. Call /house_bot/base/enable to arm."
        )

    def on_cmd_vel(self, message: Twist) -> None:
        if not math.isfinite(message.linear.x) or not math.isfinite(message.angular.z):
            self.enabled = False
            self.enabled_at = None
            self.clear_command()
            self.get_logger().error("Non-finite Twist rejected; base disabled")
            return
        if abs(message.linear.y) > 1e-6 or abs(message.linear.z) > 1e-6:
            self.get_logger().warn("Ignoring non-differential linear Twist components")
        self.requested_linear = float(message.linear.x)
        self.requested_angular = float(message.angular.z)
        self.last_command_at = time.monotonic()

    def on_estop(self, message: Bool) -> None:
        if message.data:
            self.estopped = True
            self.enabled = False
            self.enabled_at = None
            self.clear_command()
            self.get_logger().error("Emergency stop asserted; base disabled")

    def on_enable(self, request: SetBool.Request, response: SetBool.Response):
        if request.data and not self.proportional_control_verified:
            response.success = False
            response.message = (
                "base cannot be armed: proportional two-tread control is not verified"
            )
            return response
        if request.data and self.estopped:
            response.success = False
            response.message = "estop is latched; restart the base driver after checking the robot"
            return response
        self.enabled = bool(request.data)
        self.enabled_at = time.monotonic() if self.enabled else None
        self.clear_command()
        response.success = True
        response.message = "base enabled; waiting for a fresh Twist" if self.enabled else "base disabled"
        self.get_logger().warn(response.message)
        return response

    def clear_command(self) -> None:
        self.requested_linear = 0.0
        self.requested_angular = 0.0
        self.last_command_at = None
        self.applied_left = 0.0
        self.applied_right = 0.0

    def publish_camera_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x = float(self.get_parameter("camera_x_m").value)
        transform.transform.translation.y = float(self.get_parameter("camera_y_m").value)
        transform.transform.translation.z = float(self.get_parameter("camera_z_m").value)
        quaternion = rpy_quaternion(
            float(self.get_parameter("camera_roll_rad").value),
            float(self.get_parameter("camera_pitch_rad").value),
            float(self.get_parameter("camera_yaw_rad").value),
        )
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = quaternion
        optical = TransformStamped()
        optical.header.stamp = transform.header.stamp
        optical.header.frame_id = self.camera_frame
        optical.child_frame_id = self.camera_optical_frame
        optical_quaternion = rpy_quaternion(-math.pi * 0.5, 0.0, -math.pi * 0.5)
        (
            optical.transform.rotation.x,
            optical.transform.rotation.y,
            optical.transform.rotation.z,
            optical.transform.rotation.w,
        ) = optical_quaternion
        self.static_tf_broadcaster.sendTransform([transform, optical])

    def drain_acknowledgements(self, now: float) -> None:
        while True:
            try:
                payload, address = self.socket.recvfrom(4096)
            except BlockingIOError:
                return
            if address[0] != self.pi_address[0]:
                continue
            try:
                response = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if response.get("ok") and response.get("session") == self.session:
                self.last_ack_at = now

    def send_wheels(self, left_duty: float, right_duty: float) -> None:
        payload = json.dumps(
            {
                "session": self.session,
                "sequence": self.sequence,
                "left": left_duty,
                "right": right_duty,
            },
            separators=(",", ":"),
        ).encode("utf-8")
        self.socket.sendto(payload, self.pi_address)
        self.sequence += 1

    def control_tick(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self.last_update_at, 0.0), 0.2)
        self.last_update_at = now
        self.x, self.y, self.yaw = integrate_pose(
            self.x,
            self.y,
            self.yaw,
            self.applied_left,
            self.applied_right,
            self.calibration.wheel_separation_m,
            dt,
        )

        fresh = (
            self.last_command_at is not None
            and now - self.last_command_at <= self.command_timeout_s
        )
        if self.enabled and fresh and not self.estopped:
            left_duty, right_duty, self.applied_left, self.applied_right = (
                calibrated_wheel_command(
                    self.requested_linear,
                    self.requested_angular,
                    self.calibration,
                    self.motor_duty_deadband,
                )
            )
        else:
            left_duty = right_duty = 0.0
            self.applied_left = self.applied_right = 0.0

        self.send_wheels(left_duty, right_duty)
        self.drain_acknowledgements(now)
        ack_reference = self.last_ack_at
        if self.enabled_at is not None:
            ack_reference = max(ack_reference or self.enabled_at, self.enabled_at)
        if self.enabled and ack_reference is not None and now - ack_reference > self.ack_timeout_s:
            self.enabled = False
            self.enabled_at = None
            self.clear_command()
            left_duty = right_duty = 0.0
            self.send_wheels(0.0, 0.0)
            self.get_logger().error("Motor acknowledgements timed out; base disabled")

        self.publish_odometry()
        if now - self.last_status_at >= 0.5:
            self.publish_status(now, left_duty, right_duty, fresh)
            self.last_status_at = now

    def publish_odometry(self) -> None:
        stamp = self.get_clock().now().to_msg()
        quaternion = yaw_quaternion(self.yaw)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(transform)

        linear, angular = wheels_to_twist(
            self.applied_left,
            self.applied_right,
            self.calibration.wheel_separation_m,
        )
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        # Command integration can drift quickly. Large covariance prevents a
        # fusion node from treating this bootstrap odometry like an encoder.
        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[35] = 0.20
        odom.pose.covariance[14] = 999.0
        odom.pose.covariance[21] = 999.0
        odom.pose.covariance[28] = 999.0
        odom.twist.covariance[0] = 0.10
        odom.twist.covariance[7] = 999.0
        odom.twist.covariance[14] = 999.0
        odom.twist.covariance[21] = 999.0
        odom.twist.covariance[28] = 999.0
        odom.twist.covariance[35] = 0.10
        self.odom_publisher.publish(odom)

    def publish_status(
        self, now: float, left_duty: float, right_duty: float, fresh: bool
    ) -> None:
        message = String()
        message.data = json.dumps(
            {
                "enabled": self.enabled,
                "estopped": self.estopped,
                "command_fresh": fresh,
                "last_ack_age_s": None if self.last_ack_at is None else now - self.last_ack_at,
                "left_duty": left_duty,
                "right_duty": right_duty,
                "odometry": "open_loop_command_integration",
            },
            separators=(",", ":"),
        )
        self.status_publisher.publish(message)

    def stop_motors(self) -> None:
        for _attempt in range(3):
            try:
                self.send_wheels(0.0, 0.0)
            except OSError:
                pass
            time.sleep(0.02)

    def destroy_node(self) -> bool:
        self.stop_motors()
        self.socket.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = HouseBotBase()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
