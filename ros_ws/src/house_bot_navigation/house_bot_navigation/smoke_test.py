"""End-to-end check for the loopback navigation stack."""

import json
import math
import sys
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SmokeTest(Node):
    def __init__(self) -> None:
        super().__init__("house_bot_navigation_smoke_test")
        self.state = ""
        self.detail = ""
        self.x = math.nan
        self.y = math.nan
        status_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, "/house_bot/navigation_status", self._status_callback, status_qos
        )
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.client = self.create_client(Trigger, "/house_bot/go/kitchen")

    def _status_callback(self, message: String) -> None:
        payload = json.loads(message.data)
        self.state = str(payload.get("state", ""))
        self.detail = str(payload.get("detail", ""))
        self.get_logger().info(
            f"navigation state={self.state} detail={self.detail!r}"
        )

    def _odom_callback(self, message: Odometry) -> None:
        self.x = message.pose.pose.position.x
        self.y = message.pose.pose.position.y


def _spin_until(node: SmokeTest, predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SmokeTest()
    try:
        if not node.client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("Named Kitchen service did not become available")

        # The service can appear before the lifecycle-managed Nav2 action does.
        accepted = False
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not accepted:
            future = node.client.call_async(Trigger.Request())
            if not _spin_until(node, future.done, 5.0):
                raise RuntimeError("Kitchen service call timed out")
            response = future.result()
            accepted = bool(response.success)
            if not accepted:
                node.get_logger().info(response.message)
                time.sleep(0.5)
        if not accepted:
            raise RuntimeError("Nav2 action did not become ready")

        terminal = {"succeeded", "aborted", "canceled", "failed", "error"}
        if not _spin_until(node, lambda: node.state in terminal, 60.0):
            raise RuntimeError(
                f"Navigation timed out at pose ({node.x:.3f}, {node.y:.3f})"
            )
        if node.state != "succeeded":
            raise RuntimeError(f"Navigation ended as {node.state}: {node.detail}")

        error = math.hypot(node.x - 2.6, node.y)
        if error > 0.30:
            raise RuntimeError(
                f"Reported success {error:.3f} m from Kitchen at "
                f"({node.x:.3f}, {node.y:.3f})"
            )
        node.get_logger().info(
            f"PASS: reached Kitchen at ({node.x:.3f}, {node.y:.3f}), "
            f"error={error:.3f} m"
        )
    except Exception as exc:
        node.get_logger().error(str(exc))
        rclpy.shutdown()
        sys.exit(1)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

