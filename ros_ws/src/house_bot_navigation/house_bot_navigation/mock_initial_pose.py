"""Seed Nav2's loopback simulator at the mock map's Home pose."""

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


class MockInitialPose(Node):
    def __init__(self) -> None:
        super().__init__("mock_initial_pose")
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.timer = self.create_timer(0.25, self._publish)

    def _publish(self) -> None:
        if self.publisher.get_subscription_count() == 0:
            return
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = 0.01
        self.publisher.publish(message)
        self.get_logger().info("Published mock Home initial pose")
        self.timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
