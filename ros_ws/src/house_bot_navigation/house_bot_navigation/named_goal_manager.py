"""Adapt simple UI topics and named services to Nav2 actions."""

from functools import partial
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from house_bot_navigation.destinations import Destination, load_destinations, service_slug


RESULT_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}


class NamedGoalManager(Node):
    """Expose browser-friendly goal inputs without reimplementing planning."""

    def __init__(self) -> None:
        super().__init__("named_goal_manager")
        self.declare_parameter("destinations_file", "")
        destinations_file = self.get_parameter("destinations_file").value
        if not destinations_file:
            raise ValueError("destinations_file parameter is required")

        self.destinations = load_destinations(Path(destinations_file))
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_publisher = self.create_publisher(
            String, "/house_bot/navigation_status", latched
        )
        self.destinations_publisher = self.create_publisher(
            String, "/house_bot/destinations", latched
        )

        self.navigate_to_pose = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.navigate_through_poses = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses"
        )
        self.active_goal_handle = None
        self.active_label = ""
        self.last_feedback_time = 0.0

        self.create_subscription(PoseStamped, "/goal_pose", self._pose_callback, 10)
        self.create_subscription(PoseArray, "/waypoints", self._waypoints_callback, 10)
        self.create_subscription(
            String, "/house_bot/navigate_named", self._named_callback, 10
        )

        self._service_handles = [
            self.create_service(
                Trigger,
                "/house_bot/cancel_navigation",
                self._cancel_callback,
            )
        ]
        for destination in self.destinations.values():
            self._service_handles.append(
                self.create_service(
                    Trigger,
                    f"/house_bot/go/{service_slug(destination.name)}",
                    partial(self._destination_callback, destination=destination),
                )
            )

        self._publish_destinations()
        self._publish_status("ready", detail="Waiting for a navigation goal")
        self.get_logger().info(
            f"Loaded {len(self.destinations)} destinations from {destinations_file}"
        )

    def _publish_destinations(self) -> None:
        payload = {
            destination.name: {
                "x": destination.x,
                "y": destination.y,
                "yaw": destination.yaw,
                "description": destination.description,
                "service": f"/house_bot/go/{service_slug(destination.name)}",
            }
            for destination in self.destinations.values()
        }
        self.destinations_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_status(self, state: str, **fields: object) -> None:
        payload = {
            "state": state,
            "goal": self.active_label or None,
            "stamp_ns": self.get_clock().now().nanoseconds,
            **fields,
        }
        self.status_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _destination_callback(
        self, request: Trigger.Request, response: Trigger.Response, destination: Destination
    ) -> Trigger.Response:
        del request
        if not self.navigate_to_pose.server_is_ready():
            response.success = False
            response.message = "Nav2 navigate_to_pose action is not ready"
            return response
        self._send_pose(self._destination_pose(destination), destination.name)
        response.success = True
        response.message = f"Navigation requested: {destination.name}"
        return response

    def _cancel_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        if self.active_goal_handle is None:
            response.success = True
            response.message = "No active navigation goal"
            return response
        self.active_goal_handle.cancel_goal_async()
        self._publish_status("canceling")
        response.success = True
        response.message = f"Cancel requested: {self.active_label}"
        return response

    def _named_callback(self, message: String) -> None:
        requested = message.data.strip().lower()
        destination = next(
            (
                item
                for item in self.destinations.values()
                if item.name.lower() == requested or service_slug(item.name) == requested
            ),
            None,
        )
        if destination is None:
            self._publish_status("rejected", detail=f"Unknown destination: {message.data}")
            return
        self._send_pose(self._destination_pose(destination), destination.name)

    def _pose_callback(self, pose: PoseStamped) -> None:
        if not pose.header.frame_id:
            pose.header.frame_id = "map"
        self._send_pose(pose, "map goal")

    def _waypoints_callback(self, message: PoseArray) -> None:
        if not message.poses:
            self._publish_status("rejected", detail="Waypoint list was empty")
            return
        if not self.navigate_through_poses.server_is_ready():
            self._publish_status(
                "rejected", detail="Nav2 navigate_through_poses action is not ready"
            )
            return

        stamp = self.get_clock().now().to_msg()
        frame = message.header.frame_id or "map"
        goal = NavigateThroughPoses.Goal()
        for pose in message.poses:
            stamped = PoseStamped()
            stamped.header.frame_id = frame
            stamped.header.stamp = stamp
            stamped.pose = pose
            goal.poses.append(stamped)

        self.active_label = f"{len(goal.poses)} waypoints"
        future = self.navigate_through_poses.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(partial(self._goal_response_callback, label=self.active_label))
        self._publish_status("requested", count=len(goal.poses))

    def _destination_pose(self, destination: Destination) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = destination.x
        pose.pose.position.y = destination.y
        pose.pose.orientation.z = math.sin(destination.yaw / 2.0)
        pose.pose.orientation.w = math.cos(destination.yaw / 2.0)
        return pose

    def _send_pose(self, pose: PoseStamped, label: str) -> None:
        if not self.navigate_to_pose.server_is_ready():
            self._publish_status(
                "rejected", detail="Nav2 navigate_to_pose action is not ready"
            )
            return
        pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.active_label = label
        future = self.navigate_to_pose.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(partial(self._goal_response_callback, label=label))
        self._publish_status(
            "requested",
            x=round(pose.pose.position.x, 3),
            y=round(pose.pose.position.y, 3),
        )

    def _goal_response_callback(self, future, label: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # rclpy transports exception details here
            self._publish_status("error", detail=str(exc))
            return
        if not goal_handle.accepted:
            self._publish_status("rejected", detail=f"Nav2 rejected {label}")
            return
        self.active_goal_handle = goal_handle
        self.active_label = label
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self._result_callback, label=label))
        self._publish_status("navigating")

    def _feedback_callback(self, feedback_message) -> None:
        now = time.monotonic()
        if now - self.last_feedback_time < 0.5:
            return
        self.last_feedback_time = now
        feedback = feedback_message.feedback
        self._publish_status(
            "navigating",
            distance_remaining=round(float(feedback.distance_remaining), 3),
            recoveries=int(feedback.number_of_recoveries),
        )

    def _result_callback(self, future, label: str) -> None:
        try:
            wrapped_result = future.result()
            state = RESULT_NAMES.get(wrapped_result.status, "failed")
            detail = getattr(wrapped_result.result, "error_msg", "")
            error_code = int(getattr(wrapped_result.result, "error_code", 0))
        except Exception as exc:  # rclpy transports exception details here
            state = "error"
            detail = str(exc)
            error_code = -1
        self.active_goal_handle = None
        self.active_label = label
        self._publish_status(state, detail=detail, error_code=error_code)
        self.active_label = ""


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NamedGoalManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
