#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__("coverage_planner")

        # Params
        self.declare_parameter("width", 2.0)
        self.declare_parameter("height", 2.0)
        self.declare_parameter("spacing", 0.45)

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("frame_id", "map")

        self.width = float(self.get_parameter("width").value)
        self.height = float(self.get_parameter("height").value)
        self.spacing = float(self.get_parameter("spacing").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        # State
        self.map_msg: Optional[OccupancyGrid] = None
        self.pose_msg: Optional[PoseWithCovarianceStamped] = None
        self.plan: List[Tuple[float, float, float]] = []
        self.idx = 0
        self.started = False
        self.goal_in_flight = False

        # IMPORTANT: /map is typically transient_local (latched)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.on_pose, pose_qos)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.get_logger().info(
            f"Waiting for {self.map_topic} (TRANSIENT_LOCAL) and {self.pose_topic}... "
            f"(set initial pose / AMCL must be active)"
        )

        # Timer drives everything (no “missed callback” issues)
        self.create_timer(0.5, self.tick)

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def on_pose(self, msg: PoseWithCovarianceStamped):
        self.pose_msg = msg

    def tick(self):
        if self.started:
            return

        if self.map_msg is None or self.pose_msg is None:
            return

        # Build plan centered on current AMCL pose
        cx = float(self.pose_msg.pose.pose.position.x)
        cy = float(self.pose_msg.pose.pose.position.y)

        self.plan = self.make_lawnmower_plan(cx, cy, self.width, self.height, self.spacing)
        self.idx = 0
        self.started = True

        self.get_logger().info(
            f"Auto-centered coverage: center=({cx:.2f},{cy:.2f}) area={self.width}x{self.height}m "
            f"spacing={self.spacing:.2f} -> {len(self.plan)} waypoints"
        )

        # Start Nav2
        self.create_timer(0.2, self.run_plan)

    def make_lawnmower_plan(self, cx: float, cy: float, w: float, h: float, s: float):
        x0 = cx - w / 2.0
        x1 = cx + w / 2.0
        y0 = cy - h / 2.0
        y1 = cy + h / 2.0

        # number of “rows”
        rows = max(1, int(math.floor((y1 - y0) / s)) + 1)

        pts: List[Tuple[float, float, float]] = []
        for i in range(rows):
            y = y0 + i * s
            if y > y1:
                y = y1

            if i % 2 == 0:
                # left -> right
                pts.append((x0, y, 0.0))
                pts.append((x1, y, 0.0))
            else:
                # right -> left
                pts.append((x1, y, math.pi))
                pts.append((x0, y, math.pi))

        # remove duplicates that can happen at boundaries
        dedup: List[Tuple[float, float, float]] = []
        for p in pts:
            if not dedup or (abs(dedup[-1][0] - p[0]) > 1e-6 or abs(dedup[-1][1] - p[1]) > 1e-6):
                dedup.append(p)
        return dedup

    def run_plan(self):
        if not self.started:
            return
        if self.goal_in_flight:
            return
        if self.idx >= len(self.plan):
            self.get_logger().info("Coverage finished: all waypoints sent.")
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info("Waiting for Nav2 action server (navigate_to_pose)...")
            return

        x, y, yaw = self.plan[self.idx]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_to_quat(float(yaw))

        self.goal_in_flight = True
        self.get_logger().info(f"Sending waypoint {self.idx+1}/{len(self.plan)}: x={x:.2f}, y={y:.2f}")

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal REJECTED by Nav2.")
            self.goal_in_flight = False
            self.idx += 1
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        res = future.result().result
        status = future.result().status

        # status codes: 4 = SUCCEEDED, 6 = ABORTED, 5 = CANCELED (Nav2 standard)
        if status == 4:
            self.get_logger().info("Nav2 result: SUCCEEDED")
        elif status == 6:
            self.get_logger().warn("Nav2 result: ABORTED (skipping waypoint)")
        elif status == 5:
            self.get_logger().warn("Nav2 result: CANCELED (skipping waypoint)")
        else:
            self.get_logger().warn(f"Nav2 result: status={status} (skipping waypoint)")

        self.goal_in_flight = False
        self.idx += 1


def main():
    rclpy.init()
    node = CoveragePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
