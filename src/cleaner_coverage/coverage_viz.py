#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class CoverageViz(Node):
    def __init__(self):
        super().__init__('coverage_viz')

        # Params
        self.declare_parameter('radius_m', 0.40)
        self.declare_parameter('publish_hz', 2.0)
        self.declare_parameter('pose_source', 'odom')  # 'odom' or 'amcl'
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('amcl_topic', '/amcl_pose')

        self.radius_m = float(self.get_parameter('radius_m').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.pose_source = str(self.get_parameter('pose_source').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.amcl_topic = str(self.get_parameter('amcl_topic').value)

        # TF (for odom -> map)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Map storage
        self.map_msg = None
        self.res = None
        self.w = None
        self.h = None
        self.origin_x = None
        self.origin_y = None
        self.free_mask = None
        self.free_cells = 0

        # Coverage storage
        self.visited = set()

        # Path
        self.path = Path()
        self.path.header.frame_id = 'map'

        # QoS for map (latched)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)

        # Pose source subscriptions
        if self.pose_source == 'amcl':
            from geometry_msgs.msg import PoseWithCovarianceStamped
            self.create_subscription(PoseWithCovarianceStamped, self.amcl_topic, self.on_amcl_pose, 20)
            self.get_logger().info(f"Using AMCL pose: {self.amcl_topic}")
        else:
            self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
            self.get_logger().info(f"Using ODOM pose + TF map<-odom: {self.odom_topic}")

        # Publishers
        self.pub_markers = self.create_publisher(MarkerArray, '/coverage_markers', 10)
        self.pub_text = self.create_publisher(Marker, '/coverage_text', 10)
        self.pub_path = self.create_publisher(Path, '/coverage_path', 10)

        # Timer
        period = 1.0 / max(0.1, self.publish_hz)
        self.create_timer(period, self.publish_all)

        self.get_logger().info(f"coverage_viz started: radius_m={self.radius_m} publish_hz={self.publish_hz}")

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.res = float(msg.info.resolution)
        self.w = int(msg.info.width)
        self.h = int(msg.info.height)
        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)

        data = msg.data
        self.free_mask = [False] * (self.w * self.h)
        self.free_cells = 0
        for i, v in enumerate(data):
            if v == 0:
                self.free_mask[i] = True
                self.free_cells += 1

        self.get_logger().info(f"Map loaded: {self.w}x{self.h} res={self.res:.3f} free_cells={self.free_cells}")

    def world_to_cell(self, x, y):
        cx = int((x - self.origin_x) / self.res)
        cy = int((y - self.origin_y) / self.res)
        return cx, cy

    def cell_to_world_center(self, cx, cy):
        x = self.origin_x + (cx + 0.5) * self.res
        y = self.origin_y + (cy + 0.5) * self.res
        return x, y

    def idx(self, cx, cy):
        return cy * self.w + cx

    def paint_at_map_xy(self, x_map: float, y_map: float):
        if self.map_msg is None or self.free_mask is None:
            return

        cx, cy = self.world_to_cell(x_map, y_map)
        r_cells = max(1, int(math.ceil(self.radius_m / self.res)))

        for dy in range(-r_cells, r_cells + 1):
            yy = cy + dy
            if yy < 0 or yy >= self.h:
                continue
            for dx in range(-r_cells, r_cells + 1):
                xx = cx + dx
                if xx < 0 or xx >= self.w:
                    continue
                if (dx * dx + dy * dy) * (self.res * self.res) > self.radius_m * self.radius_m:
                    continue
                i = self.idx(xx, yy)
                if self.free_mask[i]:
                    self.visited.add(i)

    def append_path(self, x_map, y_map, q):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x_map)
        ps.pose.position.y = float(y_map)
        ps.pose.position.z = 0.0
        ps.pose.orientation = q
        self.path.poses.append(ps)
        self.path.header.stamp = ps.header.stamp

    def on_amcl_pose(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.paint_at_map_xy(x, y)
        self.append_path(x, y, msg.pose.pose.orientation)

    def on_odom(self, msg: Odometry):
        # Convert odom pose -> map using TF
        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return

        pose_in = PoseStamped()
        pose_in.header = msg.header
        pose_in.pose = msg.pose.pose

        try:
            pose_out = do_transform_pose(pose_in.pose, tf)
        except Exception:
            return

        x = float(pose_out.position.x)
        y = float(pose_out.position.y)

        self.paint_at_map_xy(x, y)
        self.append_path(x, y, pose_out.orientation)

    def publish_all(self):
        if self.map_msg is None or self.free_mask is None:
            return

        visited_free = len(self.visited)
        pct = (100.0 * visited_free / float(self.free_cells)) if self.free_cells > 0 else 0.0

        stamp = self.get_clock().now().to_msg()

        # Text marker
        text = Marker()
        text.header.frame_id = 'map'
        text.header.stamp = stamp
        text.ns = 'coverage'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.origin_x + 0.5 * self.w * self.res
        text.pose.position.y = self.origin_y + 0.05 * self.h * self.res
        text.pose.position.z = 0.15
        text.scale.z = 0.35
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.text = f"Coverage: {pct:.1f}%   visited {visited_free}/{self.free_cells}"

        # Visited cells (cap for RViz performance)
        cubes = Marker()
        cubes.header.frame_id = 'map'
        cubes.header.stamp = stamp
        cubes.ns = 'coverage_cells'
        cubes.id = 2
        cubes.type = Marker.CUBE_LIST
        cubes.action = Marker.ADD
        cubes.scale.x = self.res
        cubes.scale.y = self.res
        cubes.scale.z = 0.02
        cubes.color.a = 0.55
        cubes.color.r = 0.2
        cubes.color.g = 1.0
        cubes.color.b = 0.2

        MAX_POINTS = 25000
        count = 0
        for i in self.visited:
            cy = i // self.w
            cx = i - cy * self.w
            px, py = self.cell_to_world_center(cx, cy)
            p = Point()
            p.x, p.y, p.z = px, py, 0.01
            cubes.points.append(p)
            count += 1
            if count >= MAX_POINTS:
                break

        arr = MarkerArray()
        arr.markers.append(text)
        arr.markers.append(cubes)

        self.pub_markers.publish(arr)
        self.pub_text.publish(text)
        self.pub_path.publish(self.path)


def main():
    rclpy.init()
    node = CoverageViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

