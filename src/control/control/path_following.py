import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from px4_msgs.msg import TrajectorySetpoint, VehicleOdometry
import numpy as np
import yaml
from common.coordinate_transforms import (
    ned_to_enu,
    enu_to_ned,
    quaternion_from_euler,
    quaternion_to_euler,
    enu_to_ned_orientation,
    quaternion_get_yaw
)

# QoS settings
qos_profile_sub = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    depth=10
)

qos_profile_pub = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

class PathFollowing(Node):
    def __init__(self):
        super().__init__('path_following')
        
        # Declare parameters
        self.declare_parameter('frequency', 10)
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value

        self.current_path = []
        self.current_waypoint_index = 0
        self.current_position = None

        # Subscribe to waypoints
        self.waypoints_subscription = self.create_subscription(
            Path,
            '/drone/waypoints',
            self.waypoints_callback,
            qos_profile_sub
        )

        # Subscribe to vehicle odometry
        self.vehicle_odometry_subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback,
            qos_profile_sub
        )

        # Create publisher for trajectory setpoints
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile_pub
        )

        # Create timer
        self.timer = self.create_timer(1.0 / frequency, self.timer_callback)

    def waypoints_callback(self, msg):
        self.current_path = msg.poses
        self.current_waypoint_index = 0
        self.get_logger().info(f'Received {len(self.current_path)} waypoints')

    def vehicle_odometry_callback(self, msg):
        self.current_position = ned_to_enu([msg.position[0], msg.position[1], msg.position[2]])

    def timer_callback(self):
        if self.current_position is None or self.current_waypoint_index >= len(self.current_path):
            return

        next_waypoint = self.current_path[self.current_waypoint_index].pose.position
        position = [next_waypoint.x, next_waypoint.y, next_waypoint.z]
        velocity = [0.1, 0.1, 0.0]
        yaw = 0.0

        self.publish_trajectory_setpoint(position, velocity, yaw)

        distance = np.linalg.norm(np.array(position) - np.array(self.current_position))
        if distance < 0.5:
            self.current_waypoint_index += 1

    def publish_trajectory_setpoint(self, position, velocity, yaw):
        px4_ned_position = enu_to_ned(position)
        msg = TrajectorySetpoint()
        msg.position = px4_ned_position
        msg.velocity = enu_to_ned(velocity)
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.yaw = yaw
        msg.timestamp = self.get_clock().now().nanoseconds / 1000
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f'Published setpoint: {msg.position}, {msg.velocity}, {msg.yaw}')

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
