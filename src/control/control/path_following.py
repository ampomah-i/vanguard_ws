import rclpy
import yaml
import numpy as np
import math
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand, VehicleOdometry

def load_path(file_path, path_id):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data['paths'][str(path_id)]

class PathFollowingNode(Node):
    def __init__(self):
        super().__init__("path_following_node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.odometry_subscriber = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.odometry_callback, qos_profile)

        self.control_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.armed = False
        self.up = False
        self.current_pos = None

        # Load waypoints from YAML file
        self.path_id = 2  # Set the path ID to follow (1 for square, 2 for circle)
        self.waypoints = load_path('path.yaml', self.path_id)
        self.current_waypoint_index = 0

        self.timer = self.create_timer(1.0, self.follow_path)  # Adjust the timer period as needed

    def set_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # PX4 timestamp in microseconds
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.publisher.publish(msg)

    def follow_path(self):
        if self.current_pos is None:
            return

        # Arm the drone and set offboard mode if not already done
        if not self.armed:
            self.set_offboard_mode()
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.armed = True
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info(f"All waypoints for path {self.path_id} reached.")
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        waypoint_pos = np.array([waypoint['x'], waypoint['y'], waypoint['z']])

        if np.linalg.norm(waypoint_pos - np.array(self.current_pos)) < 0.2:  # Check if close to waypoint
            self.current_waypoint_index += 1
            return

        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = [waypoint['x'], waypoint['y'], waypoint['z']]
        msg.velocity = [float("nan") for _ in range(3)]
        msg.acceleration = [float("nan") for _ in range(3)]
        msg.yaw = float("nan")

        self.set_offboard_mode()
        self.control_publisher.publish(msg)

    def odometry_callback(self, msg):
        self.current_pos = [msg.position[0], msg.position[1], msg.position[2]]

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()