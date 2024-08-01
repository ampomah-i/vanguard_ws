#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class HoverNode(Node):

    def __init__(self):
        super().__init__('hover_node')

        # Configure QoS profile for the publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/offboard_control_mode/in', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/trajectory_setpoint/in', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/vehicle_command/in', qos_profile)

        # Create subscribers
        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/vehicle_local_position/out', self.local_position_callback, qos_profile)

        # Initialize variables
        self.current_altitude = None
        self.target_altitude = 2.0  # Target hover altitude in meters
        self.hover_initiated = False

        # Create timers
        self.offboard_control_timer = self.create_timer(0.1, self.offboard_control_callback)
        self.hover_check_timer = self.create_timer(1.0, self.hover_check)

        self.get_logger().info("Hover node initialized")

    def local_position_callback(self, msg):
        self.current_altitude = -msg.z  # PX4 uses NED frame, so negative z is altitude

    def offboard_control_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # Arm the vehicle and switch to offboard mode after 2 seconds
        if not self.hover_initiated and self.get_clock().now().nanoseconds / 1e9 > 2.0:
            self.arm()
            self.set_offboard_mode()
            self.hover_initiated = True

    def hover_check(self):
        if self.current_altitude is not None:
            self.get_logger().info(f"Current altitude: {self.current_altitude:.2f} m, Target: {self.target_altitude:.2f} m")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -self.target_altitude]  # x, y, z
        msg.yaw = 0.0  # (0 degrees yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def set_offboard_mode(self):
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Offboard mode set")

    def publish_vehicle_command(self, command, param1):
        msg = VehicleCommand()
        msg.param1 = param1
        # msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    hover_node = HoverNode()
    rclpy.spin(hover_node)
    hover_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()