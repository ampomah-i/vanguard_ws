import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        self.target_height = self.declare_parameter('target_height', 10.0).get_parameter_value().double_value

        # Publisher for vehicle commands
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        # Start the arming and takeoff sequence
        self.arm_and_takeoff()

    def arm_and_takeoff(self):
        self.arm_drone()
        self.create_timer(2.0, self.takeoff_drone)  # Add a delay to ensure the drone is armed before sending the takeoff command

    def arm_drone(self):
        self.get_logger().info('Arming drone...')
        arm_cmd = VehicleCommand()
        arm_cmd.timestamp = self.get_clock().now().nanoseconds // 1000
        arm_cmd.param1 = 1.0  # 1 to arm, 0 to disarm
        arm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_cmd.target_system = 1
        arm_cmd.target_component = 1
        arm_cmd.source_system = 1
        arm_cmd.source_component = 1
        arm_cmd.from_external = True
        self.vehicle_command_publisher.publish(arm_cmd)

    def takeoff_drone(self):
        self.get_logger().info('Taking off...')
        takeoff_cmd = VehicleCommand()
        takeoff_cmd.timestamp = self.get_clock().now().nanoseconds // 1000
        takeoff_cmd.param7 = self.target_height  # Altitude to take off to
        takeoff_cmd.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        takeoff_cmd.target_system = 1
        takeoff_cmd.target_component = 1
        takeoff_cmd.source_system = 1
        takeoff_cmd.source_component = 1
        takeoff_cmd.from_external = True
        self.vehicle_command_publisher.publish(takeoff_cmd)

def main(args=None):
    rclpy.init(args=args)
    takeoff_node = TakeoffNode()
    try:
        rclpy.spin(takeoff_node)
    except KeyboardInterrupt:
        takeoff_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        takeoff_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
