#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from px4_msgs.msg import VehicleCommand

class ArmingNode(Node):
    """
    A ROS 2 node for arming and disarming a drone using PX4 commands.
    """

    def __init__(self):
        # Initialize the node with the name 'arm_disarm_node'
        super().__init__("arm_disarm_node")

        # Declare and get parameters
        self.declare_parameter('command_topic', '/fmu/in/vehicle_command')
        self.command_topic = self.get_parameter('command_topic').get_parameter_value().string_value

        # Create a publisher for vehicle commands
        self.publisher = self.create_publisher(VehicleCommand, self.command_topic, 10)

        # Inform about node initialization
        self.get_logger().info("Arming node initialized")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """
        Publish a vehicle command to the PX4 system.
        """
        try:
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
            msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds

            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish vehicle command: {e}")

    def arm(self):
        """
        Send an arm command to the drone.
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        """
        Send a disarm command to the drone.
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")

    def destroy_node(self):
        """
        Ensure resources are cleaned up on shutdown.
        """
        self.get_logger().info("Shutting down node.")
        super().destroy_node()

def main(args=None):
    """
    Main function to initialize and spin the node.
    """
    rclpy.init(args=args)
    arm_disarm_node = ArmingNode()

    try:
        while rclpy.ok():
            user_input = input("Type 'arm' to arm the drone or 'disarm' to disarm the drone: ").strip().lower()
            if user_input == "arm":
                arm_disarm_node.arm()
            elif user_input == "disarm":
                arm_disarm_node.disarm()
            else:
                print("Invalid input. Please type 'arm' or 'disarm'.")
    except KeyboardInterrupt:
        print("\nUser interrupted the process.")
    finally:
        arm_disarm_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
