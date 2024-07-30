import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class Hover(Node):
    def __init__(self):
        super().__init__('height_control_node')
        self.target_height = -1.0  # Desired height in meters
        self.height_tolerance = 0.5  # Tolerance in meters

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile)
        
        self.control_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_height = None
        self.offboard_mode_set = False
        self.armed = False

    def odometry_callback(self, msg):
        self.current_height = msg.position[2]

    def set_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_publisher_.publish(msg)
    
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
        msg.timestamp = self.get_clock().now().nanoseconds // 1000 # time in microseconds
        self.cmd_publisher_.publish(msg)

    def control_loop(self):
        if self.current_height is None:
            self.get_logger().info(f'current height none.')
            return

        if not self.armed:
            self.arm_drone()
            return

        height_error = self.target_height - self.current_height

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.set_offboard_mode()

        if abs(height_error) > self.height_tolerance:

            trajectory_setpoint_msg = TrajectorySetpoint()
            trajectory_setpoint_msg.timestamp = self.get_clock().now().nanoseconds // 1000  
            trajectory_setpoint_msg.position = [float('nan'), float('nan'), self.target_height]  
            trajectory_setpoint_msg.yaw = float('nan')  # NaN to ignore yaw
            self.control_publisher_.publish(trajectory_setpoint_msg)

            self.get_logger().info(f'Adjusting height to {-self.target_height} meters.')
        else:
            self.get_logger().info('Height within tolerance, no adjustment needed.')

    def arm_drone(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.armed = True

def main(args=None):
    rclpy.init(args=args)
    hover_node = Hover()
    rclpy.spin(hover_node)
    hover_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()