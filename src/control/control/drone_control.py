import sys
import os
import yaml

# Add the `src` directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleControlMode,
    VehicleOdometry
)
import math
import threading
import numpy as np
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

class State:
    PREFLIGHT = 0
    IDLE = 1
    OFFBOARD = 2
    MISSION = 3
    LAND = 4
    RTL = 5
    FAIL = 6
    LIMBO = 7
    ERROR = 8

class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        self.get_logger().info('Init Node')
        self.get_logger().info('State transitioned to PREFLIGHT')

        # Declare default parameters values
        self.declare_parameter('frequency', 100)
        self.declare_parameter('waypoints_file', 'path.yaml')
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value

        # Load waypoints from YAML file
        self.waypoints = self.load_waypoints(waypoints_file)
        self.current_waypoint_index = 0

        # Create publishers
        self.offboard_control_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile_pub)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile_pub)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile_pub)
        self.home_position_publisher_ = self.create_publisher(Point, '/home_position', qos_profile_pub)
        self.waypoints_publisher_ = self.create_publisher(Path, '/drone/waypoints', qos_profile_pub)
        
        # Create subscribers
        self.vehicle_odometry_subscriber_ = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile_sub)
        self.vehicle_control_mode_subscriber_ = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile_sub)
        self.path_subscriber_ = self.create_subscription(Path, '/f2c_path', self.path_callback, qos_profile_sub)
        self.vehicle_pose_subscriber_ = self.create_subscription(PoseStamped, '/vehicle_pose', self.pose_callback, qos_profile_sub)

        # Create timer
        self.timer_ = self.create_timer(1.0 / frequency, self.timer_callback)

        # Variables
        self.offboard_setpoint_counter_ = 0
        self.test_counter_ = 0
        self.current_state_ = State.PREFLIGHT
        self.vehicle_position_ = Point()
        self.waypoints_ = []
        self.f2c_path_ros_ = Path()
        self.vehicle_status_ = VehicleControlMode()
        self.global_i_ = 0
        self.position_tolerance_ = 0.3
        self.take_off_waypoint = Point()
        self.home_position_ros_ = Point()
        self.drone_yaw_ros_ = 0.0
        self.take_off_heading_ = 0.0
        self.take_off_height = 2.2
        self.vehicle_position_ros_ = Point()
        self.path_moved_to_drone_local_coordinates_ = Point()
        self.velocity_setpoint_ = Point()
        self.current_path = None
        self.current_position = None

        # Flags
        self.mutex_ = threading.Lock()
        self.flag_timer_done_ = False
        self.has_executed_ = False
        self.flag_next_waypoint_ = True
        self.flag_mission_ = False
        self.flag_vehicle_odometry_ = False
        self.flag_take_off_position = False

    def load_waypoints(self, filepath):
        with open(filepath, 'r') as file:
            data = yaml.safe_load(file)
        waypoints = data.get('paths', {}).get('1', [])
        return waypoints

    def vehicle_odometry_callback(self, msg):
        px4_ned = np.array([msg.position[0], msg.position[1], msg.position[2]])
        ros_enu = ned_to_enu(px4_ned)
        self.vehicle_position_.x, self.vehicle_position_.y, self.vehicle_position_.z = ros_enu
        self.current_position = ros_enu

    def vehicle_control_mode_callback(self, msg):
        self.vehicle_status_ = msg

    def path_callback(self, msg):
        self.f2c_path_ros_ = msg
        self.flag_mission_ = True
        self.current_path = msg.poses
        self.get_logger().info(f'Received new path with {len(self.current_path)} waypoints')

    def pose_callback(self, msg):
        self.vehicle_position_ros_ = msg.pose.position

        tf_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        roll, pitch, yaw = quaternion_to_euler(tf_q)
        self.drone_yaw_ros_ = yaw
        self.flag_vehicle_odometry_ = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds / 1000
        self.offboard_control_publisher_.publish(msg)

    def publish_trajectory_setpoint(self, position, velocity, yaw):
        ros_enu = np.array(position)
        px4_ned = enu_to_ned(ros_enu)
        q_ros = quaternion_from_euler(0, 0, yaw)
        q_px4 = enu_to_ned_orientation(q_ros)
        yaw = quaternion_get_yaw(q_px4)

        msg = TrajectorySetpoint()
        msg.position = px4_ned.tolist()
        msg.velocity = velocity
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.yaw = yaw
        msg.yawspeed = 0.174533
        msg.timestamp = self.get_clock().now().nanoseconds / 1000
        self.trajectory_setpoint_publisher_.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent')

    def takeoff(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 0, 0, 0, 4.21, 473977222, 85456111, 570200)
        self.get_logger().info('Takeoff command sent')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')

    def RTL(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info('RTL command sent')

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds / 1000
        self.vehicle_command_publisher_.publish(msg)

    def nonBlockingWait(self, duration):
        def wait():
            rclpy.sleep(duration)
            with self.mutex_:
                self.flag_timer_done_ = True

        thread = threading.Thread(target=wait)
        thread.daemon = True
        thread.start()

    def euclidean_distance(self, v1, v2):
        return math.sqrt((v2.x - v1.x)**2 + (v2.y - v1.y)**2 + (v2.z - v1.z)**2)

    def reached_setpoint(self, v1, v2, tolerance=1.0):
        return abs(self.euclidean_distance(v1, v2)) <= tolerance

    def degreesToRadians(self, degrees):
        return degrees * (math.pi / 180.0)

    def check_pilot_state_switch(self):
        if not self.vehicle_status_.flag_control_offboard_enabled and self.vehicle_status_.flag_control_auto_enabled:
            self.current_state_ = State.LIMBO
            self.get_logger().info('Pilot switched state! State transitioned to LIMBO')

    def set_take_off_waypoint(self):
        if not self.flag_take_off_position:
            self.take_off_waypoint.x = self.vehicle_position_.x
            self.take_off_waypoint.y = self.vehicle_position_.y
            self.take_off_waypoint.z = self.vehicle_position_.z + self.take_off_height
            self.take_off_heading_ = self.drone_yaw_ros_

            self.home_position_ros_.x = self.vehicle_position_.x
            self.home_position_ros_.y = self.vehicle_position_.y
            self.home_position_ros_.z = self.vehicle_position_.z

            self.get_logger().info(f'Home position: x: {self.home_position_ros_.x} y: {self.home_position_ros_.y} z: {self.home_position_ros_.z}')
        
        self.flag_take_off_position = True

    def publish_waypoints(self):
        waypoints = self.load_waypoints('path.yaml')
        path_msg = Path()
        path_msg.header.frame_id = 'map'  # Set the frame id appropriately
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for wp in waypoints:
            pose = PoseStamped()
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.position.z = wp['z']
            path_msg.poses.append(pose)
        
        self.waypoints_publisher_.publish(path_msg)
        self.get_logger().info('Published waypoints')

    def calculate_velocity_setpoint(self):
        velocity_setpoint = Point()
        n_steps = 10
        x_vel = 0.1

        if self.global_i_ + n_steps < len(self.f2c_path_ros_.poses) and self.global_i_ >= 1:
            if (self.f2c_path_ros_.poses[self.global_i_-1].pose.position.x ==
                self.f2c_path_ros_.poses[self.global_i_ + n_steps].pose.position.x):
                if (self.f2c_path_ros_.poses[self.global_i_ - 1].pose.position.y <
                    self.f2c_path_ros_.poses[self.global_i_ + n_steps].pose.position.y):
                    velocity_setpoint.x = x_vel
                else:
                    velocity_setpoint.x = -x_vel
                velocity_setpoint.y = 0.0
            else:
                velocity_setpoint.x = 0.1
                velocity_setpoint.y = 0.1
        else:
            velocity_setpoint.x = 0.1
            velocity_setpoint.y = 0.1

        velocity_setpoint.z = 0.0

        return velocity_setpoint

    def path_update_to_takeoff_position(self, x, y, z):
        point = Point()
        point.x = x + self.home_position_ros_.x
        point.y = y + self.home_position_ros_.y
        point.z = z + self.home_position_ros_.z
        return point

    def timer_callback(self):
        if self.flag_take_off_position:
            self.home_position_publisher_.publish(self.home_position_ros_)

        if self.current_state_ == State.PREFLIGHT:
            if self.flag_vehicle_odometry_:
                self.set_take_off_waypoint()
                self.current_state_ = State.IDLE

        elif self.current_state_ == State.IDLE:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(
                [self.take_off_waypoint.x, self.take_off_waypoint.y, self.take_off_waypoint.z],
                [0.1, 0.1, 0.1],
                self.take_off_heading_
            )

            if self.offboard_setpoint_counter_ >= 200 and self.flag_mission_:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
                self.get_logger().info('Sent Vehicle Command')
                self.arm()

                if self.vehicle_status_.flag_armed and self.vehicle_status_.flag_control_offboard_enabled:
                    self.current_state_ = State.OFFBOARD
                    self.get_logger().info('State transitioned to OFFBOARD')

            self.offboard_setpoint_counter_ += 1

        elif self.current_state_ == State.OFFBOARD:
            self.check_pilot_state_switch()
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(
                [self.take_off_waypoint.x, self.take_off_waypoint.y, self.take_off_waypoint.z],
                [0.0, 0.0, 0.0],
                self.take_off_heading_
            )

            if self.reached_setpoint(self.take_off_waypoint, self.vehicle_position_, self.position_tolerance_):
                if not self.has_executed_:
                    self.nonBlockingWait(5)
                    self.has_executed_ = True

                if self.flag_timer_done_:
                    self.current_state_ = State.MISSION
                    self.get_logger().info('State transitioned to MISSION with Fields2Cover path')
                    self.flag_timer_done_ = False
                    self.has_executed_ = False

        elif self.current_state_ == State.MISSION:
            self.check_pilot_state_switch()
            self.publish_offboard_control_mode()

            if self.flag_next_waypoint_ and self.current_waypoint_index < len(self.waypoints):
                next_waypoint = self.waypoints[self.current_waypoint_index]
                self.publish_trajectory_setpoint(
                    [next_waypoint['x'], next_waypoint['y'], next_waypoint['z']],
                    [0.1, 0.1, 0.0],  # Example velocity, adjust as needed
                    0.0  # Example yaw, adjust as needed
                )
                self.flag_next_waypoint_ = False
            elif self.current_waypoint_index < len(self.waypoints):
                next_waypoint = self.waypoints[self.current_waypoint_index]
                if self.reached_setpoint(Point(x=next_waypoint['x'], y=next_waypoint['y'], z=next_waypoint['z']), self.vehicle_position_, self.position_tolerance_):
                    if not self.has_executed_:
                        self.nonBlockingWait(0.001)
                        self.has_executed_ = True

                    if self.flag_timer_done_:
                        self.flag_timer_done_ = False
                        self.has_executed_ = False
                        self.flag_next_waypoint_ = True
                        self.current_waypoint_index += 1

                        if self.current_waypoint_index >= len(self.waypoints):
                            self.current_state_ = State.RTL
                            self.get_logger().info('State transitioned to LAND')

        elif self.current_state_ == State.LAND:
            self.land()
            if not self.vehicle_status_.flag_control_offboard_enabled and self.vehicle_status_.flag_control_auto_enabled:
                self.current_state_ = State.LIMBO
                self.get_logger().info('State transitioned to LIMBO')

        elif self.current_state_ == State.RTL:
            self.RTL()
            if not self.vehicle_status_.flag_control_offboard_enabled and self.vehicle_status_.flag_control_auto_enabled:
                self.current_state_ = State.LIMBO
                self.get_logger().info('State transitioned to LIMBO')

        elif self.current_state_ == State.FAIL:
            self.current_state_ = State.LAND

        elif self.current_state_ == State.LIMBO:
            pass

        else:
            self.get_logger().info('------ Default state ------')


def main(args=None):
    rclpy.init(args=args)
    drone_control = DroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
