#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleOdometry  # Assuming this is the message type for PX4 odometry
import numpy as np

class GlobalLocalizationNode(Node):

    def __init__(self):
        super().__init__("global_localization")

        # Initialize variables
        self.initial_position = None
        self.initial_odometry = None
        self.latest_odometry = None
        self.position_initialized = False

        # Tolerance for position update verification (in meters)
        self.update_tolerance = 0.5

        # Create a subscriber for the estimated position from AR tags
        self.create_subscription(Point, "/estimated_position", self.tag_position_callback, 10)

        # Create a subscriber for the PX4 odometry data
        self.create_subscription(VehicleOdometry, "/px4/odometry", self.odometry_callback, 10)

        # Create a publisher for the global estimated position
        self.global_position_pub = self.create_publisher(Point, "/global_estimated_position", 10)

        self.get_logger().info("Global Localization Node has been initialized.")

    def tag_position_callback(self, msg):
        tag_position = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
        if not self.position_initialized:
            self.initial_position = tag_position
            if self.latest_odometry is not None:
                self.initial_odometry = self.latest_odometry
                self.position_initialized = True
                self.get_logger().info(f"Initial position set: {self.initial_position}")
                self.get_logger().info(f"Initial odometry set: {self.initial_odometry}")
        else:
            # Verify and update the estimate if necessary
            self.verify_and_update_estimate(tag_position)

    def odometry_callback(self, msg):
        # Extract odometry data (assuming position in odometry message)
        self.latest_odometry = np.array([msg.position[0], msg.position[1], msg.position[2]], dtype=np.float32)

        if not self.position_initialized:
            self.get_logger().info("Waiting for initial tag position.")
            return

        # Compute the global position using the initial position, initial odometry, and latest odometry
        global_position = self.compute_global_position()

        # Publish the global position
        global_position_msg = Point()
        global_position_msg.x, global_position_msg.y, global_position_msg.z = global_position
        self.global_position_pub.publish(global_position_msg)

    def compute_global_position(self):
        # Calculate the change in position from the initial odometry
        delta_position = self.latest_odometry - self.initial_odometry
        # Adjust the global position using the initial AR tag position and the delta position
        global_position = self.initial_position + delta_position
        return global_position

    def verify_and_update_estimate(self, tag_position):
        # Compute the current estimated position
        current_estimate = self.compute_global_position()

        # Calculate the difference between the AR tag position and the current estimate
        position_difference = np.linalg.norm(tag_position - current_estimate)

        # If the difference exceeds the tolerance, update the initial position and odometry
        if position_difference > self.update_tolerance:
            self.initial_position = tag_position
            self.initial_odometry = self.latest_odometry
            self.get_logger().info(f"Estimate updated with new AR tag position: {self.initial_position}")
            self.get_logger().info(f"New initial odometry set: {self.initial_odometry}")

def main(args=None):
    rclpy.init(args=args)

    # Create and spin the global localization node
    node = GlobalLocalizationNode()
    rclpy.spin(node)

    # Shutdown the rclpy library
    rclpy.shutdown()

if __name__ == "__main__":
    main()
