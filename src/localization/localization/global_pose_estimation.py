import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleOdometry
from localization.msg import TagDetection
import yaml
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

def load_tag_locations(file_path):
    """
    Load tag locations from a YAML file into a dictionary.
    """
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
        tag_dict = {tag['id']: tuple(tag['position']) for tag in data['ar_tags']}
    return tag_dict

class PositionEstimationNode(Node):
    """
    ROS 2 Node for estimating the global position of the drone.
    """

    def __init__(self, tag_file_path):
        super().__init__("position_estimation_node")
        self.get_logger().warn("Vehicle not localized: No AR tags detected yet.")
        # Load tag locations from the YAML file
        self.TAG_LOCATIONS_ = load_tag_locations(tag_file_path)
        self.TAG_IDS_ = set(self.TAG_LOCATIONS_.keys())
        self.last_seen_ar_tag = None
        self.position_estimate = None

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10)

        # Flags to check if messages have been received
        self.odometry_received = False
        self.tag_detection_received = False
        
        # Timers to periodically check for message reception
        self.create_timer(3, self.check_for_odometry)
        self.create_timer(3, self.check_for_tag_detection)

        # Create subscribers for AR tag detection and vehicle odometry
        self.create_subscription(TagDetection, "/ar_tag_detections", self.ar_tag_callback, 10)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.odom_callback, qos_profile)

        # Create a publisher for the estimated position
        self.position_pub = self.create_publisher(Point, "/estimated_position", 10)

    def ar_tag_callback(self, msg):
        """
        Callback function to update position estimate based on AR tag detection.
        """
        self.tag_detection_received = True
        if msg.id in self.TAG_IDS_:
            known_location = self.TAG_LOCATIONS_[msg.id]
            relative_position = np.array([msg.position.pose.position.x, msg.position.pose.position.y, msg.position.pose.position.z])
            
            # Convert the relative position from the camera frame to the NED frame
            relative_position_ned = self.convert_camera_to_ned(relative_position)

            # Update the position estimate
            self.position_estimate = np.array(known_location) + relative_position_ned
            self.last_seen_ar_tag = self.get_clock().now()
            self.publish_position()

    def odom_callback(self, msg):
        """
        Callback function to update position estimate based on vehicle odometry.
        """
        self.odometry_received = True
        if self.position_estimate is None:
            self.get_logger().warn("Vehicle not localized: No AR tags detected yet.")
        else:
            # Update position based on odometry
            self.position_estimate[0] = msg.position[0]  # North
            self.position_estimate[1] = msg.position[1]  # East
            self.position_estimate[2] = msg.position[2]  # Down (Z-axis)
            self.publish_position()

    def publish_position(self):
        """
        Publish the estimated global position of the drone.
        """
        if self.position_estimate is not None:
            position_msg = Point()
            position_msg.x = self.position_estimate[0]
            position_msg.y = self.position_estimate[1]
            position_msg.z = self.position_estimate[2]
            self.position_pub.publish(position_msg)

    def convert_camera_to_ned(self, camera_position):
        """
        Convert the camera position from the camera frame to the NED frame.
        """
        # Assuming the camera frame has the following axes:
        # x (right), y (down), z (forward)
        # Convert to NED frame:
        # x -> x (North)
        # y -> -z (Down)
        # z -> -y (East)
        return np.array([camera_position[0], -camera_position[2], -camera_position[1]])

    def check_for_odometry(self):
        """
        Timer callback to check if odometry messages have been received.
        """
        if not self.odometry_received:
            self.get_logger().info("Waiting for odometry messages...")

    def check_for_tag_detection(self):
        """
        Timer callback to check if tag detection messages have been received.
        """
        if not self.tag_detection_received:
            self.get_logger().info("Waiting for AR tag detection messages...")

def main(args=None):
    rclpy.init(args=args)

    # Path to the file containing tag locations
    tag_file_path = '/home/immanuel/vanguard_ws/src/localization/config/ar_tag_positions.yaml'

    # Create and spin the node
    node = PositionEstimationNode(tag_file_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
