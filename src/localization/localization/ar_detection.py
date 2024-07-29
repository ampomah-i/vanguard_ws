#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point 

# Predefined dictionary for AR tags
test_dict = {99: (23, 20, 0)}

# <tag_id>,<x>,<y>,<z>
def load_tag_locations(self, file_path):
        tag_dict = {}
        with open(file_path, 'r') as file:
            for line in file:
                parts = line.strip().split(':')
                tag_id = int(parts[0].strip())  # Convert the tag ID to an integer
                location = tuple(map(int, parts[1].strip()[1:-1].split(',')))  # Convert the location to a tuple of integers
                tag_dict[tag_id] = location
        return tag_dict

class ARTagsDetectNode(Node):

    br_: CvBridge
    LIBRARY_: cv.aruco.Dictionary
    PARAMETERS_: cv.aruco.DetectorParameters
    DETECTOR_: cv.aruco.ArucoDetector
    TAG_LOCATIONS_: dict
    TAG_IDS_: set

    # Camera intrinsic parameters (replace with actual calibration data)
    camera_matrix = np.array([
        [1000, 0, 320],  # fx, 0, cx
        [0, 1000, 240],  # 0, fy, cy
        [0, 0, 1]        # 0, 0, 1
    ], dtype=np.float32)
    
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)  # Assuming no lens distortion

    # Tag size in meters (replace with actual tag size)
    tag_size = 0.266

    def __init__(self, tag_file_path):
        super().__init__("camera_read_ar_tags")

        # Load tag locations from the file
        self.TAG_LOCATIONS_ = load_tag_locations(tag_file_path)
        self.TAG_IDS_ = set(self.TAG_LOCATIONS_.keys())

        # Create a subscriber to the "/image_raw" topic
        self.create_subscription(Image, "/camera2/image_raw", self.receive_image_data, 10)
        # Create the bridge for converting ROS images to OpenCV images
        self.position_pub = self.create_publisher(Point, "/estimated_position", 10)  # Create the publisher

        self.br_ = CvBridge()

        # Load the predefined AR tag dictionary
        self.LIBRARY_ = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
        self.PARAMETERS_ = cv.aruco.DetectorParameters()
        self.DETECTOR_ = cv.aruco.ArucoDetector(self.LIBRARY_, self.PARAMETERS_)

        self.get_logger().info("\nSuccessfully launched the camera display module!")

    def receive_image_data(self, msg: Image):
        # Convert the ROS Image message to an OpenCV image
        img = self.br_.imgmsg_to_cv2(msg)

        # Detect AR tags in the grayscale version of the image
        markerCorners, markerIDs, rejectedCandidates = self.DETECTOR_.detectMarkers(cv.cvtColor(img, cv.COLOR_BGR2GRAY))

        if markerIDs is not None:
            # Initialize data structures for localization
            detected_tags = {}
            
            # Process each detected marker
            for markerID, corners in zip(markerIDs, markerCorners):
                tag_id = markerID[0]
                if tag_id in self.TAG_IDS_:
                    # Compute tag location and add to detected_tags
                    location = self.TAG_LOCATIONS_[tag_id]
                    detected_tags[tag_id] = self.compute_tag_position(corners, location)
            
            # Perform localization using detected tags
            if detected_tags:
                self.localize_drone(detected_tags)
        else:
            self.get_logger().info("\nNo AR Tags detected.")

    def compute_tag_position(self, corners, known_location):
        """
        Compute the position of the tag based on detected corners.
        """
        # Convert corners to numpy array
        corners = np.array(corners, dtype=np.float32)

        # Define the 3D coordinates of the tag corners in the tag's coordinate system
        tag_corners_3d = np.array([
            [-self.tag_size / 2, -self.tag_size / 2, 0],
            [self.tag_size / 2, -self.tag_size / 2, 0],
            [self.tag_size / 2, self.tag_size / 2, 0],
            [-self.tag_size / 2, self.tag_size / 2, 0]
        ], dtype=np.float32)

        # Estimate the pose of the tag
        _, rvec, tvec = cv.solvePnP(tag_corners_3d, corners, self.camera_matrix, self.dist_coeffs)

        # Convert rotation vector to rotation matrix
        R, _ = cv.Rodrigues(rvec)

        # Compute the tag position in the global frame
        tag_position_camera = tvec.flatten()
        tag_position_global = np.dot(R, tag_position_camera)

        # Adjust based on the known location
        tag_position_global += np.array(known_location, dtype=np.float32)

        return tuple(tag_position_global)

    def localize_drone(self, detected_tags):
        """
        Estimate the drone's position based on detected tag positions.
        """
        if not detected_tags:
            self.get_logger().info("No detected tags for localization.")
            return

        # Extract positions of detected tags
        positions = np.array(list(detected_tags.values()))

        # Example method: Average position of all detected tags
        estimated_position = np.mean(positions, axis=0)
        # Log the estimated position
        self.get_logger().info(f"Drone estimated position: {estimated_position}")
        
        position_msg = Point()
        position_msg.x, position_msg.y, position_msg.z = estimated_position

        # Publish the estimated position
        self.position_pub.publish(position_msg)

def main(args=None):
    rclpy.init(args=args)

    # Path to the file containing tag locations
    tag_file_path = '/home/sarah/vanguard_ws/src/localization/localization/tags_locations.txt'

    # Create and spin the node
    node = ARTagsDetectNode(tag_file_path)
    rclpy.spin(node)

    # Shutdown the rclpy library
    rclpy.shutdown()

if __name__ == "__main__":
    main()
