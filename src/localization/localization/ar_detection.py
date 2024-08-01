import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from localization.msg import TagDetection
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class ARTagDetectionNode(Node):
    """
    ROS 2 Node for detecting AR tags and publishing the average position of the drone relative to the tags.
    """

    def __init__(self):
        super().__init__("ar_tag_detection_node")

        # Create a subscriber to the camera image topic
        self.create_subscription(Image, "/image_raw", self.receive_image_data, 10)
        # Create a publisher for AR tag detection
        self.detection_pub = self.create_publisher(TagDetection, "/ar_tag_detections", 10)

        # Initialize the bridge for converting ROS images to OpenCV images
        self.br_ = CvBridge()

        # Load the predefined AR tag dictionary
        self.LIBRARY_ = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
        self.PARAMETERS_ = cv.aruco.DetectorParameters()
        self.DETECTOR_ = cv.aruco.ArucoDetector(self.LIBRARY_, self.PARAMETERS_)

        self.get_logger().info("AR tag detection node launched!")

        # Camera intrinsic parameters: TO DO
        self.camera_matrix = np.array([
            [1000, 0, 320],  # fx, 0, cx
            [0, 1000, 240],  # 0, fy, cy
            [0, 0, 1]        # 0, 0, 1
        ], dtype=np.float32)

        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)  # Assuming no lens distortion

        # Tag size in meters: TO DO
        self.tag_size = 0.266

    def receive_image_data(self, msg: Image):
        """
        Callback function to process the image and detect AR tags.
        """
        # Convert the ROS Image message to an OpenCV image
        img = self.br_.imgmsg_to_cv2(msg)

        # Detect AR tags in the grayscale version of the image
        markerCorners, markerIDs, _ = self.DETECTOR_.detectMarkers(cv.cvtColor(img, cv.COLOR_BGR2GRAY))

        if markerIDs is not None:
            detected_positions = []
            detected_ids = []

            # Process each detected marker
            for markerID, corners in zip(markerIDs, markerCorners):
                tag_id = markerID[0]
                pose = self.compute_camera_position(corners)
                detected_positions.append(pose)
                detected_ids.append(tag_id)

            # Compute the average position
            avg_position = np.mean(detected_positions, axis=0)

            # Create the custom message
            detection_msg = TagDetection()
            detection_msg.id = -1  # Use -1 to indicate it's an average position
            detection_msg.position.header.stamp = self.get_clock().now().to_msg()
            detection_msg.position.header.frame_id = 'camera'
            detection_msg.position.pose.position.x = avg_position[0]
            detection_msg.position.pose.position.y = avg_position[1]
            detection_msg.position.pose.position.z = avg_position[2]

            # Optionally log the detected IDs and average position
            self.get_logger().info(f"Detected AR Tags: {detected_ids}")
            self.get_logger().info(f"Average Position: {avg_position}")

            # Publish the detection message
            self.detection_pub.publish(detection_msg)
        else:
            self.get_logger().info("No AR Tags detected.")

    def compute_camera_position(self, corners):
        """
        Compute the position of the camera (drone) relative to the detected AR tag.
        """
        corners = np.array(corners, dtype=np.float32)
        tag_corners_3d = np.array([
            [-self.tag_size / 2, -self.tag_size / 2, 0],
            [self.tag_size / 2, -self.tag_size / 2, 0],
            [self.tag_size / 2, self.tag_size / 2, 0],
            [-self.tag_size / 2, self.tag_size / 2, 0]
        ], dtype=np.float32)

        # Solve for the pose of the tag
        _, rvec, tvec = cv.solvePnP(tag_corners_3d, corners, self.camera_matrix, self.dist_coeffs)
        
        # tvec is the translation vector representing the position of the camera in the tag's coordinate system
        camera_position = tvec.flatten()

        return tuple(camera_position)


def main(args=None):
    rclpy.init(args=args)
    node = ARTagDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
