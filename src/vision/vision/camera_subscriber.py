import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription1 = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.listener_callback1,
            10)
        self.subscription2 = self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.listener_callback2,
            10)
        self.bridge = CvBridge()
        self.image1 = None
        self.image2 = None
        self.merged_image = None
        self.get_logger().info('CameraSubscriber node has been started')

    def listener_callback1(self, msg):
        self.get_logger().info('Received an image from Camera 1')
        try:
            self.image1 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.get_logger().info(f'Camera 1 image shape: {self.image1.shape}')
        except Exception as e:
            self.get_logger().error(f'Error converting Camera 1 image: {e}')
        self.merge_and_store()

    def listener_callback2(self, msg):
        self.get_logger().info('Received an image from Camera 2')
        try:
            self.image2 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.get_logger().info(f'Camera 2 image shape: {self.image2.shape}')
        except Exception as e:
            self.get_logger().error(f'Error converting Camera 2 image: {e}')
        self.merge_and_store()

    def merge_and_store(self):
        if self.image1 is not None and self.image2 is not None:
            self.get_logger().info('Merging images from Camera 1 and Camera 2')
            try:
                self.merged_image = cv2.hconcat([self.image1, self.image2])
                _, buffer = cv2.imencode('.jpg', self.merged_image)
                self.merged_image = buffer.tobytes()
                self.get_logger().info('Images merged and stored successfully')
            except Exception as e:
                self.get_logger().error(f'Error merging images: {e}')
        else:
            self.get_logger().info('One or both images are None')

    def get_merged_image(self):
        if self.merged_image is None:
            self.get_logger().info('No merged image available')
        return self.merged_image

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
