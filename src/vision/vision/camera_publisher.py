import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher1 = self.create_publisher(Image, '/camera1/image_raw', 10)
        self.publisher2 = self.create_publisher(Image, '/camera2/image_raw', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()

        # Initialize cameras
        self.cap1 = cv2.VideoCapture(0)
        self.cap2 = cv2.VideoCapture(2)

        if not self.cap1.isOpened():
            self.get_logger().error('Failed to open Camera 1')
        if not self.cap2.isOpened():
            self.get_logger().error('Failed to open Camera 2')

    def timer_callback(self):
        ret1, frame1 = self.cap1.read()
        if ret1:
            img_msg1 = self.bridge.cv2_to_imgmsg(frame1, 'bgr8')
            self.publisher1.publish(img_msg1)
        else:
            self.get_logger().warn('Failed to capture image from Camera 1')

        ret2, frame2 = self.cap2.read()
        if ret2:
            img_msg2 = self.bridge.cv2_to_imgmsg(frame2, 'bgr8')
            self.publisher2.publish(img_msg2)
        else:
            self.get_logger().warn('Failed to capture image from Camera 2')

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
