#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import cv2
import depthai as dai
import math
import numpy as np

class DepthPublisher(Node):
    def __init__(self):
        super().__init__('depth_publisher')
        self.center_publisher = self.create_publisher(Float32, 'roi_center_distance', 10)
        self.left_publisher = self.create_publisher(Float32, 'roi_left_distance', 10)
        self.right_publisher = self.create_publisher(Float32, 'roi_right_distance', 10)
        self.top_publisher = self.create_publisher(Float32, 'roi_top_distance', 10)
        self.bottom_publisher = self.create_publisher(Float32, 'roi_bottom_distance', 10)
        self.bool_publisher = self.create_publisher(Bool, 'roi_center_within_1m', 10)

        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutSpatialData = pipeline.create(dai.node.XLinkOut)
        xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

        xoutDepth.setStreamName("depth")
        xoutSpatialData.setStreamName("spatialData")
        xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

        # Properties
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        spatialLocationCalculator.inputConfig.setWaitForMessage(False)

        # Create ROIs with a larger center box and peripheral side boxes
        rois = [
            (0.25, 0.25, 0.75, 0.75),  # Center
            (0.0, 0.25, 0.2, 0.75),  # Left
            (0.8, 0.25, 1.0, 0.75),  # Right
            (0.25, 0.0, 0.75, 0.2),  # Top
            (0.25, 0.8, 0.75, 1.0)   # Bottom
        ]

        for roi in rois:
            config = dai.SpatialLocationCalculatorConfigData()
            config.depthThresholds.lowerThreshold = 200
            config.depthThresholds.upperThreshold = 10000
            config.roi = dai.Rect(dai.Point2f(roi[0], roi[1]), dai.Point2f(roi[2], roi[3]))
            spatialLocationCalculator.initialConfig.addROI(config)

        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
        stereo.depth.link(spatialLocationCalculator.inputDepth)

        spatialLocationCalculator.out.link(xoutSpatialData.input)
        xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

        # Connect to device and start pipeline
        self.device = dai.Device(pipeline)
        self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        self.spatialCalcQueue = self.device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        inDepth = self.depthQueue.get()  # Blocking call, will wait until new data has arrived
        depthFrame = inDepth.getFrame()  # depthFrame values are in millimeters

        depth_downscaled = depthFrame[::4]
        if np.all(depth_downscaled == 0):
            min_depth = 0  # Set a default minimum depth value when all elements are zero
        else:
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
        max_depth = np.percentile(depth_downscaled, 99)
        depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        spatialData = self.spatialCalcQueue.get().getSpatialLocations()
        distances = []
        for i, depthData in enumerate(spatialData):
            roi = depthData.config.roi
            roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])

            xmin = int(roi.topLeft().x)
            ymin = int(roi.topLeft().y)
            xmax = int(roi.bottomRight().x)
            ymax = int(roi.bottomRight().y)

            coords = depthData.spatialCoordinates
            distance = math.sqrt(coords.x ** 2 + coords.y ** 2 + coords.z ** 2)
            distances.append(distance / 1000.0)  # Convert to meters

            cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), (0, 200, 40), thickness=2)
            cv2.putText(depthFrameColor, "{:.1f}m".format(distance / 1000.0), (xmin + 10, ymin + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (0, 200, 40))

            # Publish distances to respective topics
            msg = Float32()
            msg.data = distance / 1000.0  # Convert to meters

            if i == 0:  # Center
                self.center_publisher.publish(msg)
                boolean_msg = Bool()
                boolean_msg.data = bool(distance / 1000.0 <= 1.0)  # True if distance is 1 meter or less
                self.bool_publisher.publish(boolean_msg)
                self.get_logger().info(f"Object within 1m: {boolean_msg.data}")
            elif i == 1:  # Left
                self.left_publisher.publish(msg)
            elif i == 2:  # Right
                self.right_publisher.publish(msg)
            elif i == 3:  # Top
                self.top_publisher.publish(msg)
            elif i == 4:  # Bottom
                self.bottom_publisher.publish(msg)

        # Show the frame
        cv2.imshow("depth", depthFrameColor)

        if cv2.waitKey(1) == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    depth_publisher = DepthPublisher()
    rclpy.spin(depth_publisher)
    depth_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()