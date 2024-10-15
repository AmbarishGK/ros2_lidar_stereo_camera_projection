#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from lidar_camera_calibration.utils.image_geometry import PinholeCameraModel
from ultralytics import YOLO

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        self.subscription_image = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10
        )
        self.subscription_camera_info = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/camera_info',
            self.camera_info_callback,
            10
        )
        self.bridge = CvBridge()
        self.camera_image = None
        self.camera_info  = None
        self.pinhole_camera_model = PinholeCameraModel()
        self.model = YOLO("yolo11n.pt")

    def image_callback(self, msg):
        # Process camera image
        self.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def camera_info_callback(self, msg):
        # Process camera info
        self.camera_info = msg
        # Update PinholeCameraModel with camera info
        self.pinhole_camera_model.fromCameraInfo(self.camera_info)

    def objectDetection(self):
        if self.camera_image is None or self.camera_info is None:
            return
        results = self.model(np.array(self.camera_image))  # return a list of Results objects

        # Process results list
        for result in results:
            boxes = result.boxes  # Boxes object for bounding box outputs
            masks = result.masks  # Masks object for segmentation masks outputs
            keypoints = result.keypoints  # Keypoints object for pose outputs
            probs = result.probs  # Probs object for classification outputs
            obb = result.obb  # Oriented boxes object for OBB outputs
            result.show()  # display to screen
            result.save(filename="result.jpg")  # save to disk



def main(args=None):
    rclpy.init(args=args)
    object_detection = ObjectDetection()
    try:
        while True:
            object_detection.objectDetection()
            rclpy.spin_once(object_detection)
    except KeyboardInterrupt:
        object_detection.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
