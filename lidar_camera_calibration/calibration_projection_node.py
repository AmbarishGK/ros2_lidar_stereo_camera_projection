#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from lidar_camera_calibration.utils.ros2_numpy import pointcloud2_to_xyz_array
from lidar_camera_calibration.utils.image_geometry import PinholeCameraModel

class CalibrationProjectionNode(Node):
    def __init__(self):
        super().__init__('calibration_projection_node')

        self.subscription_lidar = self.create_subscription(
            PointCloud2, '/velodyne_points', self.lidar_callback, 10)
        self.subscription_image = self.create_subscription(
            Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, 10)
        self.subscription_camera_info = self.create_subscription(
            CameraInfo, '/zed/zed_node/rgb/camera_info', self.camera_info_callback, 10)

        self.publisher_projected_image = self.create_publisher(Image, '/fused_image', 10)
        self.publisher_camera_info = self.create_publisher(CameraInfo, '/camera_info', 10)

        self.bridge = CvBridge()
        self.lidar_points = None
        self.camera_image = None
        self.camera_info = None
        self.pinhole_camera_model = PinholeCameraModel()

    def lidar_callback(self, msg):
        self.lidar_points = pointcloud2_to_xyz_array(msg)

    def image_callback(self, msg):
        self.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.pinhole_camera_model.fromCameraInfo(msg)

    def project_lidar_on_image(self):
        if self.lidar_points is None or self.camera_image is None or self.camera_info is None:
            return

        projected_image = self.camera_image.copy()
        height, width = projected_image.shape[:2]

        for point in self.lidar_points:
            img_point = self.pinhole_camera_model.project3dToPixel(point)
            x, y = int(img_point[0]), int(img_point[1])

            if 0 <= x < width and 0 <= y < height:
                cv2.circle(projected_image, (x, y), 2, (0, 255, 0), -1)

        self.publish_projected_image(projected_image)

    def publish_projected_image(self, image):
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        image_msg.header.frame_id = self.camera_info.header.frame_id
        image_msg.header.stamp = self.camera_info.header.stamp

        self.publisher_projected_image.publish(image_msg)
        self.publisher_camera_info.publish(self.camera_info)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationProjectionNode()

    try:
        while rclpy.ok():
            node.project_lidar_on_image()
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
