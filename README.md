# LiDAR-Camera Projection Node

This ROS 2 node projects 3D LiDAR point cloud data onto a 2D camera image using camera calibration information. It visually fuses the data streams to aid in calibration validation and multi-sensor fusion tasks.

## 📆 Package Overview

**Node Name**: `calibration_projection_node`  
**Main Script**: `calibration_projection_node.py`  
**Topics Subscribed**:
- `/velodyne_points` (`sensor_msgs/PointCloud2`)
- `/zed/zed_node/rgb/image_rect_color` (`sensor_msgs/Image`)
- `/zed/zed_node/rgb/camera_info` (`sensor_msgs/CameraInfo`)

**Topics Published**:
- `/fused_image` (`sensor_msgs/Image`) - Image with projected LiDAR points
- `/camera_info` (`sensor_msgs/CameraInfo`) - Republished camera info

## 🧠 Dependencies

Make sure the following dependencies are installed:

- ROS 2 (Humble/Foxy or compatible)
- OpenCV (`cv2`)
- NumPy
- `cv_bridge`
- Custom utility modules:
  - `lidar_camera_calibration.utils.ros2_numpy`
  - `lidar_camera_calibration.utils.image_geometry.PinholeCameraModel`

> 🔧 Ensure your workspace has the `lidar_camera_calibration` package or those utilities available in your Python path.

## 🛠️ How It Works

1. Subscribes to a LiDAR point cloud, a rectified camera image, and camera intrinsics.
2. Projects each 3D LiDAR point into the 2D camera frame using `PinholeCameraModel`.
3. Draws a green dot at the corresponding 2D point.
4. Publishes the fused image on `/fused_image`.

## 🚀 How to Run

Make sure you source your ROS 2 workspace:
```bash
source install/setup.bash
```

Run the node:
```bash
ros2 run <your_package_name> calibration_projection_node.py
```

Make sure the required topics (`/velodyne_points`, `/zed/zed_node/rgb/image_rect_color`, `/zed/zed_node/rgb/camera_info`) are active and publishing.

## 🧪 Sample Launch Setup

You might want to include this node in a launch file alongside your ZED and LiDAR drivers. Example (pseudo-launch format):

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='calibration_projection_node.py',
            name='calibration_projection_node',
            output='screen'
        )
    ])
```

## 🖼️ Output Example

The output `/fused_image` topic will contain the original camera image with green dots representing projected LiDAR points. You can view it using:

```bash
ros2 run rqt_image_view rqt_image_view
```
And select the `/fused_image` topic.

## 🧹 Troubleshooting

- **No points drawn?**
  - Ensure all topics are publishing.
  - Check camera calibration (`camera_info`) is valid.
- **Incorrect alignment?**
  - Calibration may be off — check extrinsics.
- **Node crashing?**
  - Check for missing imports or malformed point cloud data.

