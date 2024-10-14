cmake_minimum_required(VERSION 3.8)
project(lidar_camera_calibration)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python package (this assumes your Python package structure is correct)
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
lidar_camera_calibration/calibration.py
lidar_camera_calibration/calibration_projection_node.py
lidar_camera_calibration/object_detection.py
  DESTINATION lib/${PROJECT_NAME}  # This ensures that the executables are placed in the right directory
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
