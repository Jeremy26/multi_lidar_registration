cmake_minimum_required(VERSION 3.8)
project(lidar_calibration)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(transforms3d REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Open3D REQUIRED)

install(
  PROGRAMS
    lidar_calibration/lidar_calibration/lidar_calibration_node.py
    lidar_calibration/lidar_calibration/uncalibrated_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()  # This should always be the last line
