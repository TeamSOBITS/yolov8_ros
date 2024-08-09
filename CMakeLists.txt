cmake_minimum_required(VERSION 3.10.2)
project(yolov8_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Install Python scripts
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts as executables
install(PROGRAMS
${PROJECT_NAME}/detect_ros.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch weights
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
