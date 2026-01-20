cmake_minimum_required(VERSION 3.9)
project(fosdem26_autoapms_robot)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install executables
install(PROGRAMS
    scripts/robot_hand_node.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
