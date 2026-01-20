cmake_minimum_required(VERSION 3.9)
project(fosdem26_autoapms_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/RobotVelocity.msg"
    "msg/RobotPosition.msg"
)

ament_package()
