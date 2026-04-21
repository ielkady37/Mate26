cmake_minimum_required(VERSION 3.5)
project(rov_msgs)  # must match package.xml

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Depth.msg"
  "msg/IMU.msg"
  "msg/Temp.msg"
  "msg/Ms5.msg"
  "msg/Joy.msg"
)

ament_package()