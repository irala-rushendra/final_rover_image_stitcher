cmake_minimum_required(VERSION 3.8)
project(final_panorama_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. Generate the Action Interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CapturePanorama.action"
  DEPENDENCIES action_msgs
)

# 2. Build the Action Server Executable
add_executable(panorama_action_server src/panorama_action_server.cpp)

ament_target_dependencies(panorama_action_server
  rclcpp
  rclcpp_action
  action_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
)

# 3. Link the Action Interface to the Executable
rosidl_target_interfaces(panorama_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install
install(TARGETS panorama_action_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY action DESTINATION share/${PROJECT_NAME})

ament_package()
