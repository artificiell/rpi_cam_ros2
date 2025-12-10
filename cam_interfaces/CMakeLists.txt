cmake_minimum_required(VERSION 3.8)
project(cam_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Image.srv"
  DEPENDENCIES sensor_msgs
)

ament_package()
