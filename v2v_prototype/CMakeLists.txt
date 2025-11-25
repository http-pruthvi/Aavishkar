cmake_minimum_required(VERSION 3.8)
project(v2v_prototype)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BasicSafetyMessage.msg"
)

add_executable(bsm_publisher src/bsm_publisher.cpp)
ament_target_dependencies(bsm_publisher rclcpp std_msgs)
rosidl_target_interfaces(bsm_publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(bsm_listener src/bsm_listener.cpp)
ament_target_dependencies(bsm_listener rclcpp std_msgs)
rosidl_target_interfaces(bsm_listener ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  bsm_publisher
  bsm_listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
