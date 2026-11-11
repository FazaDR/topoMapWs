cmake_minimum_required(VERSION 3.8)
project(preprocess)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ProcessedScan.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

add_executable(preprocess_node
  src/preprocess_main.cpp
  src/preprocess_node.cpp
)

target_include_directories(preprocess_node PRIVATE include)

ament_target_dependencies(preprocess_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(preprocess_node "${cpp_typesupport_target}")

target_link_libraries(preprocess_node
  gz-transport13::core
  gz-msgs10::core
  Eigen3::Eigen
)

install(TARGETS
  preprocess_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
