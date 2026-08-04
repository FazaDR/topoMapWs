cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(preprocess_node
  src/main.cpp
  src/preprocess_node.cpp
)

ament_target_dependencies(preprocess_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(preprocess_node
  gz-transport13::core
  gz-msgs10::core
  ${PCL_LIBRARIES}
)

target_link_libraries(preprocess_node
    Eigen3::Eigen
)

install(TARGETS
  preprocess_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()