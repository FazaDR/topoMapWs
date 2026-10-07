cmake_minimum_required(VERSION 3.8)
project(mapping_mlatc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(mlatc
  src/mlatc.cpp
)
target_include_directories(mlatc PUBLIC include)
target_link_libraries(mlatc Eigen3::Eigen)

add_executable(mlatc_node
  src/mlatc_node.cpp
  src/mlatc_main.cpp
)
target_include_directories(mlatc_node PRIVATE include)
ament_target_dependencies(mlatc_node rclcpp sensor_msgs visualization_msgs)
target_link_libraries(mlatc_node mlatc Eigen3::Eigen)

install(TARGETS mlatc mlatc_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
