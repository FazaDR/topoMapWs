cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GZ_TRANSPORT REQUIRED gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED gz-msgs10)

include_directories(
  include
  ${GZ_TRANSPORT_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
)

link_directories(
  ${GZ_TRANSPORT_LIBRARY_DIRS}
  ${GZ_MSGS_LIBRARY_DIRS}
)

add_executable(controller_node
  src/main.cpp
  src/controller_node.cpp
)

ament_target_dependencies(controller_node
  rclcpp
  geometry_msgs
)

target_link_libraries(controller_node
  ${GZ_TRANSPORT_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
)

install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
