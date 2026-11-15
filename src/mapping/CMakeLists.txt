cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(preprocess REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

# ---------------- core library ----------------
add_library(atc_core SHARED
  src/core/topological_map.cpp
  src/core/atc_layer.cpp
  src/core/atc.cpp
  src/core/free_area.cpp
  src/core/free_area_kdtree.cpp
  src/core/hierarchy.cpp
  src/core/mlatc.cpp
)


target_include_directories(atc_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(atc_core PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(atc_core
  PUBLIC Eigen3::Eigen
  PRIVATE ${PCL_LIBRARIES}
)

# ---------------- node ----------------
add_executable(mapping_node
  src/mapping_node.cpp
  src/mapping_main.cpp
)
target_include_directories(mapping_node PRIVATE include)
target_link_libraries(mapping_node
  atc_core
  Eigen3::Eigen
  preprocess::preprocess__rosidl_typesupport_cpp
  preprocess::preprocess__rosidl_generator_c
)

ament_target_dependencies(mapping_node
  rclcpp
  preprocess
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  rosbag2_cpp
  rosbag2_storage
)

# ---------------- install ----------------
install(TARGETS atc_core
  EXPORT export_atc_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mapping_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_atc_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()