cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(preprocess REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)

include_directories(include ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(mlatc
  src/mlatc.cpp
)
target_include_directories(mlatc PUBLIC include)
target_link_libraries(mlatc Eigen3::Eigen)

add_executable(topology_node
    src/topology_main.cpp
    src/topology_node.cpp
    src/atcdt.cpp
)

add_executable(mlatc_node
  src/mlatc_node.cpp
  src/mlatc_main.cpp
)

add_executable(atcdt_free_node
  src/atcdt_free_node.cpp
  src/atcdt_free_main.cpp
  src/atcdt.cpp
)

add_executable(mlatc_free_node
  src/mlatc_free_node.cpp
  src/mlatc_free_main.cpp
)

target_include_directories(mlatc_node PRIVATE include)

ament_target_dependencies(topology_node
    rclcpp
    preprocess
    sensor_msgs
    visualization_msgs
)

ament_target_dependencies(mlatc_node
    rclcpp
    preprocess
    sensor_msgs
    visualization_msgs
)

ament_target_dependencies(atcdt_free_node
    rclcpp
    preprocess
    sensor_msgs
    visualization_msgs
)

ament_target_dependencies(mlatc_free_node
    rclcpp
    preprocess
    sensor_msgs
    geometry_msgs
    visualization_msgs
)

target_link_libraries(topology_node
    Eigen3::Eigen
    preprocess::preprocess__rosidl_typesupport_cpp
    preprocess::preprocess__rosidl_generator_c
)

target_link_libraries(mlatc_node
    mlatc
    Eigen3::Eigen
    preprocess::preprocess__rosidl_typesupport_cpp
    preprocess::preprocess__rosidl_generator_c
)

target_link_libraries(atcdt_free_node
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    preprocess::preprocess__rosidl_typesupport_cpp
    preprocess::preprocess__rosidl_generator_c
)

target_link_libraries(mlatc_free_node
    mlatc
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    preprocess::preprocess__rosidl_typesupport_cpp
    preprocess::preprocess__rosidl_generator_c
)

install(TARGETS
    mlatc
    topology_node
    mlatc_node
    atcdt_free_node
    mlatc_free_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()