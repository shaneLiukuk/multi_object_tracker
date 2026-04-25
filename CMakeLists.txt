cmake_minimum_required(VERSION 3.8)
project(od_fusion)

if(CMAKE_CXX_STANDARD VERSION_LESS 17)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(calmcar REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)
include_directories(common)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

add_executable(${PROJECT_NAME}_node
  fusion_node.cc
  src/applications/multi_object_tracker_core.cc
  src/lib/tracker_processor.cc
  src/lib/cv_kalman_filter.cc
  src/lib/track.cc
  src/lib/rviz_display.cc
  src/lib/hungarian.cc
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  visualization_msgs
  geometry_msgs
  calmcar
)

ament_add_gtest(test_od_fusion
  test/test_od_fusion.cc
  src/applications/multi_object_tracker_core.cc
  src/lib/tracker_processor.cc
  src/lib/cv_kalman_filter.cc
  src/lib/track.cc
  src/lib/hungarian.cc
)

ament_target_dependencies(test_od_fusion Eigen3)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
