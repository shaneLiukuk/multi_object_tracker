cmake_minimum_required(VERSION 3.5)
project(calmcar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

file(
  GLOB
  MSG_FILES
  RELATIVE
  ${CMAKE_CURRENT_SOURCE_DIR}
  "msg/*.msg"
)

#message("[ROS2] ${MSG_FILES}")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
)

install(
  FILES my_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
