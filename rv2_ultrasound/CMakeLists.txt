cmake_minimum_required(VERSION 3.8)
project(rv2_ultrasound)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Check ROS_DISTRO
set(MY_ROS2_DISTRO $ENV{ROS_DISTRO})
if(${MY_ROS2_DISTRO} STREQUAL "foxy")
  add_definitions(-DFOUND_FOXY)
elseif(${MY_ROS2_DISTRO} STREQUAL "humble")
  add_definitions(-DFOUND_HUMBLE)
else()
  message(FATAL_ERROR "ROS_DISTRO is not found or not supported.")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rv2_interfaces REQUIRED)

set(DEPENDENCIES
    rclcpp
    rclcpp_components
    sensor_msgs
    rv2_interfaces
)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Build

## Component

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# rv2_camera Component
add_library(rv2_ultrasound_component SHARED
  src/serial_module.cpp
  src/ultrasound_component.cpp
)
target_compile_definitions(rv2_ultrasound_component
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(rv2_ultrasound_component ${DEPENDENCIES})
rclcpp_components_register_nodes(rv2_ultrasound_component "rv2_sensors::UltrasoundComponent")
set(node_plugins "${node_plugins}rv2_sensors::UltrasoundComponent;$<TARGET_FILE:rv2_ultrasound_component>\n")

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

## Executable

add_executable(ultrasound_node src/ultrasound_node.cpp)
target_link_libraries(ultrasound_node
  rv2_ultrasound_component
)
ament_target_dependencies(ultrasound_node ${DEPENDENCIES})

# Install

## Component
install(TARGETS
  rv2_ultrasound_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Executable
install(TARGETS
  ultrasound_node
  DESTINATION lib/${PROJECT_NAME}
)

## Include
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

## Files
install(DIRECTORY
  launch params scripts
  DESTINATION share/${PROJECT_NAME}
)

## Requirements
install(FILES
  requirements_apt.txt
  requirements_pip.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(rv2_ultrasound_component)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
