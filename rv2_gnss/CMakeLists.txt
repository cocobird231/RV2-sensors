cmake_minimum_required(VERSION 3.8)
project(rv2_gnss)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rv2_interfaces REQUIRED)

# Require pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPS REQUIRED libgps)

set(DEPENDENCIES
    rcl
    rclcpp
    rclcpp_components
    sensor_msgs
    rv2_interfaces
)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

include_directories(${GPS_INCLUDE_DIRS})

link_directories(${GPS_LIBRARY_DIRS})



# Build

## Component

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# rv2_camera Component
add_library(rv2_gnss_component
  SHARED src/gnss_component.cpp
)
target_compile_definitions(rv2_gnss_component
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(rv2_gnss_component
  ${GPS_LIBRARIES}
)
ament_target_dependencies(rv2_gnss_component ${DEPENDENCIES})
rclcpp_components_register_nodes(rv2_gnss_component "rv2_sensors::GNSSComponent")
set(node_plugins "${node_plugins}rv2_sensors::GNSSComponent;$<TARGET_FILE:rv2_gnss_component>\n")

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

## Executable

add_executable(gnss_node src/gnss_node.cpp)
target_link_libraries(gnss_node
  rv2_gnss_component
  ${GPS_LIBRARIES}
)
ament_target_dependencies(gnss_node ${DEPENDENCIES})

# Install

## Component
install(TARGETS
  rv2_gnss_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Executable
install(TARGETS
  gnss_node
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
ament_export_libraries(rv2_gnss_component)
ament_export_dependencies(${DEPENDENCIES})
ament_package()