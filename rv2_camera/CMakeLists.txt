cmake_minimum_required(VERSION 3.8)
project(rv2_camera)

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
find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)

set(DEPENDENCIES
    rclcpp
    rclcpp_components
    sensor_msgs
    rv2_interfaces
)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIBRARY_DIRS})



# Build

## Component

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# rv2_camera Component
add_library(rv2_camera_component
  SHARED src/camera_component.cpp
)
target_compile_definitions(rv2_camera_component
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(rv2_camera_component
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(rv2_camera_component ${DEPENDENCIES})
rclcpp_components_register_nodes(rv2_camera_component "rv2_sensors::CameraComponent")
set(node_plugins "${node_plugins}rv2_sensors::CameraComponent;$<TARGET_FILE:rv2_camera_component>\n")

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

## Executable

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node
  rv2_camera_component
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(camera_node ${DEPENDENCIES})


# Install

## Component
install(TARGETS
  rv2_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Executable
install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

## Include
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch params scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(rv2_camera_component)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
