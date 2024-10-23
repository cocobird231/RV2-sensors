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

if($ENV{ROS_DISTRO} STREQUAL "foxy")
  add_compile_options(-DROS_DISTRO=1)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
  add_compile_options(-DROS_DISTRO=2)
else()# eloquent
  add_compile_options(-DROS_DISTRO=0)
endif()

# find dependencies
SET(ENABLE_VS_NODE False CACHE BOOL "Some user-specified option")
if( ENABLE_VS_NODE ) 
  add_compile_options(-DVS_NODE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rv2_interfaces REQUIRED)

# Require pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPS REQUIRED libgps)

set (SRC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gnss_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

include_directories(${GPS_INCLUDE_DIRS})

link_directories(${GPS_LIBRARY_DIRS})

## Build
add_executable(main ${SRC_FILES})
ament_target_dependencies(main rcl rclcpp rv2_interfaces sensor_msgs)
target_link_libraries(main
  ${GPS_LIBRARIES}
)

## Install
install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch params scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
