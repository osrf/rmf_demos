cmake_minimum_required(VERSION 3.5)
project(rmf_gazebo_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(building_map_msgs REQUIRED)

include(GNUInstallDirs)

add_library(rmf_sim_utils SHARED src/utils.cpp)

target_include_directories(rmf_sim_utils
  PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
)

###############################
# TeleportDispenser Plugin
###############################

add_library(teleport_dispenser SHARED src/TeleportDispenser.cpp)

ament_target_dependencies(teleport_dispenser
  rclcpp
  gazebo_ros
  rmf_fleet_msgs
  rmf_dispenser_msgs
)

target_include_directories(teleport_dispenser
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
)

###############################
# TeleportIngestor Plugin
###############################

add_library(teleport_ingestor SHARED src/TeleportIngestor.cpp)

ament_target_dependencies(teleport_ingestor
  rclcpp
  gazebo_ros
  rmf_fleet_msgs
  rmf_dispenser_msgs
)

target_include_directories(teleport_ingestor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
)

###############################
# Readonly Plugin
###############################

add_library(readonly SHARED ${PROJECT_SOURCE_DIR}/src/readonly.cpp)

ament_target_dependencies(readonly
    rmf_fleet_msgs
    building_map_msgs
    rclcpp
    gazebo_ros
)

target_include_directories(readonly
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
)

###############################
# Install Targets
###############################

install(
  TARGETS 
    rmf_sim_utils
    teleport_dispenser
    teleport_ingestor
    readonly
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
