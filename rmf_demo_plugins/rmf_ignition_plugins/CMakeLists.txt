cmake_minimum_required(VERSION 3.5)
project(rmf_ignition_plugins)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ignition-cmake2 QUIET)
if (NOT ignition-cmake2_FOUND)
  message("ignition-cmake not found, skipping ignition gazebo plugins")
  return()
endif()
ign_find_package(ignition-gazebo4 QUIET)
if (NOT ignition-gazebo4_FOUND)
  message("Wrong ignition-gazebo version found. Plugins require ignition-gazebo4 (dome)")
  # To generate a setup.bash and avoid CMake warnings
  ament_package()
  return()
endif()
set(IGN_GAZEBO_VER 4)
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER 1)
ign_find_package(ignition-common3 REQUIRED)
set(IGN_COMMON_VER 3)
ign_find_package(ignition-math6 REQUIRED)
ign_find_package(sdformat10 REQUIRED)
set(IGN_MATH_VER 6)

find_package(std_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(rmf_ingestor_msgs REQUIRED)
find_package(building_map_msgs REQUIRED)
find_package(rmf_plugins_common REQUIRED)

include(GNUInstallDirs)

###############################
# TeleportIngestor Plugin
###############################

add_library(teleport_ingestor SHARED src/TeleportIngestor.cpp)

ament_target_dependencies(teleport_ingestor
    ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}
    rmf_fleet_msgs
    rmf_ingestor_msgs
    rclcpp
    rmf_plugins_common
    Eigen3
)

target_include_directories(teleport_ingestor
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################
# TeleportDispenser Plugin
###############################

add_library(teleport_dispenser SHARED src/TeleportDispenser.cpp)

ament_target_dependencies(teleport_dispenser
    ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}
    rmf_fleet_msgs
    rmf_dispenser_msgs
    rclcpp
    rmf_plugins_common
)

target_include_directories(teleport_dispenser
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################
# Readonly Plugin             #
###############################

add_library(readonly SHARED ${PROJECT_SOURCE_DIR}/src/readonly.cpp)

target_link_libraries(readonly
  PUBLIC
    ${rmf_fleet_msgs_LIBRARIES}
    ${building_map_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
    #${GAZEBO_LIBRARIES}
    #${gazebo_ros_LIBRARIES}
    ${rmf_plugins_common_LIBRARIES}
)

target_include_directories(readonly
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    #${GAZEBO_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${rmf_plugins_common_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${building_map_msgs_INCLUDE_DIRS}
)

###############################
# Install Targets
###############################

install(
  TARGETS 
    teleport_dispenser
    teleport_ingestor
    readonly
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
