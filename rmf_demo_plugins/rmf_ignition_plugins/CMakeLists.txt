cmake_minimum_required(VERSION 3.5)
project(rmf_ignition_plugins)

set (CMAKE_AUTOMOC ON)

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
set(IGN_MATH_VER 6)
ign_find_package(ignition-gui4 REQUIRED)
set(IGN_GUI_VER 4)
ign_find_package(ignition-msgs6 REQUIRED)
set(IGN_MSGS_VER 6)
ign_find_package(ignition-transport9 REQUIRED)
set(IGN_TRANSPORT_VER 9)
ign_find_package(ignition-rendering4 REQUIRED)
set(IGN_RENDERING_VER 4)
ign_find_package(sdformat10 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(rmf_ingestor_msgs REQUIRED)
find_package(building_map_msgs REQUIRED)
find_package(rmf_plugins_common REQUIRED)
find_package (Qt5
  COMPONENTS
    Core
    Qml
    Quick
  REQUIRED
)

include(GNUInstallDirs)

###############################
# TeleportIngestor Plugin
###############################

add_library(teleport_ingestor SHARED src/TeleportIngestor.cpp)

ament_target_dependencies(teleport_ingestor
    ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}
    ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}
    rmf_fleet_msgs
    rmf_ingestor_msgs
    rclcpp
    rmf_plugins_common
    Eigen3
)

target_include_directories(teleport_ingestor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
)

###############################
# TeleportDispenser Plugin
###############################

add_library(teleport_dispenser SHARED src/TeleportDispenser.cpp)

ament_target_dependencies(teleport_dispenser
    ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}
    ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}
    rmf_fleet_msgs
    rmf_dispenser_msgs
    rclcpp
    rmf_plugins_common
    Eigen3
)

target_include_directories(teleport_dispenser
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
)

###############################
# Readonly Plugin             #
###############################

add_library(readonly SHARED ${PROJECT_SOURCE_DIR}/src/readonly.cpp)

ament_target_dependencies(readonly
    rmf_fleet_msgs
    building_map_msgs
    rclcpp
    ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}
    rmf_plugins_common
    Eigen3
)

target_include_directories(readonly
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
)

###############################
# LightTuning
###############################

QT5_ADD_RESOURCES(resources_RCC src/LightTuning/LightTuning.qrc)

add_library(LightTuning SHARED ${headers_MOC}
  src/LightTuning/LightTuning.cpp
  ${resources_RCC}
)

ament_target_dependencies(LightTuning
  ignition-gui${IGN_GUI_VER}
  ignition-gazebo${IGN_GAZEBO_VER}
  ignition-msgs${IGN_MSGS_VER}
  ignition-transport${IGN_TRANSPORT_VER}
  ignition-rendering${IGN_RENDERING_VER}
  Qt5Core
  Qt5Qml
  Qt5Quick
  rclcpp
)

target_include_directories(LightTuning
  PUBLIC
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
)

###############################
# Install Targets
###############################

install(
  TARGETS 
    teleport_dispenser
    teleport_ingestor
    readonly
    LightTuning
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
