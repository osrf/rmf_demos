cmake_minimum_required(VERSION 3.5)
project(rmf_plugins_common)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(building_map_msgs REQUIRED)

include(GNUInstallDirs)

##################################
# TeleportIngestor Plugin Common #
##################################

add_library(ingestor_common SHARED ${PROJECT_SOURCE_DIR}/src/ingestor_common.cpp)

ament_target_dependencies(ingestor_common 
    rmf_fleet_msgs
    building_map_msgs
    rmf_dispenser_msgs
    rclcpp
    std_msgs
)

target_include_directories(ingestor_common 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################
# TeleportDispenser Plugin
###############################

add_library(dispenser_common SHARED ${PROJECT_SOURCE_DIR}/src/dispenser_common.cpp)

ament_target_dependencies(dispenser_common
    rmf_fleet_msgs
    building_map_msgs
    rmf_dispenser_msgs
    rclcpp
    std_msgs
)

target_include_directories(dispenser_common 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################
# Install Targets
###############################

ament_export_include_directories(include)
ament_export_libraries(ingestor_common)
ament_export_libraries(dispenser_common)

install(
  TARGETS ingestor_common dispenser_common
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_package()
