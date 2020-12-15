cmake_minimum_required(VERSION 3.5)
project(rmf_plugins_common)

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

find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(rmf_ingestor_msgs REQUIRED)
find_package(building_map_msgs REQUIRED)

include(GNUInstallDirs)

###############################
# Utils                       #
###############################

add_library(rmf_sim_utils SHARED ${PROJECT_SOURCE_DIR}/src/utils.cpp)

ament_target_dependencies(rmf_sim_utils
  rclcpp
  Eigen3
)

target_include_directories(rmf_sim_utils
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

##################################
# TeleportIngestor Plugin Common #
##################################

add_library(ingestor_common SHARED ${PROJECT_SOURCE_DIR}/src/ingestor_common.cpp)

ament_target_dependencies(ingestor_common 
    rmf_fleet_msgs
    building_map_msgs
    rmf_ingestor_msgs
    rclcpp
    std_msgs
    Eigen3
)

target_include_directories(ingestor_common 
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###################################
# TeleportDispenser Plugin Common #
###################################

add_library(dispenser_common SHARED ${PROJECT_SOURCE_DIR}/src/dispenser_common.cpp)

ament_target_dependencies(dispenser_common
    rmf_fleet_msgs
    building_map_msgs
    rmf_dispenser_msgs
    rclcpp
    std_msgs
    Eigen3
)

target_include_directories(dispenser_common 
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###################################
# Readonly Plugin Common #
###################################

add_library(readonly_common SHARED ${PROJECT_SOURCE_DIR}/src/readonly_common.cpp)

ament_target_dependencies(readonly_common
    rmf_fleet_msgs
    building_map_msgs
    rclcpp
    std_msgs
    Eigen3
)

target_include_directories(readonly_common
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################
# Install Targets             #
###############################
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)
ament_export_targets(ingestor_common HAS_LIBRARY_TARGET)
ament_export_targets(dispenser_common HAS_LIBRARY_TARGET)
ament_export_targets(readonly_common HAS_LIBRARY_TARGET)
ament_export_targets(rmf_sim_utils HAS_LIBRARY_TARGET)

install(
  TARGETS ingestor_common
  EXPORT ingestor_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS dispenser_common
  EXPORT dispenser_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS readonly_common
  EXPORT readonly_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS rmf_sim_utils
  EXPORT rmf_sim_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
