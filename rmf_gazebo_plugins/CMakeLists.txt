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
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_door_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(building_map_msgs REQUIRED)
find_package(rmf_traffic REQUIRED)

include(GNUInstallDirs)

add_library(rmf_sim_utils SHARED src/utils.cpp)

target_include_directories(rmf_sim_utils
  PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
)

###############################
# Slotcar Plugin
###############################

add_library(slotcar SHARED ${PROJECT_SOURCE_DIR}/src/slotcar.cpp)

target_link_libraries(slotcar
  PUBLIC
    rmf_sim_utils
    ${rmf_fleet_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${gazebo_ros_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
)

target_include_directories(slotcar
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

###############################
# Door Plugin
###############################

add_library(door SHARED src/door.cpp)

target_link_libraries(door
  PUBLIC
    rmf_sim_utils
    ${rmf_fleet_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${gazebo_ros_LIBRARIES}
    ${rmf_door_msgs_LIBRARIES}
)

target_include_directories(door
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
)

###############################
# Teleport Plugin
###############################

add_library(teleport SHARED src/teleport.cpp)

target_link_libraries(teleport
  PUBLIC
    rmf_sim_utils
    ${rclcpp_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${gazebo_ros_LIBRARIES}
    ${rmf_dispenser_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
)

target_include_directories(teleport
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${rmf_dispenser_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

###############################
# Readonly Plugin
###############################

add_library(readonly SHARED ${PROJECT_SOURCE_DIR}/src/readonly.cpp)

target_link_libraries(readonly
  PUBLIC
    rmf_sim_utils
    ${rmf_fleet_msgs_LIBRARIES}
    ${building_map_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${gazebo_ros_LIBRARIES}
    rmf_traffic::rmf_traffic
)

target_include_directories(readonly
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${building_map_msgs_INCLUDE_DIRS}
)

###############################
# Install Targets
###############################

install(
  TARGETS rmf_sim_utils slotcar door teleport readonly
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
