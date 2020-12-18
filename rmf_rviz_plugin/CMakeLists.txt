cmake_minimum_required(VERSION 3.5)
project(rmf_rviz_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_door_msgs REQUIRED)
find_package(rmf_lift_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Test)
find_package(std_msgs REQUIRED)

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME} SHARED src/RmfPanel.cpp src/ParseGraph.cpp src/ParseActionPlan.cpp)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${QT5_LIBRARIES} 
    ${rclcpp_LIBRARIES}
    ${rmf_task_msgs_LIBRARIES}
    ${rmf_fleet_msgs_LIBRARIES}
    ${rmf_door_msgs_LIBRARIES}
    ${rmf_lift_msgs_LIBRARIES}
    ${rmf_dispenser_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
    rviz_common::rviz_common
    rviz_default_plugins::rviz_default_plugins
    rmf_traffic::rmf_traffic
    ${rviz_rendering_LIBRARIES}
    ${rviz_default_plugins_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${std_msgs_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
    ${rmf_lift_msgs_INCLUDE_DIRS}
    ${rmf_traffic_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${rmf_dispenser_msgs_INCLUDE_DIRS}
    ${rviz_common_INCLUDE_DIRS}
    ${rviz_rendering_INCLUDE_DIRS}
    ${rviz_default_plugins_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${QT5_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  Qt5
  rviz_common
  rclcpp
  Qt5Widgets
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/
)

ament_package()
