cmake_minimum_required(VERSION 3.5)
project(rmf_rviz_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

add_definitions(-DQT_NO_KEYWORDS)

set(src_files src/Control.cpp)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${QT5_LIBRARIES} 
    ${rclcpp_LIBRARIES}
    ${rmf_task_msgs_LIBRARIES}
    rviz_common::rviz_common
    ${rviz_rendering_LIBRARIES}
    ${rviz_default_plugins_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
    ${rviz_common_INCLUDE_DIRS}
    ${rviz_rendering_INCLUDE_DIRS}
    ${rviz_default_plugins_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${QT5_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories(include)
ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)
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
  DESTINATION share
)

ament_package()
