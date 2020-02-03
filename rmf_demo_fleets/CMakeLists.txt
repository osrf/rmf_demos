cmake_minimum_required(VERSION 3.5)

project(rmf_demo_fleets)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)

# -----------------------------------------------------------------------------

add_executable(fake_fleet
  src/FakeFleet.cpp
)
ament_target_dependencies(fake_fleet rclcpp rmf_fleet_msgs)

# -----------------------------------------------------------------------------

install(
  TARGETS
    fake_fleet
  RUNTIME DESTINATION lib/rmf_demo_fleets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
