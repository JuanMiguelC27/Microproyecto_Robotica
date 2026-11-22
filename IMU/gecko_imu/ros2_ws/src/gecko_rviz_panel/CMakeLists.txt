cmake_minimum_required(VERSION 3.8)
project(gecko_rviz_panel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_library(${PROJECT_NAME} SHARED
  include/gecko_rviz_panel/gecko_panel.hpp
  include/gecko_rviz_panel/joystick_widget.hpp
  src/gecko_panel.cpp
  src/joystick_widget.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME} pluginlib rclcpp rviz_common std_msgs)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_drive_mapping test/test_drive_mapping.cpp)
  target_include_directories(test_drive_mapping PRIVATE include)
endif()

ament_package()
