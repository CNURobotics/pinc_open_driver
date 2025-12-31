cmake_minimum_required(VERSION 3.8)
project(pinc_open_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

# Build library
add_library(${PROJECT_NAME} SHARED
  src/gripper.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  hardware_interface::hardware_interface
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
)

# Export plugin description
pluginlib_export_plugin_description_file(hardware_interface pinc_open_driver.xml)

# Install
install(
  DIRECTORY launch meshes rviz urdf config  src
  DESTINATION share/${PROJECT_NAME}
)
# Install the executable to lib/<package_name>
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(PROGRAMS
  scripts/flash_test_gripper
  scripts/get_params
  scripts/pinc_open_driver_control_gui
  scripts/pinc_open_driver_monitor
  scripts/plot_pinc_open_driver_log
  scripts/read_servo
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
