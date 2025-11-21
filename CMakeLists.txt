cmake_minimum_required(VERSION 3.8)
project(omni_wheel_controller)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcutils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(realtime_tools REQUIRED)

# Create the controller library
add_library(${PROJECT_NAME} SHARED
  src/omni_wheel_controller.cpp
)

# Specify include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  rcutils
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  realtime_tools
)

# Export plugin description so controller_manager/pluginlib can find the plugin XML after install
pluginlib_export_plugin_description_file(controller_interface plugin_description.xml)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install config files (optional)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install launch files (optional)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)


# Export information
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  rcutils
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  realtime_tools
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
