cmake_minimum_required(VERSION 3.16)
project(rightbot_config_pkg LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rightbot_hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/rightbot_config_pkg
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/rightbot_config_pkg
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/rightbot_config_pkg
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
