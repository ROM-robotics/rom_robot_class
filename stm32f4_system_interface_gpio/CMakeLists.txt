cmake_minimum_required(VERSION 3.16)
project(stm32f4_system_interface_gpio LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  control_msgs
  std_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  stm32f4_system_interface_gpio
  SHARED
  hardware/diffbot_system.cpp
  controllers/gpio_controller.cpp
)

target_compile_features(stm32f4_system_interface_gpio PUBLIC cxx_std_17)
target_include_directories(stm32f4_system_interface_gpio PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
$<INSTALL_INTERFACE:include/stm32f4_system_interface_gpio>
)

ament_target_dependencies(
  stm32f4_system_interface_gpio PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(stm32f4_system_interface_gpio PUBLIC serial)
# ?? 

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "stm32f4_system_interface_gpio_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface stm32f4_system_interface_gpio.xml)
# Export controllers
pluginlib_export_plugin_description_file(controller_interface stm32f4_system_interface_gpio.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/stm32f4_system_interface_gpio
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/stm32f4_system_interface_gpio
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/stm32f4_system_interface_gpio
)
install(TARGETS stm32f4_system_interface_gpio
  EXPORT export_stm32f4_system_interface_gpio
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_interface REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_stm32f4_system_interface_gpio HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
