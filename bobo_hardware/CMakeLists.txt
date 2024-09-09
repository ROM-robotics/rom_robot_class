cmake_minimum_required(VERSION 3.8)
project(bobo_hardware)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find dependencies
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  bobo_hardware
  SHARED
  hardware/diffbot_system.cpp
  controllers/gpio_controller.cpp
)

target_compile_features(bobo_hardware PUBLIC cxx_std_17)
target_include_directories(bobo_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
$<INSTALL_INTERFACE:include/bobo_hardware>
)

ament_target_dependencies(
  bobo_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(bobo_hardware PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "bobo_hardware_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface bobo_hardware.xml)
pluginlib_export_plugin_description_file(controller_interface bobo_hardware.xml)


# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/bobo_hardware
)

install(DIRECTORY
	launch
  config
	DESTINATION share/${PROJECT_NAME}
)

install(TARGETS bobo_hardware
  EXPORT export_bobo_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_interface REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_bobo_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
