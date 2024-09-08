# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_stm32f4_system_interface_gpio_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED stm32f4_system_interface_gpio_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(stm32f4_system_interface_gpio_FOUND FALSE)
  elseif(NOT stm32f4_system_interface_gpio_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(stm32f4_system_interface_gpio_FOUND FALSE)
  endif()
  return()
endif()
set(_stm32f4_system_interface_gpio_CONFIG_INCLUDED TRUE)

# output package information
if(NOT stm32f4_system_interface_gpio_FIND_QUIETLY)
  message(STATUS "Found stm32f4_system_interface_gpio: 0.0.0 (${stm32f4_system_interface_gpio_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'stm32f4_system_interface_gpio' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${stm32f4_system_interface_gpio_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(stm32f4_system_interface_gpio_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${stm32f4_system_interface_gpio_DIR}/${_extra}")
endforeach()
