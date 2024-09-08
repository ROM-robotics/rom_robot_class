#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "stm32f4_system_interface_gpio::stm32f4_system_interface_gpio" for configuration "Debug"
set_property(TARGET stm32f4_system_interface_gpio::stm32f4_system_interface_gpio APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(stm32f4_system_interface_gpio::stm32f4_system_interface_gpio PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libstm32f4_system_interface_gpio.so"
  IMPORTED_SONAME_DEBUG "libstm32f4_system_interface_gpio.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS stm32f4_system_interface_gpio::stm32f4_system_interface_gpio )
list(APPEND _IMPORT_CHECK_FILES_FOR_stm32f4_system_interface_gpio::stm32f4_system_interface_gpio "${_IMPORT_PREFIX}/lib/libstm32f4_system_interface_gpio.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
