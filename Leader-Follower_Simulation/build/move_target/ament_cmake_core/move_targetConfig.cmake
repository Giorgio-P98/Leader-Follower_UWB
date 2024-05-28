# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_move_target_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED move_target_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(move_target_FOUND FALSE)
  elseif(NOT move_target_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(move_target_FOUND FALSE)
  endif()
  return()
endif()
set(_move_target_CONFIG_INCLUDED TRUE)

# output package information
if(NOT move_target_FIND_QUIETLY)
  message(STATUS "Found move_target: 0.0.0 (${move_target_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'move_target' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${move_target_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(move_target_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${move_target_DIR}/${_extra}")
endforeach()
