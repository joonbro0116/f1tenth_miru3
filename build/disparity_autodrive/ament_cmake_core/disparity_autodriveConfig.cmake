# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_disparity_autodrive_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED disparity_autodrive_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(disparity_autodrive_FOUND FALSE)
  elseif(NOT disparity_autodrive_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(disparity_autodrive_FOUND FALSE)
  endif()
  return()
endif()
set(_disparity_autodrive_CONFIG_INCLUDED TRUE)

# output package information
if(NOT disparity_autodrive_FIND_QUIETLY)
  message(STATUS "Found disparity_autodrive: 1.0.0 (${disparity_autodrive_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'disparity_autodrive' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${disparity_autodrive_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(disparity_autodrive_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${disparity_autodrive_DIR}/${_extra}")
endforeach()
