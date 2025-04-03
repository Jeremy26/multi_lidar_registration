# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_kiss_icp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED kiss_icp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(kiss_icp_FOUND FALSE)
  elseif(NOT kiss_icp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(kiss_icp_FOUND FALSE)
  endif()
  return()
endif()
set(_kiss_icp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT kiss_icp_FIND_QUIETLY)
  message(STATUS "Found kiss_icp: 1.2.3 (${kiss_icp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'kiss_icp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${kiss_icp_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(kiss_icp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${kiss_icp_DIR}/${_extra}")
endforeach()
