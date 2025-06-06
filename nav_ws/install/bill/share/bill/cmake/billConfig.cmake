# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bill_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bill_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bill_FOUND FALSE)
  elseif(NOT bill_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bill_FOUND FALSE)
  endif()
  return()
endif()
set(_bill_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bill_FIND_QUIETLY)
  message(STATUS "Found bill: 0.0.0 (${bill_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bill' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT bill_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bill_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${bill_DIR}/${_extra}")
endforeach()
