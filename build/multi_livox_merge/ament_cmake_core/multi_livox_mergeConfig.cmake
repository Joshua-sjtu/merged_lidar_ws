# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_multi_livox_merge_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED multi_livox_merge_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(multi_livox_merge_FOUND FALSE)
  elseif(NOT multi_livox_merge_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(multi_livox_merge_FOUND FALSE)
  endif()
  return()
endif()
set(_multi_livox_merge_CONFIG_INCLUDED TRUE)

# output package information
if(NOT multi_livox_merge_FIND_QUIETLY)
  message(STATUS "Found multi_livox_merge: 0.0.0 (${multi_livox_merge_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'multi_livox_merge' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${multi_livox_merge_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(multi_livox_merge_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${multi_livox_merge_DIR}/${_extra}")
endforeach()
