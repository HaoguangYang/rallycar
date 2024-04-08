cmake_minimum_required(VERSION 3.8)
project(rallycar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(rallycar_driver_lib SHARED
  src/serial_impl.cc
  src/serial.cc
  src/rallycar_driver_node.cpp
)

rclcpp_components_register_node(rallycar_driver_lib
  PLUGIN "rallycar::RallycarDriverNode"
  EXECUTABLE rallycar_driver_node
)

# tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  #ament_add_gtest(${PROJECT_NAME}_gtest test/rallycar_driver_test.cpp)
  #target_link_libraries(${PROJECT_NAME}_gtest ${rclcpp_LIBRARIES} ${rmw_connext_cpp_LIBRARIES} ${std_interfaces} ${TARGET_LIB_NAME})

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)

  # Temporarily disable uncrustify
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
  )

  ament_lint_auto_find_test_dependencies()

  # make ament_uncrustify skip C/C++ files.
  file(GLOB_RECURSE FILES_MINUS_SOME
    ${CMAKE_CURRENT_SOURCE_DIR}/*
  )
  list(FILTER FILES_MINUS_SOME EXCLUDE REGEX ".*\.(cpp|hpp|cc|hh|c|h)")
  # Re-enable uncrustify
  if(FILES_MINUS_SOME)
    find_package(ament_cmake_uncrustify)
    ament_uncrustify(${FILES_MINUS_SOME})
  endif()

endif()

# export information to downstream packages
ament_auto_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
  INSTALL_TO_SHARE resources
)
