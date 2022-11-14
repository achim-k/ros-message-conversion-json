cmake_minimum_required(VERSION 3.8)
project(message_conversion_json)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

add_library(message_json_conversion
  src/message_parser.cpp
)
ament_target_dependencies(message_json_conversion
  rclcpp
  rosidl_typesupport_introspection_cpp
)
target_include_directories(message_json_conversion PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_export_include_directories(include)
ament_export_libraries(message_json_conversion)
ament_export_targets(message_json_conversion HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rosidl_typesupport_introspection_cpp)
ament_export_include_directories(include)

add_executable(example src/example.cpp)
target_link_libraries(example
  message_json_conversion
)
install(TARGETS example DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(
  TARGETS message_json_conversion
  EXPORT message_json_conversion
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
