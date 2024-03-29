cmake_minimum_required(VERSION 3.8)
project(cpp_add_three_ints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shared_srvs REQUIRED)

include_directories(include)

# Declare a C++ executable for the service node
add_executable(service_node src/service.cpp)
ament_target_dependencies(service_node rclcpp shared_srvs)

# Declare a C++ executable for the client node
add_executable(client_node src/client.cpp)
ament_target_dependencies(client_node rclcpp shared_srvs)

# Install executables and/or libraries
install(TARGETS
  service_node
  client_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/)

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
