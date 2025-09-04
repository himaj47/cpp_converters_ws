cmake_minimum_required(VERSION 3.8)
project(package_two)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(package_one REQUIRED)
find_package(cpp_converter REQUIRED)
find_package(rosidl_default_generators REQUIRED)  

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

include_directories(
  include
)

add_executable(talker src/simplePublisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs package_one)

convert_to_ros_msg(msg_generator include/package_two/pkg_two_msg.hpp DEPENDENCIES package_one)
add_dependencies(talker msg_generator)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(
  include
  include/${PROJECT_NAME}
)

ament_package()
