cmake_minimum_required(VERSION 3.8)
project(test_msg_pkg_one)

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
# find_package(msg_pkg_one REQUIRED)
find_package(msg_pkg_two REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

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

add_executable(pub src/publisher.cpp)
add_executable(sub src/subscriber.cpp)

target_link_libraries(
  pub
  "/home/himaj/cpp_converters_ws/install/msg_pkg_two/lib/libmsg_pkg_two__rosidl_typesupport_cpp.so"
)
target_link_libraries(
  sub
  "/home/himaj/cpp_converters_ws/install/msg_pkg_two/lib/libmsg_pkg_two__rosidl_typesupport_cpp.so"
)

ament_target_dependencies(pub rclcpp msg_pkg_two)
ament_target_dependencies(sub rclcpp msg_pkg_two)

install(TARGETS pub sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
