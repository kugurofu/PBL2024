cmake_minimum_required(VERSION 3.8)
project(pubsub_lecture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add/hello.cpp
add_executable(hello src/hello.cpp)
ament_target_dependencies(hello rclcpp)
install(TARGETS hello DESTINATION lib/${PROJECT_NAME})

# add/publisher1.cpp,subscriber1.cpp
set(NODES publisher1;subscriber1)
foreach(target IN LISTS NODES)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} rclcpp std_msgs)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

# add/turtlesim_controller.cpp
add_executable(turtlesim_controller src/turtlesim_controller.cpp)
ament_target_dependencies(turtlesim_controller rclcpp geometry_msgs)
install(TARGETS turtlesim_controller DESTINATION lib/${PROJECT_NAME})
 
# add/gazebo_controller.cpp
add_executable(gazebo_controller src/gazebo_controller.cpp)
ament_target_dependencies(gazebo_controller rclcpp geometry_msgs)
install(TARGETS gazebo_controller DESTINATION lib/${PROJECT_NAME})

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
