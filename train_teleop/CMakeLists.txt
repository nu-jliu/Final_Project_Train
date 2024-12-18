cmake_minimum_required(VERSION 3.8)
project(train_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(train_interfaces REQUIRED)
find_package(modern_robotics REQUIRED)
find_package(arm_kinematics REQUIRED)

add_executable(arm_teleop src/arm_teleop.cpp)
target_include_directories(arm_teleop PRIVATE include)
target_link_libraries(arm_teleop
  arm_kinematics::arm_kinematics
)
ament_target_dependencies(arm_teleop 
  rclcpp
  sensor_msgs
  std_srvs
  train_interfaces
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS arm_teleop
  DESTINATION lib/${PROJECT_NAME}
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
