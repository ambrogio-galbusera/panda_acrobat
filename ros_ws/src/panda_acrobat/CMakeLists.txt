cmake_minimum_required(VERSION 3.8)
project(panda_acrobat)

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
find_package(OpenCV 4.5.4 REQUIRED)

add_executable(panda_cam src/panda_cam.cpp)
add_executable(panda_path src/panda_path.cpp)
add_executable(panda_arduino src/panda_arduino.cpp)
ament_target_dependencies(panda_cam rclcpp std_msgs OpenCV)
ament_target_dependencies(panda_path rclcpp std_msgs)
ament_target_dependencies(panda_arduino rclcpp std_msgs)

install(TARGETS
  panda_cam
  panda_path
  panda_arduino
  DESTINATION lib/${PROJECT_NAME})

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