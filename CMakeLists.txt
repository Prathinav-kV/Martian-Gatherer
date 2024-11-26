cmake_minimum_required(VERSION 3.8)
project(martian_gatherer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GTest REQUIRED)
find_package(gazebo REQUIRED)

# Add source files
set(SOURCE_FILES
  src/aruco_marker.cpp
  src/random_object.cpp
  src/robot_movement.cpp
)

# Add executable for the main application
add_executable(martian_gatherer_node ${SOURCE_FILES})
target_link_libraries(martian_gatherer_node rclcpp)

# Add test executable for random_test
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

  # Add test executable
  add_executable(random_test test/random_test.cpp)
  target_link_libraries(random_test
    rclcpp
    GTest::GTest
    GTest::Main
    ${GAZEBO_LIBRARIES}
    pthread
  )
  ament_add_test(NAME RandomObjectPluginTest COMMAND random_test)
endif()

ament_package()
