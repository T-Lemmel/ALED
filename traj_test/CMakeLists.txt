cmake_minimum_required(VERSION 3.8)
project(traj_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(traj_test_node src/test_trajectory_node.cpp)
ament_target_dependencies(traj_test_node rclcpp geometry_msgs nav_msgs tf2_ros)  # Updated dependencies
target_include_directories(traj_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${HEADERS}>
  ${OpenCV_INCLUDE_DIRS}  # Include OpenCV directories
)
target_compile_features(traj_test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS traj_test_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
