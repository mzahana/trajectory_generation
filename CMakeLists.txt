cmake_minimum_required(VERSION 3.8)
project(trajectory_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_trajectory_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

#
# mpc.cpp
#
add_library(mpc_lib src/mpc.cpp)
target_include_directories(mpc_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(mpc_lib Eigen3)
target_link_libraries(mpc_lib OsqpEigen::OsqpEigen osqp::osqp)
# this line to exports the library
# ament_export_targets(mpc HAS_LIBRARY_TARGET)


#
# mpc_ros.cpp
#
add_library(moc_ros_lib src/mpc_ros.cpp)
target_include_directories(moc_ros_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(moc_ros_lib rclcpp nav_msgs sensor_msgs custom_trajectory_msgs trajectory_msgs)
target_link_libraries(moc_ros_lib mpc_lib)
# this line to exports the library
# ament_export_targets(mpc_ros HAS_LIBRARY_TARGET)

#
# Build the ROS 2 node (mpc_node)
#
add_executable(mpc_node
  src/mpc_node.cpp
)
target_include_directories(mpc_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(mpc_node
  mpc_lib
  moc_ros_lib
)
ament_target_dependencies(mpc_node
  rclcpp
)

# Install targets
install(
  TARGETS mpc_lib moc_ros_lib mpc_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Export package information
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp)


# install(
#   TARGETS mpc
#   EXPORT mpc
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# install(
#   TARGETS mpc_ros
#   EXPORT mpc_ros
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )


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
