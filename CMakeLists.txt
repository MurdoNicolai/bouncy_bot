cmake_minimum_required(VERSION 3.8)
project(bouncy_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(<dependency> REQUIRED)

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

# Install package.xml
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Install data directories (modify paths based on your file structure)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf FILES_MATCHING PATTERN "*")
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz FILES_MATCHING PATTERN "*.rviz")
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes FILES_MATCHING PATTERN "*.stl")
install(DIRECTORY launches/ DESTINATION share/${PROJECT_NAME}/launch FILES_MATCHING PATTERN "*.launch.*")


# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Entity.msg"
#   DEPENDENCIES std_msgs
# )
ament_package()
