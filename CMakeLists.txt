cmake_minimum_required(VERSION 3.8)
project(pointcloud_to_ply)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io surface filters features)

# Include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
)

# Add executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
