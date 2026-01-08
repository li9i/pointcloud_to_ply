cmake_minimum_required(VERSION 3.8)
project(pointcloud_to_ply)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Include PCL directories
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(pointcloud_to_ply_node src/pointcloud_to_ply_node.cpp)

# Link libraries
target_include_directories(pointcloud_to_ply_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(pointcloud_to_ply_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(pointcloud_to_ply_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
)

# Install executable
install(TARGETS
  pointcloud_to_ply_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
