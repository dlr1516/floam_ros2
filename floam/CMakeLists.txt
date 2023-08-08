cmake_minimum_required(VERSION 3.5)
project(floam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
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


add_executable(floam_laser_processing_node src/LaserProcessingNode.cpp src/LaserProcessingClass.cpp src/lidar.cpp)
ament_target_dependencies(floam_laser_processing_node rclcpp sensor_msgs geometry_msgs nav_msgs PCL Ceres tf2
  tf2_ros pcl_conversions)

add_executable(floam_odom_estimation_node src/OdomEstimationNode.cpp src/lidarOptimization.cpp src/lidar.cpp src/OdomEstimationClass.cpp)
ament_target_dependencies(floam_odom_estimation_node rclcpp sensor_msgs geometry_msgs nav_msgs PCL Ceres tf2
  tf2_ros pcl_conversions)

add_executable(floam_laser_mapping_node src/LaserMappingNode.cpp src/LaserMappingClass.cpp src/lidar.cpp)
ament_target_dependencies(floam_laser_mapping_node rclcpp sensor_msgs geometry_msgs nav_msgs PCL Ceres tf2
  tf2_ros pcl_conversions)

target_link_libraries(floam_laser_processing_node ${PCL_LIBRARIES})

target_link_libraries(floam_odom_estimation_node ${PCL_LIBRARIES})
target_link_libraries(floam_odom_estimation_node ${CERES_LIBRARY_DIRS})
target_link_libraries(floam_odom_estimation_node ${CERES_LIBRARIES})

target_link_libraries(floam_laser_mapping_node ${PCL_LIBRARIES})

install(TARGETS
  floam_laser_processing_node
  floam_odom_estimation_node
  floam_laser_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
