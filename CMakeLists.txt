cmake_minimum_required(VERSION 3.8)
project(lidar_mapper_visualiser)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(NODE_NAME lidar_mapper_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

add_executable(${NODE_NAME} src/lidar_mapper_rviz.cpp)
ament_target_dependencies(${NODE_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
  message_filters
  laser_geometry
  tf2_ros
  tf2_eigen
  pcl_conversions
  pcl_ros
)

target_include_directories(${NODE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${NODE_NAME} ${PCL_LIBRARIES})

install(TARGETS ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
