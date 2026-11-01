cmake_minimum_required(VERSION 3.22)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -std=c++17 -g")
project(topoprm_2d)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

include_directories(./planner/path_searching/include)
include_directories(./planner/GridMap2D/include)
include_directories(./planner/plan_manage/include/plan_manage)
include_directories(./planner/plan_manage/include)

add_subdirectory(planner/path_searching)
add_subdirectory(planner/GridMap2D)
add_subdirectory(planner/plan_manage)

# 编译节点
add_executable(motion_plan src/trajectory_publisher.cpp)
ament_target_dependencies(
  motion_plan
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
  visualization_msgs
)

target_link_libraries(
  motion_plan
  path_searching  
  local_map 
  kinodynamic_planner
)

# 安装节点
install(TARGETS
  motion_plan
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(FILES rviz.rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
