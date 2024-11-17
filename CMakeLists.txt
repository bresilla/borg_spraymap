cmake_minimum_required(VERSION 3.8)
project(borg_spraymap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(clouder src/clouder.cpp)
add_executable(joiner src/joiner.cpp)
add_executable(saver src/saver.cpp)

ament_target_dependencies(clouder
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  message_filters
  pcl_ros
  pcl_conversions
)

ament_target_dependencies(joiner
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  message_filters
  pcl_ros
  pcl_conversions
)

ament_target_dependencies(saver
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  message_filters
  pcl_ros
  pcl_conversions
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${Eigen_LIBRARY_DIRS})
add_definitions(${Eigen_DEFINITIONS})
find_package(fmt REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(clouder
  fmt::fmt
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
)
target_link_libraries(joiner
  fmt::fmt
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
)

target_link_libraries(saver
  # fmt::fmt
  # ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${Eigen_LIBRARIES}
)

install(TARGETS
  clouder joiner saver
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
