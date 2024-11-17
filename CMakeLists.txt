cmake_minimum_required(VERSION 3.8)
project(borg_spraymap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(executables
    clouder
    joiner
    saver
    octree
)

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

find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)

set(dependencies_ros
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
  octomap
  octomap_ros
  octomap_msgs
)


find_package(fmt REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
link_directories(${Eigen_LIBRARY_DIRS})
add_definitions(${Eigen_DEFINITIONS})
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(dependencies_sys
  fmt::fmt
  ${Boost_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

foreach(executable ${executables})
    add_executable(${executable} src/${executable}.cpp)
    ament_target_dependencies(${executable} ${dependencies_ros})
    target_link_libraries(${executable} ${dependencies_sys})
endforeach()

install(TARGETS
  ${executables}
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

include_directories(
    include
    ${Eigen_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

ament_package()
