cmake_minimum_required(VERSION 3.0)
project(lego_loam_bor)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -g ")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  cloud_msgs
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp
    tf
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    cloud_msgs
    pcl_ros
    pcl_conversions
  DEPENDS
    EIGEN3
    PCL
    GTSAM
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_executable(lego_loam_bor
  src/imageProjection.cpp
  src/featureAssociation.cpp
  src/mapOptmization.cpp
  src/transformFusion.cpp
  src/main.cpp
)

add_dependencies(lego_loam_bor 
  ${catkin_EXPORTED_TARGETS} 
  cloud_msgs_gencpp
)
target_link_libraries(lego_loam_bor 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  gtsam
)