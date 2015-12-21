cmake_minimum_required(VERSION 2.8.3)

project (testing)

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  interactive_markers
  tf
  roslib
  cmake_modules
)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Boost REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs interactive_markers tf roslib
)


include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_executable(testing src/testing.cpp src/MapManager.cpp)

target_link_libraries(testing ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})
