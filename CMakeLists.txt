cmake_minimum_required(VERSION 3.0.2)
project(odom-lidar-pf)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    nav_msgs
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
    tf
    cv_bridge
)

find_package(gazebo REQUIRED turtlebot3_gazebo turtlebot3_teleop)
find_package(OpenCV)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS turtlebot3_gazebo turtlebot3_teleop
  DEPENDS GAZEBO 
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

set(mcl_sources
  src/mcl/mcl.h
  src/mcl/mcl.cpp
)

set(tool_sources
  src/tool/tool.h
  src/tool/tool.cpp
)

# set(vodom_sources
#   src/vodom/vo_features.h
# )

# add_executable(vodom src/vodom/mono_vo.cpp ${vodom_sources})
# add_dependencies(vodom ${vodom_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(vodom ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mcl src/mcl/mcl_node.cpp ${mcl_sources} ${tool_sources})
add_dependencies(mcl ${mcl_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
