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
  CATKIN_DEPENDS turtlebot3_gazebo turtlebot3_teleop
  DEPENDS GAZEBO 
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

set(tool_sources
  src/tool/tool.h
  src/tool/tool.cpp
)

##########################
# CPP MCL
##########################

set(mcl_sources
  src/mcl/mcl.h
  src/mcl/mcl.cpp
)

add_executable(mcl_cpp src/mcl/mcl_node.cpp ${mcl_sources} ${tool_sources})
add_dependencies(mcl_cpp ${mcl_cpp_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mcl_cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

##########################
# CPP GPF
##########################

set(gpf_sources
  src/gpf/gpf.h
  src/gpf/gpf.cpp
)

add_executable(gpf_cpp src/gpf/gpf_node.cpp ${gpf_sources} ${tool_sources})
add_dependencies(gpf_cpp ${gpf_cpp_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gpf_cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

##########################
# PYTHON MCL
##########################

catkin_install_python(PROGRAMS scripts/mcl_python.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
