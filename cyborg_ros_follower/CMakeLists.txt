cmake_minimum_required(VERSION 2.8.3)
project(ntnu_cyborg_follower)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}")

find_package(catkin REQUIRED COMPONENTS
    rosaria
    roscpp
    roscpp_serialization
    rospy
    std_msgs
    genmsg
    geometry_msgs
    k2_client
    tf
)

find_package(Jsoncpp REQUIRED)

catkin_package(
  LIBRARIES decision_maker
  CATKIN_DEPENDS k2_client roscpp rospy std_msgs tf
  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIR}
)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

add_executable(ntnu_cyborg_follower_node src/node.cpp)
add_dependencies(ntnu_cyborg_follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ntnu_cyborg_follower_node ${catkin_LIBRARIES})
