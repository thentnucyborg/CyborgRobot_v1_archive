cmake_minimum_required(VERSION 2.8.3)
project(ntnu_cyborg_simon_says)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    std_msgs
    message_generation
    k2_client
    trollnode
    ntnu_cyborg_coordinator
)

find_package(Jsoncpp REQUIRED)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

catkin_package(
    CATKIN_DEPENDS
    roscpp
    geometry_msgs
    std_msgs
    message_generation
    k2_client
    trollnode
    ntnu_cyborg_coordinator
)

include_directories(
    include ${catkin_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIR}
)

add_executable(ntnu_cyborg_simon_says_node src/node.cpp src/console_output.cpp src/game.cpp src/gestures.cpp src/troll_output.cpp)
add_dependencies(ntnu_cyborg_simon_says_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ntnu_cyborg_simon_says_node ${catkin_LIBRARIES})
