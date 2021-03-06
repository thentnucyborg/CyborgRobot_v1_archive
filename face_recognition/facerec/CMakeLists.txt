cmake_minimum_required(VERSION 2.8.3)
project(facerec)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation database_manager)
find_package(OpenCV REQUIRED)



## Declare ROS messages and services
add_service_files(DIRECTORY srv FILES FaceRec.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)
 
## Declare a catkin package
catkin_package(
	INCLUDE_DIRS include
	#LIBRARIES
	# ${PROJECT_NAME}
	CATKIN_DEPENDS
		std_msgs
		message_runtime
	DEPENDS
		OpenCV
		
)
# Set the default path for built executables to the "bin" directory.
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
# Set the default path for built libraries to the "lib" directory.
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

## Build server and client
include_directories(SYSTEM
	"${PROJECT_SOURCE_DIR}/include"
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIR}
	${database_manager_INCLUDE_DIR}
)
link_directories(
	${catkin_LIBRARY_DIRS}
)
  
 
add_executable(facerec_server src/facerec_server.cpp)
target_link_libraries(facerec_server ${catkin_LIBRARIES} ${OpenCV_LIBS} ${database_manager_LIBS})
add_dependencies(facerec_server facerec_gencpp)

add_executable(facerec_client src/facerec_client.cpp)
target_link_libraries(facerec_client ${catkin_LIBRARIES} ${OpenCV_LIBS} ${database_manager_LIBS})
add_dependencies(facerec_client facerec_gencpp)



## Declare a C++ library
# add_library(beginner_tutorial
#   src/${PROJECT_NAME}/beginner_tutorial.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(beginner_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(beginner_tutorial_node src/beginner_tutorial_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(beginner_tutorial_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(beginner_tutorial_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS beginner_tutorial beginner_tutorial_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

