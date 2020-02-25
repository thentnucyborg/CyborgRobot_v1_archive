# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "trollnode: 1 messages, 0 services")

set(MSG_I_FLAGS "-Itrollnode:/home/viki/Desktop/speech_ws/src/trollnode-kinect_img/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(trollnode_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(trollnode
  "/home/viki/Desktop/speech_ws/src/trollnode-kinect_img/msg/Expression.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trollnode
)

### Generating Services

### Generating Module File
_generate_module_cpp(trollnode
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trollnode
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(trollnode_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(trollnode_generate_messages trollnode_generate_messages_cpp)

# target for backward compatibility
add_custom_target(trollnode_gencpp)
add_dependencies(trollnode_gencpp trollnode_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trollnode_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(trollnode
  "/home/viki/Desktop/speech_ws/src/trollnode-kinect_img/msg/Expression.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trollnode
)

### Generating Services

### Generating Module File
_generate_module_lisp(trollnode
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trollnode
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(trollnode_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(trollnode_generate_messages trollnode_generate_messages_lisp)

# target for backward compatibility
add_custom_target(trollnode_genlisp)
add_dependencies(trollnode_genlisp trollnode_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trollnode_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(trollnode
  "/home/viki/Desktop/speech_ws/src/trollnode-kinect_img/msg/Expression.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trollnode
)

### Generating Services

### Generating Module File
_generate_module_py(trollnode
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trollnode
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(trollnode_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(trollnode_generate_messages trollnode_generate_messages_py)

# target for backward compatibility
add_custom_target(trollnode_genpy)
add_dependencies(trollnode_genpy trollnode_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trollnode_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trollnode)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trollnode
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(trollnode_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(trollnode_generate_messages_cpp sensor_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trollnode)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trollnode
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(trollnode_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(trollnode_generate_messages_lisp sensor_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trollnode)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trollnode\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trollnode
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(trollnode_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(trollnode_generate_messages_py sensor_msgs_generate_messages_py)
