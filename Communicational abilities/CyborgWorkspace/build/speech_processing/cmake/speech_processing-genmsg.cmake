# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "speech_processing: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ispeech_processing:/home/viki/Desktop/levering/CyborgWorkspace/src/speech_processing/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(speech_processing_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(speech_processing
  "/home/viki/Desktop/levering/CyborgWorkspace/src/speech_processing/msg/Expression.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_processing
)

### Generating Services

### Generating Module File
_generate_module_cpp(speech_processing
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_processing
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(speech_processing_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(speech_processing_generate_messages speech_processing_generate_messages_cpp)

# target for backward compatibility
add_custom_target(speech_processing_gencpp)
add_dependencies(speech_processing_gencpp speech_processing_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS speech_processing_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(speech_processing
  "/home/viki/Desktop/levering/CyborgWorkspace/src/speech_processing/msg/Expression.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_processing
)

### Generating Services

### Generating Module File
_generate_module_lisp(speech_processing
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_processing
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(speech_processing_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(speech_processing_generate_messages speech_processing_generate_messages_lisp)

# target for backward compatibility
add_custom_target(speech_processing_genlisp)
add_dependencies(speech_processing_genlisp speech_processing_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS speech_processing_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(speech_processing
  "/home/viki/Desktop/levering/CyborgWorkspace/src/speech_processing/msg/Expression.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_processing
)

### Generating Services

### Generating Module File
_generate_module_py(speech_processing
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_processing
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(speech_processing_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(speech_processing_generate_messages speech_processing_generate_messages_py)

# target for backward compatibility
add_custom_target(speech_processing_genpy)
add_dependencies(speech_processing_genpy speech_processing_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS speech_processing_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_processing)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_processing
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(speech_processing_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_processing)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_processing
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(speech_processing_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_processing)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_processing\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_processing
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(speech_processing_generate_messages_py std_msgs_generate_messages_py)
