# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "spyndra: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ispyndra:/home/pi/Desktop/Spyndra/spyndra/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(spyndra_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pi/Desktop/Spyndra/spyndra/msg/BeaconPos.msg" NAME_WE)
add_custom_target(_spyndra_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spyndra" "/home/pi/Desktop/Spyndra/spyndra/msg/BeaconPos.msg" ""
)

get_filename_component(_filename "/home/pi/Desktop/Spyndra/spyndra/msg/MotorSignal.msg" NAME_WE)
add_custom_target(_spyndra_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spyndra" "/home/pi/Desktop/Spyndra/spyndra/msg/MotorSignal.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(spyndra
  "/home/pi/Desktop/Spyndra/spyndra/msg/BeaconPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spyndra
)
_generate_msg_cpp(spyndra
  "/home/pi/Desktop/Spyndra/spyndra/msg/MotorSignal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spyndra
)

### Generating Services

### Generating Module File
_generate_module_cpp(spyndra
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spyndra
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(spyndra_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(spyndra_generate_messages spyndra_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/Desktop/Spyndra/spyndra/msg/BeaconPos.msg" NAME_WE)
add_dependencies(spyndra_generate_messages_cpp _spyndra_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/Desktop/Spyndra/spyndra/msg/MotorSignal.msg" NAME_WE)
add_dependencies(spyndra_generate_messages_cpp _spyndra_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spyndra_gencpp)
add_dependencies(spyndra_gencpp spyndra_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spyndra_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(spyndra
  "/home/pi/Desktop/Spyndra/spyndra/msg/BeaconPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spyndra
)
_generate_msg_lisp(spyndra
  "/home/pi/Desktop/Spyndra/spyndra/msg/MotorSignal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spyndra
)

### Generating Services

### Generating Module File
_generate_module_lisp(spyndra
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spyndra
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(spyndra_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(spyndra_generate_messages spyndra_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/Desktop/Spyndra/spyndra/msg/BeaconPos.msg" NAME_WE)
add_dependencies(spyndra_generate_messages_lisp _spyndra_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/Desktop/Spyndra/spyndra/msg/MotorSignal.msg" NAME_WE)
add_dependencies(spyndra_generate_messages_lisp _spyndra_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spyndra_genlisp)
add_dependencies(spyndra_genlisp spyndra_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spyndra_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(spyndra
  "/home/pi/Desktop/Spyndra/spyndra/msg/BeaconPos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spyndra
)
_generate_msg_py(spyndra
  "/home/pi/Desktop/Spyndra/spyndra/msg/MotorSignal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spyndra
)

### Generating Services

### Generating Module File
_generate_module_py(spyndra
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spyndra
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(spyndra_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(spyndra_generate_messages spyndra_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/Desktop/Spyndra/spyndra/msg/BeaconPos.msg" NAME_WE)
add_dependencies(spyndra_generate_messages_py _spyndra_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/Desktop/Spyndra/spyndra/msg/MotorSignal.msg" NAME_WE)
add_dependencies(spyndra_generate_messages_py _spyndra_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spyndra_genpy)
add_dependencies(spyndra_genpy spyndra_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spyndra_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spyndra)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spyndra
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(spyndra_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spyndra)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spyndra
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(spyndra_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spyndra)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spyndra\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spyndra
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spyndra
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spyndra/.+/__init__.pyc?$"
  )
endif()
add_dependencies(spyndra_generate_messages_py std_msgs_generate_messages_py)
