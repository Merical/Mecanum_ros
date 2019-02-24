# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(FATAL_ERROR "Could not find messages which '/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/srv/GetParamList.srv' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?
Cannot locate message [parameters]: unknown package [handsfree_msgs] on search path [{'sc_msgs': ['/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}]")
message(FATAL_ERROR "Could not find messages which '/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/srv/SetParamList.srv' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?
Cannot locate message [parameters]: unknown package [handsfree_msgs] on search path [{'sc_msgs': ['/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}]")
message(STATUS "sc_msgs: 2 messages, 2 services")

set(MSG_I_FLAGS "-Isc_msgs:/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sc_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/parameters.msg" NAME_WE)
add_custom_target(_sc_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sc_msgs" "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/parameters.msg" ""
)

get_filename_component(_filename "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/robot_state.msg" NAME_WE)
add_custom_target(_sc_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sc_msgs" "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/robot_state.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sc_msgs
  "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/parameters.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sc_msgs
)
_generate_msg_cpp(sc_msgs
  "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/robot_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sc_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(sc_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sc_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sc_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sc_msgs_generate_messages sc_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/parameters.msg" NAME_WE)
add_dependencies(sc_msgs_generate_messages_cpp _sc_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/robot_state.msg" NAME_WE)
add_dependencies(sc_msgs_generate_messages_cpp _sc_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sc_msgs_gencpp)
add_dependencies(sc_msgs_gencpp sc_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sc_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(sc_msgs
  "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/parameters.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sc_msgs
)
_generate_msg_eus(sc_msgs
  "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/robot_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sc_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(sc_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sc_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(sc_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(sc_msgs_generate_messages sc_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/parameters.msg" NAME_WE)
add_dependencies(sc_msgs_generate_messages_eus _sc_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/robot_state.msg" NAME_WE)
add_dependencies(sc_msgs_generate_messages_eus _sc_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sc_msgs_geneus)
add_dependencies(sc_msgs_geneus sc_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sc_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sc_msgs
  "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/parameters.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sc_msgs
)
_generate_msg_lisp(sc_msgs
  "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/robot_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sc_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(sc_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sc_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sc_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sc_msgs_generate_messages sc_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/parameters.msg" NAME_WE)
add_dependencies(sc_msgs_generate_messages_lisp _sc_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/robot_state.msg" NAME_WE)
add_dependencies(sc_msgs_generate_messages_lisp _sc_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sc_msgs_genlisp)
add_dependencies(sc_msgs_genlisp sc_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sc_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(sc_msgs
  "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/parameters.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sc_msgs
)
_generate_msg_nodejs(sc_msgs
  "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/robot_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sc_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(sc_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sc_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(sc_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(sc_msgs_generate_messages sc_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/parameters.msg" NAME_WE)
add_dependencies(sc_msgs_generate_messages_nodejs _sc_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/robot_state.msg" NAME_WE)
add_dependencies(sc_msgs_generate_messages_nodejs _sc_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sc_msgs_gennodejs)
add_dependencies(sc_msgs_gennodejs sc_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sc_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sc_msgs
  "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/parameters.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sc_msgs
)
_generate_msg_py(sc_msgs
  "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/robot_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sc_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(sc_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sc_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sc_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sc_msgs_generate_messages sc_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/parameters.msg" NAME_WE)
add_dependencies(sc_msgs_generate_messages_py _sc_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lishenghao/ros_workspace/SC0_ws/src/sc_msgs/msg/robot_state.msg" NAME_WE)
add_dependencies(sc_msgs_generate_messages_py _sc_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sc_msgs_genpy)
add_dependencies(sc_msgs_genpy sc_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sc_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sc_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sc_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(sc_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(sc_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sc_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sc_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(sc_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(sc_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sc_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sc_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(sc_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(sc_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sc_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sc_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(sc_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(sc_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sc_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sc_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sc_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(sc_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(sc_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
