# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fingers: 5 messages, 0 services")

set(MSG_I_FLAGS "-Ifingers:/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fingers_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Norm_Work.msg" NAME_WE)
add_custom_target(_fingers_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fingers" "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Norm_Work.msg" ""
)

get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Shutdown.msg" NAME_WE)
add_custom_target(_fingers_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fingers" "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Shutdown.msg" ""
)

get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Finger.msg" NAME_WE)
add_custom_target(_fingers_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fingers" "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Finger.msg" ""
)

get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Bat_Cam.msg" NAME_WE)
add_custom_target(_fingers_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fingers" "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Bat_Cam.msg" ""
)

get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Finger.msg" NAME_WE)
add_custom_target(_fingers_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fingers" "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Finger.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Norm_Work.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fingers
)
_generate_msg_cpp(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Shutdown.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fingers
)
_generate_msg_cpp(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fingers
)
_generate_msg_cpp(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Bat_Cam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fingers
)
_generate_msg_cpp(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fingers
)

### Generating Services

### Generating Module File
_generate_module_cpp(fingers
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fingers
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fingers_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fingers_generate_messages fingers_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Norm_Work.msg" NAME_WE)
add_dependencies(fingers_generate_messages_cpp _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Shutdown.msg" NAME_WE)
add_dependencies(fingers_generate_messages_cpp _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Finger.msg" NAME_WE)
add_dependencies(fingers_generate_messages_cpp _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Bat_Cam.msg" NAME_WE)
add_dependencies(fingers_generate_messages_cpp _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Finger.msg" NAME_WE)
add_dependencies(fingers_generate_messages_cpp _fingers_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fingers_gencpp)
add_dependencies(fingers_gencpp fingers_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fingers_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Norm_Work.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fingers
)
_generate_msg_eus(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Shutdown.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fingers
)
_generate_msg_eus(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fingers
)
_generate_msg_eus(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Bat_Cam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fingers
)
_generate_msg_eus(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fingers
)

### Generating Services

### Generating Module File
_generate_module_eus(fingers
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fingers
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fingers_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fingers_generate_messages fingers_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Norm_Work.msg" NAME_WE)
add_dependencies(fingers_generate_messages_eus _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Shutdown.msg" NAME_WE)
add_dependencies(fingers_generate_messages_eus _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Finger.msg" NAME_WE)
add_dependencies(fingers_generate_messages_eus _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Bat_Cam.msg" NAME_WE)
add_dependencies(fingers_generate_messages_eus _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Finger.msg" NAME_WE)
add_dependencies(fingers_generate_messages_eus _fingers_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fingers_geneus)
add_dependencies(fingers_geneus fingers_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fingers_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Norm_Work.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fingers
)
_generate_msg_lisp(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Shutdown.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fingers
)
_generate_msg_lisp(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fingers
)
_generate_msg_lisp(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Bat_Cam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fingers
)
_generate_msg_lisp(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fingers
)

### Generating Services

### Generating Module File
_generate_module_lisp(fingers
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fingers
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fingers_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fingers_generate_messages fingers_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Norm_Work.msg" NAME_WE)
add_dependencies(fingers_generate_messages_lisp _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Shutdown.msg" NAME_WE)
add_dependencies(fingers_generate_messages_lisp _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Finger.msg" NAME_WE)
add_dependencies(fingers_generate_messages_lisp _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Bat_Cam.msg" NAME_WE)
add_dependencies(fingers_generate_messages_lisp _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Finger.msg" NAME_WE)
add_dependencies(fingers_generate_messages_lisp _fingers_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fingers_genlisp)
add_dependencies(fingers_genlisp fingers_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fingers_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Norm_Work.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fingers
)
_generate_msg_nodejs(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Shutdown.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fingers
)
_generate_msg_nodejs(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fingers
)
_generate_msg_nodejs(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Bat_Cam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fingers
)
_generate_msg_nodejs(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fingers
)

### Generating Services

### Generating Module File
_generate_module_nodejs(fingers
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fingers
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fingers_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fingers_generate_messages fingers_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Norm_Work.msg" NAME_WE)
add_dependencies(fingers_generate_messages_nodejs _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Shutdown.msg" NAME_WE)
add_dependencies(fingers_generate_messages_nodejs _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Finger.msg" NAME_WE)
add_dependencies(fingers_generate_messages_nodejs _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Bat_Cam.msg" NAME_WE)
add_dependencies(fingers_generate_messages_nodejs _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Finger.msg" NAME_WE)
add_dependencies(fingers_generate_messages_nodejs _fingers_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fingers_gennodejs)
add_dependencies(fingers_gennodejs fingers_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fingers_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Norm_Work.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fingers
)
_generate_msg_py(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Shutdown.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fingers
)
_generate_msg_py(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fingers
)
_generate_msg_py(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Bat_Cam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fingers
)
_generate_msg_py(fingers
  "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fingers
)

### Generating Services

### Generating Module File
_generate_module_py(fingers
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fingers
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fingers_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fingers_generate_messages fingers_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Norm_Work.msg" NAME_WE)
add_dependencies(fingers_generate_messages_py _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Bat_Cam_Shutdown.msg" NAME_WE)
add_dependencies(fingers_generate_messages_py _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/From_Finger.msg" NAME_WE)
add_dependencies(fingers_generate_messages_py _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Bat_Cam.msg" NAME_WE)
add_dependencies(fingers_generate_messages_py _fingers_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anton20241/rtc_work/fingers_rx_tx_ws/src/fingers/msg/To_Finger.msg" NAME_WE)
add_dependencies(fingers_generate_messages_py _fingers_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fingers_genpy)
add_dependencies(fingers_genpy fingers_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fingers_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fingers)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fingers
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(fingers_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fingers)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fingers
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(fingers_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fingers)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fingers
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(fingers_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fingers)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fingers
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(fingers_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fingers)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fingers\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fingers
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(fingers_generate_messages_py std_msgs_generate_messages_py)
endif()
