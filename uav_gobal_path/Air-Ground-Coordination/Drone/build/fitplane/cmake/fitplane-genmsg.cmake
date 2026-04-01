# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fitplane: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ifitplane:/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fitplane_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg" NAME_WE)
add_custom_target(_fitplane_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fitplane" "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg" ""
)

get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/PlaneMap.msg" NAME_WE)
add_custom_target(_fitplane_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fitplane" "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/PlaneMap.msg" "fitplane/Plane"
)

get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/GridPlaneInfo.msg" NAME_WE)
add_custom_target(_fitplane_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fitplane" "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/GridPlaneInfo.msg" "geometry_msgs/Point:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fitplane
)
_generate_msg_cpp(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/PlaneMap.msg"
  "${MSG_I_FLAGS}"
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fitplane
)
_generate_msg_cpp(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/GridPlaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fitplane
)

### Generating Services

### Generating Module File
_generate_module_cpp(fitplane
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fitplane
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fitplane_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fitplane_generate_messages fitplane_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_cpp _fitplane_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/PlaneMap.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_cpp _fitplane_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/GridPlaneInfo.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_cpp _fitplane_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fitplane_gencpp)
add_dependencies(fitplane_gencpp fitplane_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fitplane_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fitplane
)
_generate_msg_eus(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/PlaneMap.msg"
  "${MSG_I_FLAGS}"
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fitplane
)
_generate_msg_eus(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/GridPlaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fitplane
)

### Generating Services

### Generating Module File
_generate_module_eus(fitplane
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fitplane
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fitplane_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fitplane_generate_messages fitplane_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_eus _fitplane_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/PlaneMap.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_eus _fitplane_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/GridPlaneInfo.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_eus _fitplane_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fitplane_geneus)
add_dependencies(fitplane_geneus fitplane_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fitplane_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fitplane
)
_generate_msg_lisp(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/PlaneMap.msg"
  "${MSG_I_FLAGS}"
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fitplane
)
_generate_msg_lisp(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/GridPlaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fitplane
)

### Generating Services

### Generating Module File
_generate_module_lisp(fitplane
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fitplane
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fitplane_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fitplane_generate_messages fitplane_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_lisp _fitplane_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/PlaneMap.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_lisp _fitplane_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/GridPlaneInfo.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_lisp _fitplane_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fitplane_genlisp)
add_dependencies(fitplane_genlisp fitplane_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fitplane_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fitplane
)
_generate_msg_nodejs(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/PlaneMap.msg"
  "${MSG_I_FLAGS}"
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fitplane
)
_generate_msg_nodejs(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/GridPlaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fitplane
)

### Generating Services

### Generating Module File
_generate_module_nodejs(fitplane
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fitplane
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fitplane_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fitplane_generate_messages fitplane_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_nodejs _fitplane_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/PlaneMap.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_nodejs _fitplane_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/GridPlaneInfo.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_nodejs _fitplane_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fitplane_gennodejs)
add_dependencies(fitplane_gennodejs fitplane_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fitplane_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane
)
_generate_msg_py(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/PlaneMap.msg"
  "${MSG_I_FLAGS}"
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane
)
_generate_msg_py(fitplane
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/GridPlaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane
)

### Generating Services

### Generating Module File
_generate_module_py(fitplane
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fitplane_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fitplane_generate_messages fitplane_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/Plane.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_py _fitplane_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/PlaneMap.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_py _fitplane_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane/msg/GridPlaneInfo.msg" NAME_WE)
add_dependencies(fitplane_generate_messages_py _fitplane_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fitplane_genpy)
add_dependencies(fitplane_genpy fitplane_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fitplane_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fitplane)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fitplane
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(fitplane_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(fitplane_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fitplane)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fitplane
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(fitplane_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(fitplane_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fitplane)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fitplane
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(fitplane_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(fitplane_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fitplane)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fitplane
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(fitplane_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(fitplane_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(fitplane_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(fitplane_generate_messages_py geometry_msgs_generate_messages_py)
endif()
