# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fitplane_planner: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ifitplane_planner:/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fitplane_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GlobalPath.msg" NAME_WE)
add_custom_target(_fitplane_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fitplane_planner" "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GlobalPath.msg" "geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/Quaternion:std_msgs/Header"
)

get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GridPlaneInfo.msg" NAME_WE)
add_custom_target(_fitplane_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fitplane_planner" "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GridPlaneInfo.msg" "geometry_msgs/Point:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fitplane_planner
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GlobalPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fitplane_planner
)
_generate_msg_cpp(fitplane_planner
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GridPlaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fitplane_planner
)

### Generating Services

### Generating Module File
_generate_module_cpp(fitplane_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fitplane_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fitplane_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fitplane_planner_generate_messages fitplane_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GlobalPath.msg" NAME_WE)
add_dependencies(fitplane_planner_generate_messages_cpp _fitplane_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GridPlaneInfo.msg" NAME_WE)
add_dependencies(fitplane_planner_generate_messages_cpp _fitplane_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fitplane_planner_gencpp)
add_dependencies(fitplane_planner_gencpp fitplane_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fitplane_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fitplane_planner
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GlobalPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fitplane_planner
)
_generate_msg_eus(fitplane_planner
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GridPlaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fitplane_planner
)

### Generating Services

### Generating Module File
_generate_module_eus(fitplane_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fitplane_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fitplane_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fitplane_planner_generate_messages fitplane_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GlobalPath.msg" NAME_WE)
add_dependencies(fitplane_planner_generate_messages_eus _fitplane_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GridPlaneInfo.msg" NAME_WE)
add_dependencies(fitplane_planner_generate_messages_eus _fitplane_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fitplane_planner_geneus)
add_dependencies(fitplane_planner_geneus fitplane_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fitplane_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fitplane_planner
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GlobalPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fitplane_planner
)
_generate_msg_lisp(fitplane_planner
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GridPlaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fitplane_planner
)

### Generating Services

### Generating Module File
_generate_module_lisp(fitplane_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fitplane_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fitplane_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fitplane_planner_generate_messages fitplane_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GlobalPath.msg" NAME_WE)
add_dependencies(fitplane_planner_generate_messages_lisp _fitplane_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GridPlaneInfo.msg" NAME_WE)
add_dependencies(fitplane_planner_generate_messages_lisp _fitplane_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fitplane_planner_genlisp)
add_dependencies(fitplane_planner_genlisp fitplane_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fitplane_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fitplane_planner
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GlobalPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fitplane_planner
)
_generate_msg_nodejs(fitplane_planner
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GridPlaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fitplane_planner
)

### Generating Services

### Generating Module File
_generate_module_nodejs(fitplane_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fitplane_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fitplane_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fitplane_planner_generate_messages fitplane_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GlobalPath.msg" NAME_WE)
add_dependencies(fitplane_planner_generate_messages_nodejs _fitplane_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GridPlaneInfo.msg" NAME_WE)
add_dependencies(fitplane_planner_generate_messages_nodejs _fitplane_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fitplane_planner_gennodejs)
add_dependencies(fitplane_planner_gennodejs fitplane_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fitplane_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fitplane_planner
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GlobalPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane_planner
)
_generate_msg_py(fitplane_planner
  "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GridPlaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane_planner
)

### Generating Services

### Generating Module File
_generate_module_py(fitplane_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fitplane_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fitplane_planner_generate_messages fitplane_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GlobalPath.msg" NAME_WE)
add_dependencies(fitplane_planner_generate_messages_py _fitplane_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hzl/RAL_test/uav_gobal_path/Air-Ground-Coordination/Drone/src/fitplane_planner/msg/GridPlaneInfo.msg" NAME_WE)
add_dependencies(fitplane_planner_generate_messages_py _fitplane_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fitplane_planner_genpy)
add_dependencies(fitplane_planner_genpy fitplane_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fitplane_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fitplane_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fitplane_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(fitplane_planner_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(fitplane_planner_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fitplane_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fitplane_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(fitplane_planner_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(fitplane_planner_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fitplane_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fitplane_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(fitplane_planner_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(fitplane_planner_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fitplane_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fitplane_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(fitplane_planner_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(fitplane_planner_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane_planner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fitplane_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(fitplane_planner_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(fitplane_planner_generate_messages_py geometry_msgs_generate_messages_py)
endif()
