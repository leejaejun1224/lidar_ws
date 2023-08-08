# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "local_pkg: 4 messages, 0 services")

set(MSG_I_FLAGS "-Ilocal_pkg:/home/vision/lidar_ws/src/local_pkg/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Ilocal_pkg:/home/vision/lidar_ws/src/local_pkg/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(local_pkg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/NavPVTWithHeader.msg" NAME_WE)
add_custom_target(_local_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "local_pkg" "/home/vision/lidar_ws/src/local_pkg/msg/NavPVTWithHeader.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Serial_Info.msg" NAME_WE)
add_custom_target(_local_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "local_pkg" "/home/vision/lidar_ws/src/local_pkg/msg/Serial_Info.msg" ""
)

get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Local.msg" NAME_WE)
add_custom_target(_local_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "local_pkg" "/home/vision/lidar_ws/src/local_pkg/msg/Local.msg" "geometry_msgs/Quaternion:std_msgs/Header"
)

get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Control_Info.msg" NAME_WE)
add_custom_target(_local_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "local_pkg" "/home/vision/lidar_ws/src/local_pkg/msg/Control_Info.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/NavPVTWithHeader.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/local_pkg
)
_generate_msg_cpp(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Control_Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/local_pkg
)
_generate_msg_cpp(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Local.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/local_pkg
)
_generate_msg_cpp(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Serial_Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/local_pkg
)

### Generating Services

### Generating Module File
_generate_module_cpp(local_pkg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/local_pkg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(local_pkg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(local_pkg_generate_messages local_pkg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/NavPVTWithHeader.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_cpp _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Serial_Info.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_cpp _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Local.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_cpp _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Control_Info.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_cpp _local_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(local_pkg_gencpp)
add_dependencies(local_pkg_gencpp local_pkg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS local_pkg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/NavPVTWithHeader.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/local_pkg
)
_generate_msg_eus(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Control_Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/local_pkg
)
_generate_msg_eus(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Local.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/local_pkg
)
_generate_msg_eus(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Serial_Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/local_pkg
)

### Generating Services

### Generating Module File
_generate_module_eus(local_pkg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/local_pkg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(local_pkg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(local_pkg_generate_messages local_pkg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/NavPVTWithHeader.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_eus _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Serial_Info.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_eus _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Local.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_eus _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Control_Info.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_eus _local_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(local_pkg_geneus)
add_dependencies(local_pkg_geneus local_pkg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS local_pkg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/NavPVTWithHeader.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/local_pkg
)
_generate_msg_lisp(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Control_Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/local_pkg
)
_generate_msg_lisp(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Local.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/local_pkg
)
_generate_msg_lisp(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Serial_Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/local_pkg
)

### Generating Services

### Generating Module File
_generate_module_lisp(local_pkg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/local_pkg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(local_pkg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(local_pkg_generate_messages local_pkg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/NavPVTWithHeader.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_lisp _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Serial_Info.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_lisp _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Local.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_lisp _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Control_Info.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_lisp _local_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(local_pkg_genlisp)
add_dependencies(local_pkg_genlisp local_pkg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS local_pkg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/NavPVTWithHeader.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/local_pkg
)
_generate_msg_nodejs(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Control_Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/local_pkg
)
_generate_msg_nodejs(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Local.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/local_pkg
)
_generate_msg_nodejs(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Serial_Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/local_pkg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(local_pkg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/local_pkg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(local_pkg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(local_pkg_generate_messages local_pkg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/NavPVTWithHeader.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_nodejs _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Serial_Info.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_nodejs _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Local.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_nodejs _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Control_Info.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_nodejs _local_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(local_pkg_gennodejs)
add_dependencies(local_pkg_gennodejs local_pkg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS local_pkg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/NavPVTWithHeader.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_pkg
)
_generate_msg_py(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Control_Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_pkg
)
_generate_msg_py(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Local.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_pkg
)
_generate_msg_py(local_pkg
  "/home/vision/lidar_ws/src/local_pkg/msg/Serial_Info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_pkg
)

### Generating Services

### Generating Module File
_generate_module_py(local_pkg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_pkg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(local_pkg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(local_pkg_generate_messages local_pkg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/NavPVTWithHeader.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_py _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Serial_Info.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_py _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Local.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_py _local_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vision/lidar_ws/src/local_pkg/msg/Control_Info.msg" NAME_WE)
add_dependencies(local_pkg_generate_messages_py _local_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(local_pkg_genpy)
add_dependencies(local_pkg_genpy local_pkg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS local_pkg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/local_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/local_pkg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(local_pkg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(local_pkg_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET local_pkg_generate_messages_cpp)
  add_dependencies(local_pkg_generate_messages_cpp local_pkg_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/local_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/local_pkg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(local_pkg_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(local_pkg_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET local_pkg_generate_messages_eus)
  add_dependencies(local_pkg_generate_messages_eus local_pkg_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/local_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/local_pkg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(local_pkg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(local_pkg_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET local_pkg_generate_messages_lisp)
  add_dependencies(local_pkg_generate_messages_lisp local_pkg_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/local_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/local_pkg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(local_pkg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(local_pkg_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET local_pkg_generate_messages_nodejs)
  add_dependencies(local_pkg_generate_messages_nodejs local_pkg_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_pkg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_pkg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/local_pkg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(local_pkg_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(local_pkg_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET local_pkg_generate_messages_py)
  add_dependencies(local_pkg_generate_messages_py local_pkg_generate_messages_py)
endif()
