cmake_minimum_required(VERSION 3.22)
project(ismpc_interfaces VERSION 0.1)

# ============================= DEPENDENCIES ===============================================
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# =============================================================================================


# ============================= MESSAGE GENERATION =========================================
set(msg_files
  "msg/SimData.msg"
  "msg/ControlCommand.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)
# =============================================================================================

# ============================= PACKAGE =================================================
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
# =============================================================================================
