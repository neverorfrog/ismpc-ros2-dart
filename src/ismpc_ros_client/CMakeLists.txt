cmake_minimum_required(VERSION 3.22)
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)

# ============================= PROJECT SETUP ===============================================
project(ismpc_ros_client VERSION 0.1 LANGUAGES CXX)
set(PROJECT_LIB ismpc_ros_client)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(cxx_setup INTERFACE)
target_compile_options(cxx_setup INTERFACE -Wall -Wpedantic -Wextra)
target_compile_features(cxx_setup INTERFACE cxx_std_20)

if(CMAKE_CXX_COMPILER_LOADED)
    message(STATUS "Path: ${CMAKE_CXX_COMPILER}")
    message(STATUS "Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
    message(STATUS "Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/executables")

# =============================================================================================

# ============================== DEPENDENCIES ===============================================
set(PACKAGE_DEPS
    rclcpp
    ament_index_cpp
    pinocchio
    ismpc_cpp
    ismpc_interfaces
    ismpc_ros_utils
    tf2_ros
    tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)

foreach(PACKAGE ${PACKAGE_DEPS})
    find_package(${PACKAGE} REQUIRED)
    if(NOT ${PACKAGE}_FOUND)
        message(FATAL_ERROR "Package ${PACKAGE} not found!")
    else()
        message(STATUS "Found package: ${PACKAGE}")
    endif()
endforeach()

# ===========================================================================================


# ============================ ISMPC_ROS_CLIENT LIBRARY ========================================
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(FILTER SRCS EXCLUDE REGEX ".*main\\.cpp$")
add_library(${PROJECT_LIB} STATIC ${SRCS})
ament_target_dependencies(${PROJECT_LIB} ${PACKAGE_DEPS})
target_link_libraries(${PROJECT_LIB} pinocchio::pinocchio)
target_include_directories(${PROJECT_LIB}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ===========================================================================================

# ======================== ISMPC_ROS_CLIENT NODE EXECUTABLE =====================================
add_executable(${PROJECT_LIB}_exec src/main.cpp)
ament_target_dependencies(${PROJECT_LIB}_exec ${PACKAGE_DEPS})
target_link_libraries(${PROJECT_LIB}_exec ${PROJECT_LIB})

# ===========================================================================================

# ============================== PACKAGE INSTALL ============================================
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

install(TARGETS ${PROJECT_LIBRARY} ${PROJECT_LIB}_exec
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPS})

ament_package()

# ===========================================================================================
