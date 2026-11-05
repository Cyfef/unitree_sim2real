cmake_minimum_required(VERSION 3.12)
project(g1_29dof_controller)

set(CMAKE_CXX_STANDARD 17)
add_compile_definitions(G1_REFERENCE_POLICY_PROTECTION)
option(G1_SIMULATION_JOINT_LIMITS "Loopback-only training joint limits" OFF)
if(G1_SIMULATION_JOINT_LIMITS)
  add_compile_definitions(G1_SIMULATION_JOINT_LIMITS)
endif()
if(NOT TASK_ADAPTER_DIR)
  message(FATAL_ERROR "Set TASK_ADAPTER_DIR to a task controller directory")
endif()
get_filename_component(TASK_ADAPTER_DIR "${TASK_ADAPTER_DIR}" ABSOLUTE)
if(NOT EXISTS "${TASK_ADAPTER_DIR}/main.cpp" OR NOT IS_DIRECTORY "${TASK_ADAPTER_DIR}/src")
  message(FATAL_ERROR "Task adapter is incomplete: ${TASK_ADAPTER_DIR}")
endif()
set(SDK_ROOT "${PROJECT_SOURCE_DIR}/../cpp_prefix")
list(APPEND CMAKE_PREFIX_PATH "${SDK_ROOT}/usr" "${SDK_ROOT}")
set(Boost_NO_SYSTEM_PATHS ON)
set(Boost_NO_BOOST_CMAKE ON)
set(Boost_INCLUDE_DIR "${SDK_ROOT}/usr/include")
set(Boost_LIBRARY_DIR_RELEASE "${SDK_ROOT}/usr/lib/x86_64-linux-gnu")
set(Boost_LIBRARY_DIR_DEBUG "${SDK_ROOT}/usr/lib/x86_64-linux-gnu")
set(yaml-cpp_DIR "${SDK_ROOT}/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp")

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${SDK_ROOT}/usr/include
  ${SDK_ROOT}/usr/include/eigen3
  ${SDK_ROOT}/include
  ${SDK_ROOT}/include/ddscxx
  ${SDK_ROOT}/include/iceoryx/v2.0.2
  ${TASK_ADAPTER_DIR}/include
  include
  ${PROJECT_SOURCE_DIR}/../thirdparty/onnxruntime-linux-x64-1.22.0/include
  ${PROJECT_SOURCE_DIR}/../include/
)

link_directories(${SDK_ROOT}/lib ${SDK_ROOT}/usr/lib/x86_64-linux-gnu)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread # dds
  libboost_program_options.a libyaml-cpp.a fmt
  ${PROJECT_SOURCE_DIR}/../thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0
)

# Keep dependency lookup relative to the controller build directory.
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../../thirdparty/onnxruntime-linux-x64-1.22.0/lib;$ORIGIN/../../cpp_prefix/lib;$ORIGIN/../../cpp_prefix/usr/lib/x86_64-linux-gnu")

set(ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/State_RLBase.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

file(GLOB TASK_ADAPTER_SOURCES CONFIGURE_DEPENDS "${TASK_ADAPTER_DIR}/src/*.cpp")
if(NOT TASK_ADAPTER_SOURCES)
  message(FATAL_ERROR "Task adapter has no C++ sources: ${TASK_ADAPTER_DIR}")
endif()
add_executable(g1_ctrl ${TASK_ADAPTER_DIR}/main.cpp ${TASK_ADAPTER_SOURCES})
# Diagnostic only; reads recorded zero-command traces without creating DDS.
add_executable(g1_policy_replay EXCLUDE_FROM_ALL policy_replay.cpp)
add_executable(posture_progress_test EXCLUDE_FROM_ALL tests/posture_progress_test.cpp)
target_compile_options(posture_progress_test PRIVATE -UNDEBUG)
add_executable(subscription_lifetime_test EXCLUDE_FROM_ALL tests/subscription_lifetime_test.cpp)
target_compile_options(subscription_lifetime_test PRIVATE -fsanitize=address -fno-omit-frame-pointer)
target_link_options(subscription_lifetime_test PRIVATE -fsanitize=address)
