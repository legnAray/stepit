cmake_minimum_required(VERSION 3.16)

set(VALID_ROBOTS "aliengo;go1;b1")
if (NOT STEPIT_UNITREE_ROBOT IN_LIST VALID_ROBOTS)
  message(FATAL_ERROR "Unsupported robot '${STEPIT_UNITREE_ROBOT}'")
endif ()

project(stepit_plugin_robot_unitree_${STEPIT_UNITREE_ROBOT})

get_library_directory(lcm LCM_DIR REQUIRED)

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ARCH amd64)
elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(ARCH arm64)
else ()
  message(FATAL_ERROR "Unsupported architecture '${CMAKE_SYSTEM_PROCESSOR}' for ${PROJECT_NAME}.")
endif ()

init_submodule(extern/robot_sdk/unitree_${STEPIT_UNITREE_ROBOT}_sdk WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
set(UNITREE_LEGGED_SDK_HOME ${STEPIT_HOME_DIRECTORY}/extern/robot_sdk/unitree_${STEPIT_UNITREE_ROBOT}_sdk)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/unitree.cpp
    INCLUDES ${CMAKE_CURRENT_LIST_DIR}/include ${UNITREE_LEGGED_SDK_HOME}/include
    LINK_DIRS ${UNITREE_LEGGED_SDK_HOME}/lib/cpp/${ARCH}
    LINK_LIBS unitree_legged_sdk lcm
)

string(TOUPPER "UNITREE_${STEPIT_UNITREE_ROBOT}" ROBOT_MACRO)
target_compile_definitions(${PROJECT_NAME} PUBLIC BOOST_BIND_GLOBAL_PLACEHOLDERS STEPIT_UNITREE_ROBOT=${ROBOT_MACRO})
