cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_joystick_usb)

set(GAMEPAD_ENABLE_SHARED OFF)
set(GAMEPAD_ENABLE_STATIC OFF)
set(GAMEPAD_ENABLE_INSTALL OFF)
set(GAMEPAD_ENABLE_TESTS OFF)
init_submodule(extern/joystick_sdk/libgamepad WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
add_subdirectory(${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/libgamepad libgamepad)

add_library(stepit_dependency_gamepad STATIC $<TARGET_OBJECTS:gamepad>)
target_include_directories(stepit_dependency_gamepad PUBLIC
    $<BUILD_INTERFACE:${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/libgamepad/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(stepit_dependency_gamepad PUBLIC Threads::Threads)
set_target_properties(stepit_dependency_gamepad PROPERTIES
    OUTPUT_NAME gamepad
    POSITION_INDEPENDENT_CODE ON
)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/usb.cpp
    INCLUDES
      include
      ${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/libgamepad/include
    LINK_LIBS stepit_dependency_gamepad
    DEPENDS joystick_base
)

install(
    DIRECTORY ${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/libgamepad/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(TARGETS stepit_dependency_gamepad
    EXPORT stepitTargets
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
