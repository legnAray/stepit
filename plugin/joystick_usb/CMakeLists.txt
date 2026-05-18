cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_joystick_usb)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/usb.cpp
    INCLUDES
      include
    DEPENDS joystick_base
)
