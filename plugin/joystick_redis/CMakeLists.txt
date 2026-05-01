cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_joystick_redis)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/joystick_redis.cpp
    DEPENDS
      joystick_base
      redis_base
)
