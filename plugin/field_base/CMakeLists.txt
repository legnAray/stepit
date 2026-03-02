cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_field_base)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/field.cpp
      src/operator.cpp
    INCLUDES
      include
)
