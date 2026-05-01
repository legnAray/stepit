cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_pyutils)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
init_submodule(extern/pybind11 WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
add_subdirectory(${STEPIT_HOME_DIRECTORY}/extern/pybind11 pybind11)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/ndarray_utils.cpp
      src/npz_reader.cpp
      src/pkl_reader.cpp
      src/python_runtime.cpp
    ENTRY
      src/entry.cpp
    INCLUDES
      include
    FLAGS
      -Wall -Wextra -Wpedantic -Wno-unused-parameter
    LINK_LIBS
      pybind11::embed
    DEPENDS
      field_base
)
