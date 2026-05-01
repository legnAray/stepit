cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_nnrt_rknnrt)

get_library_directory(rknnrt RKNNRT_DIR REQUIRED VERBOSE)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/rknnrt.cpp
    INCLUDES include ${RKNNRT_DIR}/../include
    LINK_DIRS ${RKNNRT_DIR}
    LINK_LIBS rknnrt
    DEPENDS nnrt_base
)
