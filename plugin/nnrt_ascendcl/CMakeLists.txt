cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_nnrt_ascendcl)

get_library_directory(ascendcl ASCENDCL_DIR REQUIRED)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/ascendcl.cpp
    INCLUDES include ${ASCENDCL_DIR}/../include
    LINK_DIRS ${ASCENDCL_DIR}
    LINK_LIBS ascendcl
    DEPENDS nnrt_base
)
