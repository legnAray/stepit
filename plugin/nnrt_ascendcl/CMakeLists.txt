cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_nnrt_ascendcl)

get_library_directory(ascendcl ASCENDCL_DIR REQUIRED)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/ascendcl.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/nnrt/ascendcl.h
)
target_include_directories(${PROJECT_NAME} PUBLIC ${ASCENDCL_DIR}/../include)
target_link_directories(${PROJECT_NAME} BEFORE PUBLIC ${ASCENDCL_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_nnrt_base
    ascendcl
)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${ASCENDCL_DIR}")
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
