cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_nnrt_rknnrt)

get_library_directory(rknnrt RKNNRT_DIR REQUIRED VERBOSE)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/rknnrt.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/nnrt/rknnrt.h
)
target_include_directories(${PROJECT_NAME} PUBLIC ${RKNNRT_DIR}/../include)
target_link_directories(${PROJECT_NAME} BEFORE PUBLIC ${RKNNRT_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_nnrt_base
    rknnrt
)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${RKNNRT_DIR}")
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
