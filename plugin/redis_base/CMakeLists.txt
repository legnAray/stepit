cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_redis_base)

find_package(PkgConfig REQUIRED)
pkg_check_modules(HIREDIS REQUIRED hiredis)
find_package(nlohmann_json REQUIRED)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/redis_client.cpp
    INCLUDES
      include
      ${HIREDIS_INCLUDE_DIRS}
    LINK_LIBS
      ${HIREDIS_LIBRARIES}
      nlohmann_json::nlohmann_json
    LINK_DIRS
      ${HIREDIS_LIBRARY_DIRS}
    FLAGS
      -Wall -Wextra -Wpedantic -Wno-unused-parameter
      ${HIREDIS_CFLAGS_OTHER}
)
