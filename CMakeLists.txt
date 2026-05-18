cmake_minimum_required(VERSION 3.16)
project(stepit)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

if (NOT DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY AND
    NOT DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY AND
    NOT DEFINED CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif ()

set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
if (CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
else ()
  message(
      WARNING
      "Building in '${CMAKE_BUILD_TYPE}' mode. "
      "Add '-DCMAKE_BUILD_TYPE=Release' for better performance."
  )
endif ()
if (NOT DEFINED STEPIT_LIB_DESTINATION)
  set(STEPIT_LIB_DESTINATION "${CMAKE_INSTALL_LIBDIR}")
endif ()
if (NOT DEFINED STEPIT_BIN_DESTINATION)
  set(STEPIT_BIN_DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif ()
file(RELATIVE_PATH INSTALL_RPATH
    ${CMAKE_INSTALL_PREFIX}/${STEPIT_BIN_DESTINATION}
    ${CMAKE_INSTALL_PREFIX}/${STEPIT_LIB_DESTINATION}
)
set(CMAKE_INSTALL_RPATH "$ORIGIN/${INSTALL_RPATH}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

set(STEPIT_HOME_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}")
include(${CMAKE_CURRENT_LIST_DIR}/cmake/stepit_utils.cmake)

find_package(Boost REQUIRED COMPONENTS filesystem program_options)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
init_submodule(extern/llu WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
add_subdirectory(extern/llu)

stepit_add_library(stepit_core SHARED
    src/stepit/agent.cpp
    src/stepit/communication.cpp
    src/stepit/control_input.cpp
    src/stepit/logging.cpp
    src/stepit/plugin.cpp
    src/stepit/publisher.cpp
    src/stepit/robot.cpp
    src/stepit/spin.cpp
    src/stepit/utils.cpp
)
target_compile_definitions(stepit_core PRIVATE STEPIT_CONFIG_DIR="${STEPIT_HOME_DIRECTORY}/config")
target_compile_definitions(stepit_core PUBLIC FMT_HEADER_ONLY=1)
target_compile_options(stepit_core PUBLIC -fPIC PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
target_include_directories(stepit_core PUBLIC
    $<BUILD_INTERFACE:${STEPIT_HOME_DIRECTORY}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(stepit_core PUBLIC
    atomic
    Boost::filesystem
    Boost::program_options
    Eigen3::Eigen
    dl
    llu::llu
    Threads::Threads
    yaml-cpp
)

# Check for gettid() function
include(CheckCSourceCompiles)
check_c_source_compiles("#include <unistd.h>
int main() { (void) gettid(); return 0; }
" HAVE_GETTID)
if (NOT HAVE_GETTID)
  target_compile_definitions(stepit_core PUBLIC STEPIT_FIX_GETTID)
endif ()

stepit_add_executable(stepit src/stepit_app/main.cpp)
stepit_add_executable(rotation_transform src/stepit_app/rotation_transform.cpp)
stepit_add_plugin(stepit_plugin_builtin SOURCES src/stepit/builtin.cpp)

# Dynamically discover plugins from manifest files
set(STEPIT_DISCOVERED_PLUGINS "")
# Add the default plugin search directory
list(APPEND STEPIT_PLUGIN_DIRS "${CMAKE_CURRENT_LIST_DIR}/plugin")
foreach (plugin_search_dir ${STEPIT_PLUGIN_DIRS})
  if (NOT IS_ABSOLUTE ${plugin_search_dir})
    set(plugin_search_dir "$ENV{PWD}/${plugin_search_dir}")
  endif ()
  if (NOT IS_DIRECTORY ${plugin_search_dir})
    message(WARNING "Plugin directory '${plugin_search_dir}' does not exist.")
    continue()
  endif ()

  # Search for plugins in the directory
  file(GLOB discovered_plugin_dirs RELATIVE ${plugin_search_dir} ${plugin_search_dir}/*)
  foreach (plugin_entry ${discovered_plugin_dirs})
    set(plugin_dir "${plugin_search_dir}/${plugin_entry}")
    get_filename_component(plugin_dir "${plugin_dir}" REALPATH)
    if (
        IS_DIRECTORY ${plugin_dir}
        AND EXISTS ${plugin_dir}/stepit_plugin_manifest.cmake
        AND NOT ${plugin_dir} STREQUAL ${CMAKE_CURRENT_LIST_DIR}
    )
      stepit_load_plugin_manifest(${plugin_dir} plugin)
      if (plugin IN_LIST STEPIT_DISCOVERED_PLUGINS)
        stepit_get_plugin_property(${plugin} DIR existing_plugin_dir)
        message(
            FATAL_ERROR
            "Duplicated plugin '${plugin}' declared in "
            "'${existing_plugin_dir}' and '${plugin_dir}'."
        )
      endif ()
      list(APPEND STEPIT_DISCOVERED_PLUGINS ${plugin})
    endif ()
  endforeach ()
endforeach ()
list(SORT STEPIT_DISCOVERED_PLUGINS)
message(STATUS "Discovered stepit plugins: ${STEPIT_DISCOVERED_PLUGINS}")

if (NOT DEFINED STEPIT_BLACKLIST_PLUGINS)
  set(STEPIT_BLACKLIST_PLUGINS "")
endif ()
set(STEPIT_ENABLED_PLUGINS "${STEPIT_DISCOVERED_PLUGINS}")
foreach (blacklisted_plugin ${STEPIT_BLACKLIST_PLUGINS})
  list(REMOVE_ITEM STEPIT_ENABLED_PLUGINS ${blacklisted_plugin})
endforeach ()
message(STATUS "Blacklisted plugins: ${STEPIT_BLACKLIST_PLUGINS}")

if (NOT DEFINED STEPIT_WHITELIST_PLUGINS)
  set(STEPIT_WHITELIST_PLUGINS "${STEPIT_ENABLED_PLUGINS}")
endif ()
message(STATUS "Requested stepit plugins: ${STEPIT_WHITELIST_PLUGINS}")

# Resolve the requested plugin graph before building any plugin targets.
set_property(GLOBAL PROPERTY STEPIT_RESOLVED_PLUGINS "")
set_property(GLOBAL PROPERTY STEPIT_BUILDABLE_PLUGIN_ORDER "")
set_property(GLOBAL PROPERTY STEPIT_BUILT_PLUGINS "")

foreach (plugin ${STEPIT_WHITELIST_PLUGINS})
  if (NOT ${plugin} IN_LIST STEPIT_DISCOVERED_PLUGINS)
    message(FATAL_ERROR "Plugin '${plugin}' is not supported.")
  endif ()
  stepit_resolve_plugin(${plugin} "")
endforeach ()

get_property(STEPIT_RESOLVED_PLUGINS GLOBAL PROPERTY STEPIT_RESOLVED_PLUGINS)
get_property(STEPIT_BUILDABLE_PLUGIN_ORDER GLOBAL PROPERTY STEPIT_BUILDABLE_PLUGIN_ORDER)
message(STATUS "Resolved stepit plugin closure: ${STEPIT_RESOLVED_PLUGINS}")
message(STATUS "Buildable stepit plugins: ${STEPIT_BUILDABLE_PLUGIN_ORDER}")

# Build the buildable plugins in dependency order.
foreach (plugin ${STEPIT_BUILDABLE_PLUGIN_ORDER})
  stepit_get_plugin_property(${plugin} DEPENDS plugin_dependencies)
  set(skip_plugin FALSE)
  foreach (dependency ${plugin_dependencies})
    stepit_get_plugin_property(${dependency} STATUS dependency_status)
    if (NOT dependency_status STREQUAL "built")
      stepit_mark_plugin_skipped_due_to_dependency(${plugin} ${dependency})
      set(skip_plugin TRUE)
      break()
    endif ()
  endforeach ()
  if (skip_plugin)
    continue()
  endif ()

  stepit_get_plugin_property(${plugin} DIR plugin_dir)
  add_subdirectory(${plugin_dir} plugins/${plugin})

  if (TARGET stepit_plugin_${plugin})
    stepit_set_plugin_property(${plugin} STATUS "built")
    set_property(GLOBAL APPEND PROPERTY STEPIT_BUILT_PLUGINS ${plugin})
  else ()
    stepit_set_plugin_property(${plugin} STATUS "skipped")
    stepit_set_plugin_property(
        ${plugin}
        REASON
        "Plugin CMakeLists.txt did not create target 'stepit_plugin_${plugin}'. Check manifest requirements."
    )
    stepit_set_plugin_property(${plugin} CHAIN ${plugin})
  endif ()
endforeach ()

stepit_print_plugin_report(${STEPIT_RESOLVED_PLUGINS})

get_property(STEPIT_LIBRARIES GLOBAL PROPERTY STEPIT_LIBRARIES)
get_property(STEPIT_EXECUTABLES GLOBAL PROPERTY STEPIT_EXECUTABLES)
install(TARGETS ${STEPIT_LIBRARIES}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(TARGETS ${STEPIT_EXECUTABLES}
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
)
install(
    DIRECTORY ${STEPIT_HOME_DIRECTORY}/include/stepit
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

get_property(STEPIT_BUILT_PLUGINS GLOBAL PROPERTY STEPIT_BUILT_PLUGINS)
if (NOT STEPIT_BUILT_PLUGINS)
  set(STEPIT_BUILT_PLUGINS "")
endif ()

set(stepit_dependencies_file "${CMAKE_CURRENT_BINARY_DIR}/stepitDependencies.cmake")
file(WRITE "${stepit_dependencies_file}" "set(STEPIT_BUILT_PLUGINS \"${STEPIT_BUILT_PLUGINS}\")\n")
foreach (plugin ${STEPIT_BUILT_PLUGINS})
  stepit_get_plugin_property(${plugin} DEPENDS plugin_dependencies)
  file(APPEND "${stepit_dependencies_file}" "set(STEPIT_PLUGIN_DEPENDENCIES_${plugin} \"${plugin_dependencies}\")\n")

  stepit_get_plugin_property(${plugin} DIR plugin_dir)
  if (IS_DIRECTORY "${plugin_dir}/include")
    install(
        DIRECTORY "${plugin_dir}/include/"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
  endif ()
endforeach ()

install(EXPORT stepitTargets
    FILE stepitTargets.cmake
    NAMESPACE stepit::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/stepit
)
configure_package_config_file(
    "${STEPIT_HOME_DIRECTORY}/cmake/stepitConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/stepitConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/stepit"
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/stepitConfigVersion.cmake"
    VERSION 0.0.0
    COMPATIBILITY SameMajorVersion
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/stepitConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/stepitConfigVersion.cmake"
    "${stepit_dependencies_file}"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/stepit"
)
