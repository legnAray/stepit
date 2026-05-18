function(stepit_add_library library_name)
  add_library(${library_name} ${ARGN})
  set_property(GLOBAL APPEND PROPERTY STEPIT_LIBRARIES ${library_name})
endfunction()

function(_stepit_append_usage_include_dir output_var include_dir)
  set(include_dirs "${${output_var}}")
  if (IS_ABSOLUTE "${include_dir}")
    set(include_path "${include_dir}")
  else ()
    get_filename_component(include_path "${include_dir}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  endif ()

  string(FIND "${include_path}" "${STEPIT_HOME_DIRECTORY}" source_prefix_index)
  string(FIND "${include_path}" "${CMAKE_BINARY_DIR}" binary_prefix_index)
  if (source_prefix_index EQUAL 0 OR binary_prefix_index EQUAL 0)
    list(APPEND include_dirs "$<BUILD_INTERFACE:${include_path}>")
  else ()
    list(APPEND include_dirs "${include_path}")
  endif ()

  if (NOT IS_ABSOLUTE "${include_dir}")
    list(APPEND include_dirs "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
  endif ()

  set(${output_var} "${include_dirs}" PARENT_SCOPE)
endfunction()

function(_stepit_append_usage_link_dir output_var link_dir)
  set(link_dirs "${${output_var}}")
  if (IS_ABSOLUTE "${link_dir}")
    set(link_path "${link_dir}")
  else ()
    get_filename_component(link_path "${link_dir}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  endif ()

  string(FIND "${link_path}" "${STEPIT_HOME_DIRECTORY}" source_prefix_index)
  string(FIND "${link_path}" "${CMAKE_BINARY_DIR}" binary_prefix_index)
  if (source_prefix_index EQUAL 0 OR binary_prefix_index EQUAL 0)
    list(APPEND link_dirs "$<BUILD_INTERFACE:${link_path}>")
  else ()
    list(APPEND link_dirs "${link_path}")
  endif ()

  set(${output_var} "${link_dirs}" PARENT_SCOPE)
endfunction()

function(stepit_set_plugin_property plugin property_name)
  if (ARGC GREATER 2)
    set_property(GLOBAL PROPERTY "STEPIT_PLUGIN_${plugin}_${property_name}" "${ARGN}")
  else ()
    set_property(GLOBAL PROPERTY "STEPIT_PLUGIN_${plugin}_${property_name}" "")
  endif ()
endfunction()

function(stepit_get_plugin_property plugin property_name output_var)
  get_property(property_value GLOBAL PROPERTY "STEPIT_PLUGIN_${plugin}_${property_name}")
  if (NOT DEFINED property_value)
    set(property_value "")
  endif ()
  set(${output_var} "${property_value}" PARENT_SCOPE)
endfunction()

function(stepit_append_unique_global_property property_name)
  get_property(property_values GLOBAL PROPERTY ${property_name})
  if (NOT property_values)
    set(property_values "")
  endif ()

  foreach (property_value ${ARGN})
    if (NOT property_value IN_LIST property_values)
      list(APPEND property_values ${property_value})
    endif ()
  endforeach ()

  set_property(GLOBAL PROPERTY ${property_name} "${property_values}")
endfunction()

function(stepit_add_plugin plugin_name)
  # assert plugin_name starts with "stepit_plugin_"
  if (NOT plugin_name MATCHES "^stepit_plugin_")
    message(FATAL_ERROR "Plugin name must start with 'stepit_plugin_'")
  endif ()

  # argument structure
  set(options "")
  set(oneValueArgs "")
  set(multiValueArgs SOURCES ENTRY INCLUDES LINK_LIBS LINK_DIRS FLAGS DEPENDS)

  cmake_parse_arguments(PARSE_ARGV 1 ARG
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
  )
  if (ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unparsed arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif ()

  stepit_add_library(${plugin_name} SHARED ${ARG_SOURCES})
  set(plugin_include_dirs "")
  foreach (include_dir ${ARG_INCLUDES})
    _stepit_append_usage_include_dir(plugin_include_dirs "${include_dir}")
  endforeach ()
  target_include_directories(${plugin_name} PUBLIC ${plugin_include_dirs})
  target_link_libraries(${plugin_name} PUBLIC stepit_core ${ARG_LINK_LIBS})
  set(plugin_link_dirs "")
  foreach (link_dir ${ARG_LINK_DIRS})
    _stepit_append_usage_link_dir(plugin_link_dirs "${link_dir}")
  endforeach ()
  target_link_directories(${plugin_name} BEFORE PUBLIC ${plugin_link_dirs})
  target_compile_options(${plugin_name} PUBLIC ${ARG_FLAGS})
  set_target_properties(${plugin_name} PROPERTIES
      INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${ARG_LINK_DIRS}"
  )
  set_property(TARGET ${plugin_name} PROPERTY STEPIT_PLUGIN_DEPENDENCIES "${ARG_DEPENDS}")

  string(REGEX REPLACE "^stepit_plugin_" "" plugin_id "${plugin_name}")
  foreach (dependency ${ARG_DEPENDS})
    if (NOT TARGET stepit_plugin_${dependency})
      message(
          FATAL_ERROR
          "Plugin '${plugin_id}' depends on plugin '${dependency}', "
          "but target 'stepit_plugin_${dependency}' is unavailable. "
          "Check the plugin manifest dependency graph."
      )
    endif ()
    add_dependencies(${plugin_name} stepit_plugin_${dependency})
    target_link_libraries(${plugin_name} PUBLIC stepit_plugin_${dependency})
  endforeach ()

  if (ARG_ENTRY)
    set(plugin_entry "${plugin_name}_entry")
    stepit_add_library(${plugin_entry} MODULE ${ARG_ENTRY})
    target_link_libraries(${plugin_entry} PUBLIC ${plugin_name})
  endif ()
endfunction()

function(stepit_add_executable executable_name)
  add_executable(${executable_name} ${ARGN})
  target_link_libraries(${executable_name} PRIVATE stepit_core)
  set_property(GLOBAL APPEND PROPERTY STEPIT_EXECUTABLES ${executable_name})
endfunction()

function(get_library_directory library_name output_var)
  set(options "REQUIRED;VERBOSE")
  set(oneValueArgs "")
  set(multiValueArgs "")

  cmake_parse_arguments(PARSE_ARGV 2 ARG
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
  )
  if (ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unparsed arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif ()

  # First find the library in the LD_LIBRARY_PATH
  string(REPLACE ":" ";" LD_LIBRARY_PATH "$ENV{LD_LIBRARY_PATH}")
  find_library(${library_name}_PATH NAMES ${library_name} PATHS ${LD_LIBRARY_PATH} NO_DEFAULT_PATH)
  # If not found, try to find it in the system library paths
  if (NOT ${library_name}_PATH)
    find_library(${library_name}_PATH NAMES ${library_name})
  endif ()

  # If the library is found, set the output variable to the directory
  if (${library_name}_PATH)
    if (ARG_VERBOSE)
      message(STATUS "Found ${library_name} in ${${library_name}_PATH}")
    endif ()
    get_filename_component(library_dir ${${library_name}_PATH} DIRECTORY)
    set(${output_var} ${library_dir} PARENT_SCOPE)
  elseif (ARG_REQUIRED)
    message(FATAL_ERROR "Library ${library_name} not found.")
  endif ()
endfunction()

function(stepit_plugin_mark_unbuildable reason)
  set(STEPIT_PLUGIN_BUILDABLE FALSE PARENT_SCOPE)
  set(STEPIT_PLUGIN_REASON "${reason}" PARENT_SCOPE)
endfunction()

function(stepit_declare_plugin)
  set(options "")
  set(oneValueArgs NAME)
  set(multiValueArgs DEPENDS)

  cmake_parse_arguments(PARSE_ARGV 0 ARG
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
  )
  if (ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unparsed arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif ()
  if (NOT ARG_NAME)
    message(FATAL_ERROR "stepit_declare_plugin requires NAME.")
  endif ()

  set(STEPIT_PLUGIN_NAME "${ARG_NAME}" PARENT_SCOPE)
  set(STEPIT_PLUGIN_DEPENDS "${ARG_DEPENDS}" PARENT_SCOPE)
endfunction()

function(stepit_load_plugin_manifest plugin_dir output_var)
  unset(STEPIT_PLUGIN_NAME)
  unset(STEPIT_PLUGIN_DEPENDS)
  set(STEPIT_PLUGIN_BUILDABLE TRUE)
  set(STEPIT_PLUGIN_REASON "")
  set(STEPIT_PLUGIN_DIR "${plugin_dir}")

  # Manifests are evaluated in the top-level directory scope, so save and restore
  # directory properties to keep package find scripts from polluting unrelated targets.
  get_directory_property(saved_compile_definitions COMPILE_DEFINITIONS)
  get_directory_property(saved_compile_options COMPILE_OPTIONS)
  get_directory_property(saved_include_directories INCLUDE_DIRECTORIES)
  get_directory_property(saved_link_directories LINK_DIRECTORIES)
  get_directory_property(saved_system_include_directories SYSTEM_INCLUDE_DIRECTORIES)

  include("${plugin_dir}/stepit_plugin_manifest.cmake")

  if (NOT DEFINED STEPIT_PLUGIN_NAME OR STEPIT_PLUGIN_NAME STREQUAL "")
    message(
        FATAL_ERROR
        "Plugin manifest '${plugin_dir}/stepit_plugin_manifest.cmake' "
        "must call stepit_declare_plugin(NAME <plugin> ...)."
    )
  endif ()
  if (NOT DEFINED STEPIT_PLUGIN_DEPENDS)
    set(STEPIT_PLUGIN_DEPENDS "")
  endif ()
  if (NOT DEFINED STEPIT_PLUGIN_BUILDABLE)
    set(STEPIT_PLUGIN_BUILDABLE TRUE)
  endif ()
  if (NOT DEFINED STEPIT_PLUGIN_REASON)
    set(STEPIT_PLUGIN_REASON "")
  endif ()

  set_directory_properties(PROPERTIES
      COMPILE_DEFINITIONS "${saved_compile_definitions}"
      COMPILE_OPTIONS "${saved_compile_options}"
      INCLUDE_DIRECTORIES "${saved_include_directories}"
      LINK_DIRECTORIES "${saved_link_directories}"
      SYSTEM_INCLUDE_DIRECTORIES "${saved_system_include_directories}"
  )

  set(plugin "${STEPIT_PLUGIN_NAME}")
  stepit_set_plugin_property(${plugin} DIR "${plugin_dir}")
  stepit_set_plugin_property(${plugin} DEPENDS ${STEPIT_PLUGIN_DEPENDS})
  stepit_set_plugin_property(${plugin} BUILDABLE ${STEPIT_PLUGIN_BUILDABLE})
  stepit_set_plugin_property(${plugin} REASON "${STEPIT_PLUGIN_REASON}")
  set(${output_var} "${plugin}" PARENT_SCOPE)
endfunction()

function(stepit_resolve_plugin plugin dependency_stack)
  if (NOT plugin IN_LIST STEPIT_DISCOVERED_PLUGINS)
    message(FATAL_ERROR "Plugin '${plugin}' is not supported.")
  endif ()

  stepit_append_unique_global_property(STEPIT_RESOLVED_PLUGINS ${plugin})

  stepit_get_plugin_property(${plugin} STATUS current_status)
  if (current_status STREQUAL "buildable" OR
      current_status STREQUAL "built" OR
      current_status STREQUAL "skipped")
    return()
  endif ()

  if (plugin IN_LIST dependency_stack)
    list(APPEND dependency_stack ${plugin})
    string(REPLACE ";" " -> " cycle_text "${dependency_stack}")
    message(FATAL_ERROR "Detected plugin dependency cycle: ${cycle_text}")
  endif ()

  list(APPEND dependency_stack ${plugin})

  if (NOT plugin IN_LIST STEPIT_ENABLED_PLUGINS)
    stepit_set_plugin_property(${plugin} STATUS "skipped")
    stepit_set_plugin_property(${plugin} REASON "Plugin '${plugin}' is blacklisted.")
    stepit_set_plugin_property(${plugin} CHAIN ${plugin})
    return()
  endif ()

  stepit_get_plugin_property(${plugin} BUILDABLE plugin_buildable)
  stepit_get_plugin_property(${plugin} REASON plugin_reason)
  if (NOT plugin_buildable)
    stepit_set_plugin_property(${plugin} STATUS "skipped")
    stepit_set_plugin_property(${plugin} REASON "${plugin_reason}")
    stepit_set_plugin_property(${plugin} CHAIN ${plugin})
    return()
  endif ()

  stepit_get_plugin_property(${plugin} DEPENDS plugin_dependencies)
  foreach (dependency ${plugin_dependencies})
    if (NOT dependency IN_LIST STEPIT_DISCOVERED_PLUGINS)
      message(
          FATAL_ERROR
          "Plugin '${plugin}' declares unknown dependency '${dependency}'."
      )
    endif ()

    stepit_resolve_plugin(${dependency} "${dependency_stack}")
    stepit_get_plugin_property(${dependency} STATUS dependency_status)
    if (NOT dependency_status STREQUAL "buildable" AND
        NOT dependency_status STREQUAL "built")
      stepit_mark_plugin_skipped_due_to_dependency(${plugin} ${dependency})
      return()
    endif ()
  endforeach ()

  stepit_set_plugin_property(${plugin} STATUS "buildable")
  stepit_set_plugin_property(${plugin} REASON "")
  stepit_set_plugin_property(${plugin} CHAIN ${plugin})
  stepit_append_unique_global_property(STEPIT_BUILDABLE_PLUGIN_ORDER ${plugin})
endfunction()

function(stepit_mark_plugin_skipped_due_to_dependency plugin dependency)
  stepit_set_plugin_property(${plugin} STATUS "skipped")
  stepit_set_plugin_property(${plugin} REASON "Dependency plugin '${dependency}' is skipped.")
  stepit_set_plugin_property(${plugin} CHAIN ${plugin} ${dependency})
endfunction()

function(stepit_print_plugin_report)
  get_property(built_plugins GLOBAL PROPERTY STEPIT_BUILT_PLUGINS)
  if (NOT built_plugins)
    set(built_plugins "")
  endif ()
  set(skipped_plugins "")

  foreach (plugin ${ARGN})
    stepit_get_plugin_property(${plugin} STATUS plugin_status)
    if (plugin_status STREQUAL "skipped")
      list(APPEND skipped_plugins ${plugin})
    endif ()
  endforeach ()

  if (built_plugins)
    message(STATUS "Built stepit plugins in order:")
    foreach (plugin ${built_plugins})
      message(STATUS "  ${plugin}")
    endforeach ()
  else ()
    message(STATUS "Built stepit plugins in order: none")
  endif ()

  if (skipped_plugins)
    message(STATUS "Skipped stepit plugins:")
    foreach (plugin ${skipped_plugins})
      stepit_get_plugin_property(${plugin} REASON plugin_reason)
      stepit_get_plugin_property(${plugin} CHAIN plugin_chain)
      if (plugin_chain)
        string(REPLACE ";" " -> " plugin_chain_text "${plugin_chain}")
        message(STATUS "  ${plugin_chain_text}: ${plugin_reason}")
      else ()
        message(STATUS "  ${plugin}: ${plugin_reason}")
      endif ()
    endforeach ()
  endif ()
endfunction()

function(init_submodule submodule_name)
  find_package(Git REQUIRED)

  set(options "RECURSIVE")
  set(oneValueArgs "WORKING_DIRECTORY")
  set(multiValueArgs "")

  cmake_parse_arguments(PARSE_ARGV 1 ARG
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
  )
  if (ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unparsed arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif ()
  if (NOT ARG_WORKING_DIRECTORY)
    set(ARG_WORKING_DIRECTORY ${CMAKE_HOME_DIRECTORY})
  endif ()

  # Check if the submodule is already initialized
  execute_process(
      COMMAND ${GIT_EXECUTABLE} submodule status ${submodule_name}
      WORKING_DIRECTORY ${ARG_WORKING_DIRECTORY}
      OUTPUT_VARIABLE status_output
      OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if (status_output MATCHES "^-.*")
    message(STATUS "Initializing submodule: ${submodule_name}")
    if (ARG_RECURSIVE)
      set(recursive_flag "--recursive")
    endif ()
    execute_process(
        COMMAND ${GIT_EXECUTABLE} submodule update --init ${recursive_flag} ${submodule_name}
        WORKING_DIRECTORY ${ARG_WORKING_DIRECTORY}
        RESULT_VARIABLE update_result
    )

    if (NOT update_result EQUAL "0")
      message(FATAL_ERROR "git submodule update failed for ${submodule_name}")
    endif ()
  endif ()
endfunction()
