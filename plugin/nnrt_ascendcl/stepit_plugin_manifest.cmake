stepit_declare_plugin(NAME nnrt_ascendcl DEPENDS
    nnrt_base
)
get_library_directory(ascendcl ASCENDCL_DIR)
if (NOT ASCENDCL_DIR)
  stepit_plugin_mark_unbuildable("Missing AscendCL.")
  return()
endif ()
