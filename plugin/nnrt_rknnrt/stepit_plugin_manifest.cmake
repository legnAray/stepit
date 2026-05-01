stepit_declare_plugin(NAME nnrt_rknnrt DEPENDS
    nnrt_base
)
get_library_directory(rknnrt RKNNRT_DIR VERBOSE)
if (NOT RKNNRT_DIR)
  stepit_plugin_mark_unbuildable("Missing rknnrt.")
  return()
endif ()
