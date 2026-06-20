cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_policy_neuro)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/action_source.cpp
      src/actuator.cpp
      src/cmd_posture_source.cpp
      src/cmd_vel_source.cpp
      src/field_ops.cpp
      src/heightmap_source.cpp
      src/joint_reordering.cpp
      src/module.cpp
      src/neuro_module.cpp
      src/neuro_policy.cpp
      src/odometry_source.cpp
      src/proprioceptor.cpp
      src/subscriber_action.cpp
      src/time_source.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/policy_neuro/action_source.h
        include/stepit/policy_neuro/actuator.h
        include/stepit/policy_neuro/cmd_posture_source.h
        include/stepit/policy_neuro/cmd_vel_source.h
        include/stepit/policy_neuro/field_ops.h
        include/stepit/policy_neuro/heightmap_source.h
        include/stepit/policy_neuro/joint_reordering.h
        include/stepit/policy_neuro/module.h
        include/stepit/policy_neuro/neuro_module.h
        include/stepit/policy_neuro/neuro_policy.h
        include/stepit/policy_neuro/odometry_source.h
        include/stepit/policy_neuro/proprioceptor.h
        include/stepit/policy_neuro/subscriber_action.h
        include/stepit/policy_neuro/time_source.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_field_base
    stepit_plugin_joystick_base
    stepit_plugin_nnrt_base
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
