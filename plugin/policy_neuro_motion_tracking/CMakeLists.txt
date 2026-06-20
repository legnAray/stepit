cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_policy_neuro_motion_tracking)

find_package(pinocchio REQUIRED)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/forward_kinematics.cpp
      src/geometry_operator.cpp
      src/motion_clip.cpp
      src/motion_player.cpp
      src/relative_pose_source.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/policy_neuro/forward_kinematics.h
        include/stepit/policy_neuro/geometry_operator.h
        include/stepit/policy_neuro/motion_clip.h
        include/stepit/policy_neuro/motion_player.h
        include/stepit/policy_neuro/relative_pose_source.h
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_policy_neuro
    stepit_plugin_pyutils
    pinocchio::pinocchio
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
target_compile_definitions(${PROJECT_NAME} PUBLIC COAL_DISABLE_HPP_FCL_WARNINGS)
