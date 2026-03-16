cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_field_base)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/field.cpp
      src/data_loader.cpp
      src/operator.cpp
    INCLUDES
      include
)

stepit_add_executable(data_info app/data_info.cpp)
target_link_libraries(data_info PRIVATE ${PROJECT_NAME})
