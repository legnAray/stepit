cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_nnrt_tensorrt)

if (POLICY CMP0146)
  cmake_policy(PUSH)
  cmake_policy(SET CMP0146 OLD)
endif ()

find_package(CUDA REQUIRED)

if (POLICY CMP0146)
  cmake_policy(POP)
endif ()
get_library_directory(nvinfer TENSORRT_LIB_DIR REQUIRED)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/tensorrt.cpp
    INCLUDES
      include
      ${CUDA_INCLUDE_DIRS}
    LINK_DIRS
      ${TENSORRT_LIB_DIR}
    LINK_LIBS
      ${CUDA_LIBRARIES}
      nvinfer
      nvonnxparser
    DEPENDS
      nnrt_base
)

set(TENSORRT_INCLUDE_DIR ${TENSORRT_LIB_DIR}/../include)
if (EXISTS "${TENSORRT_INCLUDE_DIR}")
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${TENSORRT_INCLUDE_DIR})
endif ()
