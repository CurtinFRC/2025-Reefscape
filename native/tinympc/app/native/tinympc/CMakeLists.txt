cmake_minimum_required(VERSION 3.10)
project(tinympc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Path to Eigen library
set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../../../../../eigen")

# Include Eigen headers manually
include_directories(${EIGEN3_INCLUDE_DIR})

# Add static library
add_library(tinympcstatic STATIC
    admm.cpp
    tiny_api.cpp
    codegen.cpp
)

# Add shared library (reuse static library)
add_library(tinympc SHARED
    admm.cpp
    tiny_api.cpp
    codegen.cpp
)

# Link static library to shared library
target_link_libraries(tinympc PRIVATE tinympcstatic)

# Include directories for both libraries
target_include_directories(tinympcstatic PUBLIC ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_include_directories(tinympc PUBLIC ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Define export macros for shared library
target_compile_definitions(tinympc PRIVATE BUILD_DLL)
