cmake_minimum_required(VERSION 3.10)
project(tinympc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set paths
set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../../../../../eigen")
set(EIGEN3_LIB_DIR "${CMAKE_SOURCE_DIR}/../../../../../eigen/build/lib")  # Adjust to where Eigen is compiled if needed

# Include Eigen headers
if (EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Dense")
    include_directories(${EIGEN3_INCLUDE_DIR})
else()
    message(FATAL_ERROR "Eigen headers not found in ${EIGEN3_INCLUDE_DIR}.")
endif()

# Add static library
add_library(tinympcstatic STATIC
    admm.cpp
    tiny_api.cpp
    codegen.cpp
)

# Add shared library
add_library(tinympc SHARED
    admm.cpp
    tiny_api.cpp
    codegen.cpp
)

# Link the static library to the shared library
target_link_libraries(tinympc PRIVATE tinympcstatic)

# Link Eigen library (if compiled as a library, optional)
if (EXISTS "${EIGEN3_LIB_DIR}")
    target_link_directories(tinympc PUBLIC ${EIGEN3_LIB_DIR})
    target_link_libraries(tinympc PUBLIC Eigen3::Eigen)
endif()

# Include directories for both libraries
target_include_directories(tinympcstatic PUBLIC ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_include_directories(tinympc PUBLIC ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Define export macros for shared library
target_compile_definitions(tinympc PRIVATE BUILD_DLL)
