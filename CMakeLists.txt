cmake_minimum_required(VERSION 3.25)
project(draxi LANGUAGES C CXX)

# Default to Release for single-config generators: Eigen and MuJoCo control loops are slow unoptimised
get_property(DRAXI_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT DRAXI_MULTI_CONFIG AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT EXISTS ${CMAKE_SOURCE_DIR}/libs/legged-gait-core/CMakeLists.txt)
  message(FATAL_ERROR "libs/legged-gait-core is empty. Run: git submodule update --init --recursive")
endif()

# SYSTEM: gait-core's headers are a library's, so their warnings don't show in our builds
add_subdirectory(libs/legged-gait-core SYSTEM)
add_subdirectory(shared)
add_subdirectory(control)
add_subdirectory(sim)
